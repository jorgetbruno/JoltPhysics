#include <Debug/JoltDebugRenderer.h>

#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/hash.h>

#include <Utils/Conversions.h>

namespace JoltPhysics
{
    namespace
    {
        AZ::Color ToAzColor(JPH::ColorArg color)
        {
            return AZ::Color(color.r / 255.0f, color.g / 255.0f, color.b / 255.0f, color.a / 255.0f);
        }

        //! One shape's geometry, prepared once for wireframe and solid drawing.
        //!
        //! Local-space, in runs of one colour. Colours are the vertex colours Jolt gave the
        //! triangles; the per-frame model colour multiplies them at draw time, so a body
        //! changing colour as it sleeps and wakes does not invalidate the batch.
        class JoltDebugBatch final : public JPH::RefTargetVirtual
        {
        public:
            JPH_OVERRIDE_NEW_DELETE

            struct Run
            {
                JPH::Color m_color;
                AZ::u32 m_begin = 0;
                AZ::u32 m_end = 0;
            };

            void AddRef() override { ++m_refCount; }
            void Release() override
            {
                if (--m_refCount == 0)
                {
                    delete this;
                }
            }

            //! Unique edges as endpoint pairs, grouped by colour.
            JPH::Array<JPH::Vec3> m_edgePoints;
            JPH::Array<Run> m_edgeRuns;

            //! Every triangle as three points, grouped by colour, for solid drawing.
            JPH::Array<JPH::Vec3> m_trianglePoints;
            JPH::Array<Run> m_triangleRuns;

            AZ::u32 m_triangleCount = 0;

        private:
            std::atomic<JPH::uint32> m_refCount = 0;
        };

        //! An undirected edge by its endpoints' exact bits. Exact is right here, not a
        //! tolerance: two triangles share an edge because Jolt copied the same vertex into
        //! both, so the positions are bit-identical, and a tolerance would merge edges that
        //! merely happen to lie close together.
        struct EdgeKey
        {
            JPH::Float3 m_a;
            JPH::Float3 m_b;

            bool operator==(const EdgeKey& other) const
            {
                return m_a == other.m_a && m_b == other.m_b;
            }
        };

        bool LessThan(const JPH::Float3& a, const JPH::Float3& b)
        {
            if (a.x != b.x)
            {
                return a.x < b.x;
            }
            if (a.y != b.y)
            {
                return a.y < b.y;
            }
            return a.z < b.z;
        }

        EdgeKey MakeEdgeKey(const JPH::Float3& a, const JPH::Float3& b)
        {
            return LessThan(b, a) ? EdgeKey{ b, a } : EdgeKey{ a, b };
        }

        struct EdgeKeyHash
        {
            size_t operator()(const EdgeKey& key) const
            {
                size_t seed = 0;
                for (const float value : { key.m_a.x, key.m_a.y, key.m_a.z, key.m_b.x, key.m_b.y, key.m_b.z })
                {
                    AZ::u32 bits = 0;
                    memcpy(&bits, &value, sizeof(bits));
                    AZStd::hash_combine(seed, bits);
                }
                return seed;
            }
        };

        //! Builds the batch from a flat triangle list: every triangle kept for solid
        //! drawing, and each edge kept once for wireframe. A shared edge keeps the colour of
        //! the first triangle that reached it, which only matters for a mesh painted in
        //! several colours, and then only along the seams.
        JPH::DebugRenderer::Batch BuildBatch(const JPH::DebugRenderer::Triangle* triangles, int triangleCount)
        {
            auto* batch = new JoltDebugBatch();
            if (triangles == nullptr || triangleCount <= 0)
            {
                return batch;
            }
            batch->m_triangleCount = static_cast<AZ::u32>(triangleCount);

            // Bucket by colour first so every run is contiguous; most shapes are a single
            // colour and this is one bucket.
            AZStd::unordered_map<JPH::uint32, AZStd::vector<int>> trianglesByColor;
            for (int i = 0; i < triangleCount; ++i)
            {
                trianglesByColor[triangles[i].mV[0].mColor.GetUInt32()].push_back(i);
            }

            AZStd::unordered_set<EdgeKey, EdgeKeyHash> seenEdges;
            seenEdges.reserve(static_cast<size_t>(triangleCount) * 2);
            batch->m_trianglePoints.reserve(static_cast<size_t>(triangleCount) * 3);
            batch->m_edgePoints.reserve(static_cast<size_t>(triangleCount) * 3);

            for (const auto& [packedColor, indices] : trianglesByColor)
            {
                const JPH::Color color = triangles[indices.front()].mV[0].mColor;

                JoltDebugBatch::Run triangleRun{ color, static_cast<AZ::u32>(batch->m_trianglePoints.size()), 0 };
                JoltDebugBatch::Run edgeRun{ color, static_cast<AZ::u32>(batch->m_edgePoints.size()), 0 };

                for (const int index : indices)
                {
                    const JPH::DebugRenderer::Triangle& triangle = triangles[index];
                    for (int corner = 0; corner < 3; ++corner)
                    {
                        batch->m_trianglePoints.push_back(JPH::Vec3(triangle.mV[corner].mPosition));
                    }
                    for (int corner = 0; corner < 3; ++corner)
                    {
                        const JPH::Float3& from = triangle.mV[corner].mPosition;
                        const JPH::Float3& to = triangle.mV[(corner + 1) % 3].mPosition;
                        if (seenEdges.insert(MakeEdgeKey(from, to)).second)
                        {
                            batch->m_edgePoints.push_back(JPH::Vec3(from));
                            batch->m_edgePoints.push_back(JPH::Vec3(to));
                        }
                    }
                }

                triangleRun.m_end = static_cast<AZ::u32>(batch->m_trianglePoints.size());
                edgeRun.m_end = static_cast<AZ::u32>(batch->m_edgePoints.size());
                batch->m_triangleRuns.push_back(triangleRun);
                if (edgeRun.m_end > edgeRun.m_begin)
                {
                    batch->m_edgeRuns.push_back(edgeRun);
                }
            }
            return batch;
        }
    } // namespace

    void JoltDebugDrawSink::Clear()
    {
        for (auto& [color, points] : m_linesByColor)
        {
            points.clear();
        }
        for (auto& [color, points] : m_trianglesByColor)
        {
            points.clear();
        }
    }

    JoltDebugRenderer::JoltDebugRenderer()
    {
        // Required by Jolt's contract: builds the unit box, sphere, capsule and cylinder the
        // shapes draw themselves with. Once, for the life of this renderer.
        Initialize();
    }

    void JoltDebugRenderer::SetTarget(const Physics::DebugDrawSettings* settings, JoltDebugDrawSink* sink)
    {
        m_settings = settings;
        m_sink = sink;
    }

    void JoltDebugRenderer::SetCameraPosition(const AZ::Vector3& cameraPosition)
    {
        m_cameraPosition = Conversions::ToJolt(cameraPosition);
        m_cameraPositionSet = true;
    }

    void JoltDebugRenderer::ResetCounters()
    {
        m_geometryDraws = 0;
        m_geometryTriangles = 0;
        m_lines = 0;
    }

    void JoltDebugRenderer::DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor)
    {
        ++m_lines;
        const AZ::Color color = ToAzColor(inColor);
        if (m_sink)
        {
            AZStd::vector<AZ::Vector3>& points = m_sink->m_linesByColor[color.ToU32()];
            points.push_back(Conversions::FromJolt(inFrom));
            points.push_back(Conversions::FromJolt(inTo));
            return;
        }
        if (m_settings && m_settings->m_drawLineCB)
        {
            // DebugDrawSettings' draw helpers are non-const; the settings are not modified.
            const_cast<Physics::DebugDrawSettings*>(m_settings)->DrawLine(
                Physics::DebugDrawVertex(Conversions::FromJolt(inFrom), color),
                Physics::DebugDrawVertex(Conversions::FromJolt(inTo), color),
                nullptr,
                1.0f);
        }
    }

    void JoltDebugRenderer::DrawTriangle(
        JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3, JPH::ColorArg inColor,
        [[maybe_unused]] ECastShadow inCastShadow)
    {
        const AZ::Color color = ToAzColor(inColor);
        if (m_sink)
        {
            AZStd::vector<AZ::Vector3>& points = m_sink->m_trianglesByColor[color.ToU32()];
            points.push_back(Conversions::FromJolt(inV1));
            points.push_back(Conversions::FromJolt(inV2));
            points.push_back(Conversions::FromJolt(inV3));
            return;
        }
        if (m_settings && m_settings->m_drawTriBatchCB)
        {
            const Physics::DebugDrawVertex vertices[] = {
                Physics::DebugDrawVertex(Conversions::FromJolt(inV1), color),
                Physics::DebugDrawVertex(Conversions::FromJolt(inV2), color),
                Physics::DebugDrawVertex(Conversions::FromJolt(inV3), color),
            };
            const AZ::u32 indices[] = { 0, 1, 2 };
            const_cast<Physics::DebugDrawSettings*>(m_settings)->DrawTriangleBatch(vertices, 3, indices, 3, nullptr);
        }
    }

    JPH::DebugRenderer::Batch JoltDebugRenderer::CreateTriangleBatch(const Triangle* inTriangles, int inTriangleCount)
    {
        return BuildBatch(inTriangles, inTriangleCount);
    }

    JPH::DebugRenderer::Batch JoltDebugRenderer::CreateTriangleBatch(
        const Vertex* inVertices, int inVertexCount, const JPH::uint32* inIndices, int inIndexCount)
    {
        if (inVertices == nullptr || inVertexCount <= 0 || inIndices == nullptr || inIndexCount < 3)
        {
            return BuildBatch(nullptr, 0);
        }
        JPH::Array<Triangle> triangles;
        triangles.resize(static_cast<size_t>(inIndexCount / 3));
        for (size_t t = 0; t < triangles.size(); ++t)
        {
            for (size_t corner = 0; corner < 3; ++corner)
            {
                triangles[t].mV[corner] = inVertices[inIndices[t * 3 + corner]];
            }
        }
        return BuildBatch(triangles.data(), static_cast<int>(triangles.size()));
    }

    void JoltDebugRenderer::DrawGeometry(
        JPH::RMat44Arg inModelMatrix, const JPH::AABox& inWorldSpaceBounds, float inLODScaleSq,
        JPH::ColorArg inModelColor, const GeometryRef& inGeometry, [[maybe_unused]] ECullMode inCullMode,
        ECastShadow inCastShadow, EDrawMode inDrawMode)
    {
        if (inGeometry == nullptr || inGeometry->mLODs.empty())
        {
            return;
        }
        const LOD& lod = m_cameraPositionSet
            ? inGeometry->GetLOD(m_cameraPosition, inWorldSpaceBounds, inLODScaleSq)
            : inGeometry->mLODs.front();
        const auto* batch = static_cast<const JoltDebugBatch*>(lod.mTriangleBatch.GetPtr());
        if (batch == nullptr)
        {
            return;
        }

        ++m_geometryDraws;
        m_geometryTriangles += batch->m_triangleCount;

        const bool wireframe = inDrawMode == EDrawMode::Wireframe;
        const JPH::Array<JPH::Vec3>& points = wireframe ? batch->m_edgePoints : batch->m_trianglePoints;
        const JPH::Array<JoltDebugBatch::Run>& runs = wireframe ? batch->m_edgeRuns : batch->m_triangleRuns;

        if (m_sink)
        {
            // The fast path, and the whole reason this class exists: one colour lookup per
            // run, then a transform per point straight into the buffer the flush reads.
            for (const JoltDebugBatch::Run& run : runs)
            {
                const AZ::u32 color = ToAzColor(inModelColor * run.m_color).ToU32();
                AZStd::vector<AZ::Vector3>& out =
                    wireframe ? m_sink->m_linesByColor[color] : m_sink->m_trianglesByColor[color];
                out.reserve(out.size() + (run.m_end - run.m_begin));
                for (AZ::u32 i = run.m_begin; i < run.m_end; ++i)
                {
                    out.push_back(Conversions::FromJolt(JPH::RVec3(inModelMatrix * points[i])));
                }
            }
            if (wireframe)
            {
                m_lines += (points.size() / 2);
            }
            return;
        }

        // No sink: the caller's callbacks, one primitive at a time, as the bus contract has it.
        for (const JoltDebugBatch::Run& run : runs)
        {
            const JPH::Color color = inModelColor * run.m_color;
            if (wireframe)
            {
                for (AZ::u32 i = run.m_begin; i + 1 < run.m_end; i += 2)
                {
                    DrawLine(inModelMatrix * points[i], inModelMatrix * points[i + 1], color);
                }
            }
            else
            {
                for (AZ::u32 i = run.m_begin; i + 2 < run.m_end; i += 3)
                {
                    DrawTriangle(inModelMatrix * points[i], inModelMatrix * points[i + 1], inModelMatrix * points[i + 2],
                        color, inCastShadow);
                }
            }
        }
    }
} // namespace JoltPhysics
