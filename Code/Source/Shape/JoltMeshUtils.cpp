#include <Shape/JoltMeshUtils.h>
#include <Utils/Conversions.h>

#include <AzCore/Math/Vector3.h>
#include <AzCore/std/algorithm.h>

#include <Jolt/Jolt.h>
#include <Jolt/Math/Float3.h>
#include <Jolt/Geometry/IndexedTriangle.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>

namespace JoltPhysics
{
    namespace
    {
        // Simple packed format for the "cooked" blob. Jolt needs no real cooking pass
        // (MeshShapeSettings::Create() builds its BVH from the raw triangle list), so
        // this is just a header followed by the vertex and index arrays.
        struct MeshBlobHeader
        {
            AZ::u32 m_magic = 0x4C4A4D48; // 'HMJL' ("Jolt Mesh")
            AZ::u32 m_version = 1;
            AZ::u32 m_vertexCount = 0;
            AZ::u32 m_indexCount = 0; // always a multiple of 3
        };

        // Mesh blob versions: v1 is vertices + indices, v2 appends a u32 table size
        // and a per-face u8 material index table (see PackTriangleMesh's overload).
        // Jolt's tree packs material indices as 5 bits per triangle (FLAGS_MATERIAL_BITS),
        // so 31 is the largest index that survives the round trip into a MeshShape.
        constexpr AZ::u32 cMeshBlobVersionNoMaterials = 1;
        constexpr AZ::u32 cMeshBlobVersionPerFaceMaterials = 2;
        constexpr AZ::u8 cMaxMeshMaterialIndex = 31;

        // Distinct magic from MeshBlobHeader so a convex blob fed to the triangle-mesh
        // decoder (or vice versa) fails loudly instead of misinterpreting the bytes.
        struct ConvexBlobHeader
        {
            AZ::u32 m_magic = 0x4A435648; // 'HVCJ' ("Jolt Convex Hull")
            AZ::u32 m_version = 1;
            AZ::u32 m_vertexCount = 0;
        };

        // Convex blob versions: v1 is one flat point cloud (a single hull), v2 is a
        // u32 hull count followed by per-hull (u32 vertex count + float3 vertices),
        // with the header's vertex count carrying the total across all hulls.
        constexpr AZ::u32 cConvexBlobVersionSingle = 1;
        constexpr AZ::u32 cConvexBlobVersionHullGroup = 2;

        // Builds one convex hull from a point cloud, reporting Jolt's error on failure.
        JPH::RefConst<JPH::Shape> CreateHullShape(const JPH::Array<JPH::Vec3>& points)
        {
            JPH::ConvexHullShapeSettings settings(points);
            JPH::ShapeSettings::ShapeResult result = settings.Create();
            if (result.HasError())
            {
                AZ_Error("JoltPhysics", false, "Failed to create convex hull shape: %s", result.GetError().c_str());
                return nullptr;
            }
            return result.Get();
        }

        // Quiet version of CreateHullShape for cook-time validation: can these points
        // ever become a hull? (Jolt needs at least 4 non-coplanar points.)
        bool CanBuildHull(const AZStd::vector<AZ::Vector3>& points)
        {
            if (points.size() < 4)
            {
                return false;
            }
            JPH::Array<JPH::Vec3> joltPoints;
            joltPoints.reserve(points.size());
            for (const AZ::Vector3& point : points)
            {
                joltPoints.push_back(JPH::Vec3(point.GetX(), point.GetY(), point.GetZ()));
            }
            JPH::ConvexHullShapeSettings settings(joltPoints);
            return settings.Create().IsValid();
        }

        // Reads one v2 hull (vertex count + float3 vertices) off the cursor. Returns
        // false when the blob ends early; the caller then reports it as malformed.
        bool ReadHullPoints(const AZ::u8*& cursor, const AZ::u8* end, JPH::Array<JPH::Vec3>& outPoints)
        {
            if (static_cast<size_t>(end - cursor) < sizeof(AZ::u32))
            {
                return false;
            }
            AZ::u32 vertexCount;
            memcpy(&vertexCount, cursor, sizeof(vertexCount));
            cursor += sizeof(vertexCount);

            const size_t vertexBytes = static_cast<size_t>(vertexCount) * 3 * sizeof(float);
            if (vertexCount == 0 || static_cast<size_t>(end - cursor) < vertexBytes)
            {
                return false;
            }
            outPoints.reserve(vertexCount);
            for (AZ::u32 i = 0; i < vertexCount; ++i)
            {
                float xyz[3];
                memcpy(xyz, cursor + static_cast<size_t>(i) * sizeof(xyz), sizeof(xyz));
                outPoints.push_back(JPH::Vec3(xyz[0], xyz[1], xyz[2]));
            }
            cursor += vertexBytes;
            return true;
        }

        // Appends one geometry entry's vertices (brought into entity-local space) to
        // outPoints. Entries with malformed data are skipped and leave the outputs
        // untouched. When outIndices is given the entry's indices (re-based) are appended
        // too, and an out-of-range index rejects the whole entry with a warning.
        bool GatherEntryPoints(
            const AzFramework::VisibleGeometry& geometry,
            const AZ::Transform& worldToEntity,
            AZStd::vector<AZ::Vector3>& outPoints,
            AZStd::vector<AZ::u32>* outIndices)
        {
            if (geometry.m_vertices.size() < 9 || geometry.m_vertices.size() % 3 != 0 ||
                geometry.m_indices.size() < 3 || geometry.m_indices.size() % 3 != 0)
            {
                return false;
            }

            const AZ::u32 baseVertex = static_cast<AZ::u32>(outPoints.size());
            const size_t vertexCount = geometry.m_vertices.size() / 3;
            outPoints.reserve(outPoints.size() + vertexCount);
            for (size_t i = 0; i < vertexCount; ++i)
            {
                const AZ::Vector3 localVertex(
                    geometry.m_vertices[i * 3 + 0], geometry.m_vertices[i * 3 + 1], geometry.m_vertices[i * 3 + 2]);
                const AZ::Vector3 worldVertex = geometry.m_transform * localVertex;
                outPoints.push_back(worldToEntity.TransformPoint(worldVertex));
            }

            if (!outIndices)
            {
                return true;
            }

            const size_t baseIndexCount = outIndices->size();
            outIndices->reserve(baseIndexCount + geometry.m_indices.size());
            for (const uint32_t index : geometry.m_indices)
            {
                if (index >= vertexCount)
                {
                    AZ_Warning("JoltPhysics", false,
                        "JoltMeshUtils: geometry entry has an out-of-range index (%u >= %zu); the entry is skipped.",
                        index, vertexCount);
                    outPoints.resize(baseVertex);
                    outIndices->resize(baseIndexCount);
                    return false;
                }
                outIndices->push_back(baseVertex + index);
            }
            return true;
        }
    }

    AZStd::vector<AZ::u8> JoltMeshUtils::PackTriangleMesh(
        const AZ::Vector3* vertices,
        AZ::u32 vertexCount,
        const AZ::u32* indices,
        AZ::u32 indexCount)
    {
        AZStd::vector<AZ::u8> result;
        if (!vertices || vertexCount == 0 || !indices || indexCount == 0 || indexCount % 3 != 0)
        {
            return result;
        }

        MeshBlobHeader header;
        header.m_vertexCount = vertexCount;
        header.m_indexCount = indexCount;

        const size_t vertexBytes = static_cast<size_t>(vertexCount) * 3 * sizeof(float);
        const size_t indexBytes = static_cast<size_t>(indexCount) * sizeof(AZ::u32);
        result.reserve(sizeof(MeshBlobHeader) + vertexBytes + indexBytes);

        const auto* headerBytes = reinterpret_cast<const AZ::u8*>(&header);
        result.insert(result.end(), headerBytes, headerBytes + sizeof(MeshBlobHeader));

        for (AZ::u32 i = 0; i < vertexCount; ++i)
        {
            const float xyz[3] = { vertices[i].GetX(), vertices[i].GetY(), vertices[i].GetZ() };
            const auto* bytes = reinterpret_cast<const AZ::u8*>(xyz);
            result.insert(result.end(), bytes, bytes + sizeof(xyz));
        }

        const auto* indexBytesPtr = reinterpret_cast<const AZ::u8*>(indices);
        result.insert(result.end(), indexBytesPtr, indexBytesPtr + indexBytes);

        return result;
    }

    AZStd::vector<AZ::u8> JoltMeshUtils::PackTriangleMesh(
        const AZ::Vector3* vertices,
        AZ::u32 vertexCount,
        const AZ::u32* indices,
        AZ::u32 indexCount,
        const AZ::u8* materialIndices,
        AZ::u32 faceCount)
    {
        if (!materialIndices || faceCount != indexCount / 3)
        {
            // No table (or one that cannot match the faces) is exactly the v1 blob.
            return PackTriangleMesh(vertices, vertexCount, indices, indexCount);
        }

        AZStd::vector<AZ::u8> result = PackTriangleMesh(vertices, vertexCount, indices, indexCount);
        if (result.empty())
        {
            return result;
        }

        // Upgrade the header to v2 and append the table: u32 count, then one u8 per face.
        auto* header = reinterpret_cast<MeshBlobHeader*>(result.data());
        header->m_version = cMeshBlobVersionPerFaceMaterials;

        const auto* countBytes = reinterpret_cast<const AZ::u8*>(&faceCount);
        result.insert(result.end(), countBytes, countBytes + sizeof(faceCount));
        result.insert(result.end(), materialIndices, materialIndices + faceCount);

        return result;
    }

    bool JoltMeshUtils::UnpackTriangleMesh(
        const AZStd::vector<AZ::u8>& cookedData,
        AZStd::vector<AZ::Vector3>& outVertices,
        AZStd::vector<AZ::u32>& outIndices)
    {
        outVertices.clear();
        outIndices.clear();

        if (cookedData.size() < sizeof(MeshBlobHeader))
        {
            return false;
        }

        MeshBlobHeader header;
        memcpy(&header, cookedData.data(), sizeof(MeshBlobHeader));

        if (header.m_magic != MeshBlobHeader().m_magic ||
            (header.m_version != cMeshBlobVersionNoMaterials && header.m_version != cMeshBlobVersionPerFaceMaterials) ||
            header.m_vertexCount == 0 || header.m_indexCount == 0 || header.m_indexCount % 3 != 0)
        {
            return false;
        }

        const size_t vertexBytes = static_cast<size_t>(header.m_vertexCount) * 3 * sizeof(float);
        const size_t indexBytes = static_cast<size_t>(header.m_indexCount) * sizeof(AZ::u32);
        if (cookedData.size() < sizeof(MeshBlobHeader) + vertexBytes + indexBytes)
        {
            return false;
        }

        const AZ::u8* vertexCursor = cookedData.data() + sizeof(MeshBlobHeader);
        outVertices.reserve(header.m_vertexCount);
        for (AZ::u32 i = 0; i < header.m_vertexCount; ++i)
        {
            float xyz[3];
            memcpy(xyz, vertexCursor + static_cast<size_t>(i) * sizeof(xyz), sizeof(xyz));
            outVertices.emplace_back(xyz[0], xyz[1], xyz[2]);
        }

        outIndices.resize(header.m_indexCount);
        memcpy(outIndices.data(), vertexCursor + vertexBytes, indexBytes);

        for (AZ::u32 index : outIndices)
        {
            if (index >= header.m_vertexCount)
            {
                outVertices.clear();
                outIndices.clear();
                return false;
            }
        }
        return true;
    }

    JPH::RefConst<JPH::Shape> JoltMeshUtils::CreateMeshShapeFromCookedData(const AZStd::vector<AZ::u8>& cookedData)
    {
        if (cookedData.size() < sizeof(MeshBlobHeader))
        {
            return nullptr;
        }

        MeshBlobHeader header;
        memcpy(&header, cookedData.data(), sizeof(MeshBlobHeader));

        if (header.m_magic != MeshBlobHeader().m_magic ||
            (header.m_version != cMeshBlobVersionNoMaterials && header.m_version != cMeshBlobVersionPerFaceMaterials) ||
            header.m_vertexCount == 0 || header.m_indexCount == 0 || header.m_indexCount % 3 != 0)
        {
            AZ_Error("JoltPhysics", false,
                "JoltMeshUtils: cooked mesh blob is malformed or from an incompatible version "
                "(magic=0x%08X version=%u vertexCount=%u indexCount=%u blobSize=%zu)",
                header.m_magic, header.m_version, header.m_vertexCount, header.m_indexCount, cookedData.size());
            return nullptr;
        }

        const size_t vertexBytes = static_cast<size_t>(header.m_vertexCount) * 3 * sizeof(float);
        const size_t indexBytes = static_cast<size_t>(header.m_indexCount) * sizeof(AZ::u32);
        const AZ::u32 triangleCount = header.m_indexCount / 3;
        const bool hasMaterials = header.m_version == cMeshBlobVersionPerFaceMaterials;
        const size_t materialBytes = hasMaterials ? sizeof(AZ::u32) + triangleCount : 0;
        const size_t expectedSize = sizeof(MeshBlobHeader) + vertexBytes + indexBytes + materialBytes;
        if (cookedData.size() != expectedSize)
        {
            AZ_Error("JoltPhysics", false, "JoltMeshUtils: cooked mesh blob size mismatch (expected %zu, got %zu)",
                expectedSize, cookedData.size());
            return nullptr;
        }

        const AZ::u8* vertexCursor = cookedData.data() + sizeof(MeshBlobHeader);
        const AZ::u8* indexCursor = vertexCursor + vertexBytes;

        // v2 blobs carry the per-face material table after the index array.
        const AZ::u8* materialCursor = nullptr;
        if (hasMaterials)
        {
            AZ::u32 materialIndexCount = 0;
            const AZ::u8* countCursor = indexCursor + indexBytes;
            memcpy(&materialIndexCount, countCursor, sizeof(materialIndexCount));
            if (materialIndexCount != triangleCount)
            {
                AZ_Error("JoltPhysics", false,
                    "JoltMeshUtils: mesh blob material table size mismatch (%u entries for %u faces)",
                    materialIndexCount, triangleCount);
                return nullptr;
            }
            materialCursor = countCursor + sizeof(materialIndexCount);
        }

        JPH::VertexList joltVertices;
        joltVertices.reserve(header.m_vertexCount);
        for (AZ::u32 i = 0; i < header.m_vertexCount; ++i)
        {
            float xyz[3];
            memcpy(xyz, vertexCursor + static_cast<size_t>(i) * 3 * sizeof(float), sizeof(xyz));
            joltVertices.push_back(JPH::Float3(xyz[0], xyz[1], xyz[2]));
        }

        JPH::IndexedTriangleList joltTriangles;
        joltTriangles.reserve(triangleCount);
        AZStd::vector<AZ::u32> rawIndices(header.m_indexCount);
        memcpy(rawIndices.data(), indexCursor, indexBytes);
        bool clampedMaterialIndex = false;
        for (AZ::u32 t = 0; t < triangleCount; ++t)
        {
            AZ::u32 materialIndex = 0;
            if (materialCursor)
            {
                materialIndex = materialCursor[t];
                if (materialIndex > cMaxMeshMaterialIndex)
                {
                    // Jolt packs 5 bits per triangle; anything above falls to the last
                    // representable slot rather than corrupting a neighbor.
                    materialIndex = cMaxMeshMaterialIndex;
                    clampedMaterialIndex = true;
                }
            }
            joltTriangles.push_back(JPH::IndexedTriangle(
                rawIndices[t * 3 + 0], rawIndices[t * 3 + 1], rawIndices[t * 3 + 2], materialIndex));
        }
        AZ_Warning("JoltPhysics", !clampedMaterialIndex,
            "JoltMeshUtils: mesh blob has material indices above %u; they were clamped.", cMaxMeshMaterialIndex);

        JPH::MeshShapeSettings settings(joltVertices, joltTriangles);
        if (materialCursor)
        {
            // Jolt rejects materialized triangles with an empty material list. The
            // entries are placeholders: friction and restitution are resolved live at
            // contact time from the collider's slot list (see
            // JoltScene::GetMaterialForSubShape), never from this list - only the
            // per-triangle index has to survive.
            AZ::u32 maxMaterialIndex = 0;
            for (AZ::u32 t = 0; t < triangleCount; ++t)
            {
                maxMaterialIndex = AZStd::max<AZ::u32>(maxMaterialIndex, materialCursor[t]);
            }
            maxMaterialIndex = AZStd::min(maxMaterialIndex, static_cast<AZ::u32>(cMaxMeshMaterialIndex));
            settings.mMaterials.resize(maxMaterialIndex + 1, JPH::PhysicsMaterial::sDefault);
        }
        JPH::ShapeSettings::ShapeResult result = settings.Create();
        if (result.HasError())
        {
            AZ_Error("JoltPhysics", false, "Failed to create mesh shape: %s", result.GetError().c_str());
            return nullptr;
        }
        return result.Get();
    }

    AZStd::vector<AZ::u8> JoltMeshUtils::PackConvexMesh(const AZ::Vector3* vertices, AZ::u32 vertexCount)
    {
        AZStd::vector<AZ::u8> result;
        if (!vertices || vertexCount == 0)
        {
            return result;
        }

        ConvexBlobHeader header;
        header.m_vertexCount = vertexCount;

        const size_t vertexBytes = static_cast<size_t>(vertexCount) * 3 * sizeof(float);
        result.reserve(sizeof(ConvexBlobHeader) + vertexBytes);

        const auto* headerBytes = reinterpret_cast<const AZ::u8*>(&header);
        result.insert(result.end(), headerBytes, headerBytes + sizeof(ConvexBlobHeader));

        for (AZ::u32 i = 0; i < vertexCount; ++i)
        {
            const float xyz[3] = { vertices[i].GetX(), vertices[i].GetY(), vertices[i].GetZ() };
            const auto* bytes = reinterpret_cast<const AZ::u8*>(xyz);
            result.insert(result.end(), bytes, bytes + sizeof(xyz));
        }

        return result;
    }

    AZStd::vector<AZ::u8> JoltMeshUtils::PackConvexHulls(const AZStd::vector<AZStd::vector<AZ::Vector3>>& hulls)
    {
        AZStd::vector<AZ::u8> result;

        AZ::u32 hullCount = 0;
        AZ::u32 totalVertexCount = 0;
        for (const AZStd::vector<AZ::Vector3>& hull : hulls)
        {
            if (!hull.empty())
            {
                ++hullCount;
                totalVertexCount += static_cast<AZ::u32>(hull.size());
            }
        }
        if (hullCount == 0)
        {
            return result;
        }

        ConvexBlobHeader header;
        header.m_version = cConvexBlobVersionHullGroup;
        header.m_vertexCount = totalVertexCount;

        const size_t vertexBytes = static_cast<size_t>(totalVertexCount) * 3 * sizeof(float);
        result.reserve(sizeof(ConvexBlobHeader) + sizeof(AZ::u32) + hullCount * sizeof(AZ::u32) + vertexBytes);

        const auto* headerBytes = reinterpret_cast<const AZ::u8*>(&header);
        result.insert(result.end(), headerBytes, headerBytes + sizeof(ConvexBlobHeader));

        const auto* hullCountBytes = reinterpret_cast<const AZ::u8*>(&hullCount);
        result.insert(result.end(), hullCountBytes, hullCountBytes + sizeof(hullCount));

        for (const AZStd::vector<AZ::Vector3>& hull : hulls)
        {
            if (hull.empty())
            {
                continue;
            }
            const AZ::u32 vertexCount = static_cast<AZ::u32>(hull.size());
            const auto* countBytes = reinterpret_cast<const AZ::u8*>(&vertexCount);
            result.insert(result.end(), countBytes, countBytes + sizeof(vertexCount));
            for (const AZ::Vector3& vertex : hull)
            {
                const float xyz[3] = { vertex.GetX(), vertex.GetY(), vertex.GetZ() };
                const auto* bytes = reinterpret_cast<const AZ::u8*>(xyz);
                result.insert(result.end(), bytes, bytes + sizeof(xyz));
            }
        }

        return result;
    }

    JPH::RefConst<JPH::Shape> JoltMeshUtils::CreateConvexShapeFromCookedData(const AZStd::vector<AZ::u8>& cookedData)
    {
        if (cookedData.size() < sizeof(ConvexBlobHeader))
        {
            return nullptr;
        }

        ConvexBlobHeader header;
        memcpy(&header, cookedData.data(), sizeof(ConvexBlobHeader));

        if (header.m_magic != ConvexBlobHeader().m_magic ||
            (header.m_version != cConvexBlobVersionSingle && header.m_version != cConvexBlobVersionHullGroup) ||
            header.m_vertexCount == 0)
        {
            AZ_Error("JoltPhysics", false,
                "JoltMeshUtils: cooked convex blob is malformed or from an incompatible version "
                "(magic=0x%08X version=%u vertexCount=%u blobSize=%zu)",
                header.m_magic, header.m_version, header.m_vertexCount, cookedData.size());
            return nullptr;
        }

        // Gather the hull point clouds: v1 is one flat cloud, v2 is a counted list.
        AZStd::vector<JPH::Array<JPH::Vec3>> hullPointClouds;
        AZ::u32 totalVertexCount = 0;
        if (header.m_version == cConvexBlobVersionSingle)
        {
            const size_t vertexBytes = static_cast<size_t>(header.m_vertexCount) * 3 * sizeof(float);
            const size_t expectedSize = sizeof(ConvexBlobHeader) + vertexBytes;
            if (cookedData.size() != expectedSize)
            {
                AZ_Error("JoltPhysics", false, "JoltMeshUtils: cooked convex blob size mismatch (expected %zu, got %zu)",
                    expectedSize, cookedData.size());
                return nullptr;
            }

            const AZ::u8* vertexCursor = cookedData.data() + sizeof(ConvexBlobHeader);
            JPH::Array<JPH::Vec3> points;
            points.reserve(header.m_vertexCount);
            for (AZ::u32 i = 0; i < header.m_vertexCount; ++i)
            {
                float xyz[3];
                memcpy(xyz, vertexCursor + static_cast<size_t>(i) * 3 * sizeof(float), sizeof(xyz));
                points.push_back(JPH::Vec3(xyz[0], xyz[1], xyz[2]));
            }
            totalVertexCount = header.m_vertexCount;
            hullPointClouds.push_back(AZStd::move(points));
        }
        else
        {
            const AZ::u8* cursor = cookedData.data() + sizeof(ConvexBlobHeader);
            const AZ::u8* end = cookedData.data() + cookedData.size();
            if (static_cast<size_t>(end - cursor) < sizeof(AZ::u32))
            {
                AZ_Error("JoltPhysics", false, "JoltMeshUtils: hull-group blob is missing its hull count");
                return nullptr;
            }
            AZ::u32 hullCount;
            memcpy(&hullCount, cursor, sizeof(hullCount));
            cursor += sizeof(hullCount);

            // Bound the count against what is actually left before reserving for it. Every
            // hull costs at least its own vertex count plus one three-float vertex, so a
            // count larger than the remaining bytes allow cannot be real - and reserving
            // for a corrupted one (up to four billion vector objects) aborts inside the
            // allocator instead of reaching the clean error path a few lines down.
            constexpr size_t MinimumBytesPerHull = sizeof(AZ::u32) + 3 * sizeof(float);
            const size_t remainingBytes = static_cast<size_t>(end - cursor);
            if (hullCount > remainingBytes / MinimumBytesPerHull)
            {
                AZ_Error("JoltPhysics", false,
                    "JoltMeshUtils: hull-group blob claims %u hulls, which cannot fit in its remaining %zu bytes "
                    "(corrupt or truncated asset)",
                    hullCount, remainingBytes);
                return nullptr;
            }

            hullPointClouds.reserve(hullCount);
            for (AZ::u32 hull = 0; hull < hullCount; ++hull)
            {
                JPH::Array<JPH::Vec3> points;
                if (!ReadHullPoints(cursor, end, points))
                {
                    AZ_Error("JoltPhysics", false,
                        "JoltMeshUtils: hull-group blob is truncated at hull %u of %u", hull, hullCount);
                    return nullptr;
                }
                totalVertexCount += static_cast<AZ::u32>(points.size());
                hullPointClouds.push_back(AZStd::move(points));
            }
            if (cursor != end || hullCount == 0 || totalVertexCount != header.m_vertexCount)
            {
                AZ_Error("JoltPhysics", false,
                    "JoltMeshUtils: hull-group blob is inconsistent (hullCount=%u declaredVertices=%u readVertices=%u trailingBytes=%zd)",
                    hullCount, header.m_vertexCount, totalVertexCount, end - cursor);
                return nullptr;
            }
        }

        // A single hull stays a bare hull shape; only a real group pays for a compound.
        if (hullPointClouds.size() == 1)
        {
            return CreateHullShape(hullPointClouds.front());
        }

        JPH::StaticCompoundShapeSettings compoundSettings;
        for (const JPH::Array<JPH::Vec3>& points : hullPointClouds)
        {
            JPH::RefConst<JPH::Shape> hullShape = CreateHullShape(points);
            if (!hullShape)
            {
                return nullptr;
            }
            // Points are entity-local already, so every hull sits at the compound origin.
            compoundSettings.AddShape(JPH::Vec3::sZero(), JPH::Quat::sIdentity(), hullShape);
        }

        JPH::ShapeSettings::ShapeResult compoundResult = compoundSettings.Create();
        if (compoundResult.HasError())
        {
            AZ_Error("JoltPhysics", false, "Failed to create hull-group compound shape: %s",
                compoundResult.GetError().c_str());
            return nullptr;
        }
        return compoundResult.Get();
    }

    bool JoltMeshUtils::GatherVisibleGeometrySoup(
        const AzFramework::VisibleGeometryContainer& geometryContainer,
        const AZ::Transform& entityWorldTransform,
        AZStd::vector<AZ::Vector3>& outVertices,
        AZStd::vector<AZ::u32>& outIndices)
    {
        const AZ::Transform worldToEntity = entityWorldTransform.GetInverse();
        for (const AzFramework::VisibleGeometry& geometry : geometryContainer)
        {
            GatherEntryPoints(geometry, worldToEntity, outVertices, &outIndices);
        }
        return !outVertices.empty() && !outIndices.empty();
    }

    bool JoltMeshUtils::CookVisibleGeometry(
        const AzFramework::VisibleGeometryContainer& geometryContainer,
        const AZ::Transform& entityWorldTransform,
        Physics::CookedMeshShapeConfiguration::MeshType meshType,
        Physics::CookedMeshShapeConfiguration& outConfiguration,
        ConvexGrouping convexGrouping)
    {
        const AZ::Transform worldToEntity = entityWorldTransform.GetInverse();

        if (meshType == Physics::CookedMeshShapeConfiguration::MeshType::Convex &&
            convexGrouping == ConvexGrouping::PerGeometryEntry)
        {
            // Hull group: one hull per render node (wheels, body, ...), so the collision
            // can follow an asset's articulation instead of wrapping it in one blob.
            AZStd::vector<AZStd::vector<AZ::Vector3>> hulls;
            AZStd::vector<AZ::Vector3> leftovers;
            for (const AzFramework::VisibleGeometry& geometry : geometryContainer)
            {
                AZStd::vector<AZ::Vector3> entryPoints;
                if (!GatherEntryPoints(geometry, worldToEntity, entryPoints, nullptr))
                {
                    continue;
                }
                if (CanBuildHull(entryPoints))
                {
                    hulls.push_back(AZStd::move(entryPoints));
                }
                else
                {
                    AZ_Warning("JoltPhysics", false,
                        "JoltMeshUtils::CookVisibleGeometry: a geometry entry (%zu vertices) cannot form a convex "
                        "hull on its own; merging it into a shared hull.", entryPoints.size());
                    leftovers.insert(leftovers.end(), entryPoints.begin(), entryPoints.end());
                }
            }

            if (!leftovers.empty())
            {
                if (CanBuildHull(leftovers))
                {
                    hulls.push_back(AZStd::move(leftovers));
                }
                else
                {
                    // Last resort: one hull around everything still collides, which beats
                    // dropping geometry on the floor.
                    AZ_Warning("JoltPhysics", false,
                        "JoltMeshUtils::CookVisibleGeometry: the leftover geometry cannot form a hull either; "
                        "falling back to a single merged hull.");
                    AZStd::vector<AZ::Vector3> merged = AZStd::move(leftovers);
                    for (const AZStd::vector<AZ::Vector3>& hull : hulls)
                    {
                        merged.insert(merged.end(), hull.begin(), hull.end());
                    }
                    hulls.clear();
                    if (CanBuildHull(merged))
                    {
                        hulls.push_back(AZStd::move(merged));
                    }
                }
            }

            const AZStd::vector<AZ::u8> cookedData = PackConvexHulls(hulls);
            if (cookedData.empty())
            {
                return false;
            }

            outConfiguration = Physics::CookedMeshShapeConfiguration();
            outConfiguration.SetCookedMeshData(cookedData.data(), cookedData.size(), meshType);
            return true;
        }

        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        if (!GatherVisibleGeometrySoup(geometryContainer, entityWorldTransform, vertices, indices))
        {
            return false;
        }

        const AZStd::vector<AZ::u8> cookedData = (meshType == Physics::CookedMeshShapeConfiguration::MeshType::Convex)
            ? PackConvexMesh(vertices.data(), static_cast<AZ::u32>(vertices.size()))
            : PackTriangleMesh(
                  vertices.data(), static_cast<AZ::u32>(vertices.size()),
                  indices.data(), static_cast<AZ::u32>(indices.size()));
        if (cookedData.empty())
        {
            return false;
        }

        outConfiguration = Physics::CookedMeshShapeConfiguration();
        outConfiguration.SetCookedMeshData(cookedData.data(), cookedData.size(), meshType);
        return true;
    }

} // namespace JoltPhysics
