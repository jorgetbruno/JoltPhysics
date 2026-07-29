#include <Editor/EditorJoltConvexDecomposition.h>

#include <AzCore/Math/Aabb.h>

#include <VHACD.h>

namespace JoltPhysics::EditorConvexDecomposition
{
    namespace
    {
        // Forwards VHACD's progress callbacks to the caller's handler.
        class ProgressForwarder : public VHACD::IVHACD::IUserCallback
        {
        public:
            explicit ProgressForwarder(const AZStd::function<void(float)>& callback)
                : m_callback(callback)
            {
            }

            void Update(const double overallProgress,
                [[maybe_unused]] const double stageProgress,
                [[maybe_unused]] const double operationProgress,
                [[maybe_unused]] const char* const stage,
                [[maybe_unused]] const char* const operation) override
            {
                if (m_callback)
                {
                    m_callback(static_cast<float>(overallProgress));
                }
            }

        private:
            AZStd::function<void(float)> m_callback;
        };
    } // namespace

    DecompositionResult DecomposeToHullPointClouds(
        const AZStd::vector<AZ::Vector3>& vertices,
        const AZStd::vector<AZ::u32>& indices,
        const DecompositionParams& params,
        const AZStd::function<void(float)>& progressCallback)
    {
        DecompositionResult result;
        if (vertices.size() < 4 || indices.size() < 12 || indices.size() % 3 != 0)
        {
            return result;
        }

        // VHACD spins forever on flat (zero-volume) input, which could never yield a
        // hull anyway - reject it before calling in.
        AZ::Aabb bounds = AZ::Aabb::CreateNull();
        for (const AZ::Vector3& vertex : vertices)
        {
            bounds.AddPoint(vertex);
        }
        const AZ::Vector3 extent = bounds.GetMax() - bounds.GetMin();
        if (extent.GetX() <= 1e-6f || extent.GetY() <= 1e-6f || extent.GetZ() <= 1e-6f)
        {
            return result;
        }

        // VHACD consumes one flat float array of XYZ triples.
        AZStd::vector<float> points;
        points.reserve(vertices.size() * 3);
        for (const AZ::Vector3& vertex : vertices)
        {
            points.push_back(vertex.GetX());
            points.push_back(vertex.GetY());
            points.push_back(vertex.GetZ());
        }

        VHACD::IVHACD* vhacd = VHACD::CreateVHACD();
        VHACD::IVHACD::Parameters vhacdParams;
        vhacdParams.m_maxConvexHulls = params.m_maxHulls;
        vhacdParams.m_resolution = params.m_voxelResolution;
        vhacdParams.m_maxNumVerticesPerCH = params.m_maxVerticesPerHull;
        vhacdParams.m_concavity = params.m_concavity;
        // CPU only: deterministic output and no GPU/OpenCL dependence in the editor.
        vhacdParams.m_oclAcceleration = false;

        ProgressForwarder forwarder(progressCallback);
        if (progressCallback)
        {
            vhacdParams.m_callback = &forwarder;
        }

        const bool computed = vhacd->Compute(
            points.data(), static_cast<uint32_t>(vertices.size()),
            indices.data(), static_cast<uint32_t>(indices.size() / 3),
            vhacdParams);
        if (computed)
        {
            const uint32_t hullCount = vhacd->GetNConvexHulls();
            result.m_hulls.reserve(hullCount);
            for (uint32_t hullIndex = 0; hullIndex < hullCount; ++hullIndex)
            {
                VHACD::IVHACD::ConvexHull hull;
                vhacd->GetConvexHull(hullIndex, hull);

                AZStd::vector<AZ::Vector3>& cloud = result.m_hulls.emplace_back();
                cloud.reserve(hull.m_nPoints);
                for (uint32_t i = 0; i < hull.m_nPoints; ++i)
                {
                    cloud.emplace_back(
                        static_cast<float>(hull.m_points[i * 3 + 0]),
                        static_cast<float>(hull.m_points[i * 3 + 1]),
                        static_cast<float>(hull.m_points[i * 3 + 2]));
                }
            }
        }

        vhacd->Clean();
        vhacd->Release();
        return result;
    }

} // namespace JoltPhysics::EditorConvexDecomposition
