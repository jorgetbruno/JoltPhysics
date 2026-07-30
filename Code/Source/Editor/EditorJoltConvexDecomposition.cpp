#include <Editor/EditorJoltConvexDecomposition.h>

#include <AzCore/Math/Aabb.h>
#include <AzCore/std/parallel/thread.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>

#include <VHACD.h>

namespace JoltPhysics::EditorConvexDecomposition
{
    namespace
    {
        //! VHACD spins forever on flat (zero-volume) input, which could never yield a hull
        //! anyway, and wants a triangulated soup. Rejecting both here keeps the session
        //! constructor from starting a run that cannot end well.
        bool IsUsableSoup(const AZStd::vector<AZ::Vector3>& vertices, const AZStd::vector<AZ::u32>& indices)
        {
            if (vertices.size() < 4 || indices.size() < 12 || indices.size() % 3 != 0)
            {
                return false;
            }

            AZ::Aabb bounds = AZ::Aabb::CreateNull();
            for (const AZ::Vector3& vertex : vertices)
            {
                bounds.AddPoint(vertex);
            }
            const AZ::Vector3 extent = bounds.GetMax() - bounds.GetMin();
            return extent.GetX() > 1e-6f && extent.GetY() > 1e-6f && extent.GetZ() > 1e-6f;
        }

        //! VHACD consumes one flat float array of XYZ triples.
        AZStd::vector<float> FlattenPoints(const AZStd::vector<AZ::Vector3>& vertices)
        {
            AZStd::vector<float> points;
            points.reserve(vertices.size() * 3);
            for (const AZ::Vector3& vertex : vertices)
            {
                points.push_back(vertex.GetX());
                points.push_back(vertex.GetY());
                points.push_back(vertex.GetZ());
            }
            return points;
        }

        VHACD::IVHACD::Parameters ToVhacdParams(const DecompositionParams& params)
        {
            VHACD::IVHACD::Parameters vhacdParams;
            vhacdParams.m_maxConvexHulls = params.m_maxHulls;
            vhacdParams.m_resolution = params.m_voxelResolution;
            vhacdParams.m_maxNumVerticesPerCH = params.m_maxVerticesPerHull;
            vhacdParams.m_concavity = params.m_concavity;
            // CPU only: deterministic output and no GPU/OpenCL dependence in the editor.
            vhacdParams.m_oclAcceleration = false;
            return vhacdParams;
        }

        //! Reads the hulls VHACD produced into point clouds.
        DecompositionResult CollectHulls(VHACD::IVHACD& vhacd)
        {
            DecompositionResult result;
            const uint32_t hullCount = vhacd.GetNConvexHulls();
            result.m_hulls.reserve(hullCount);
            for (uint32_t hullIndex = 0; hullIndex < hullCount; ++hullIndex)
            {
                VHACD::IVHACD::ConvexHull hull;
                vhacd.GetConvexHull(hullIndex, hull);

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
            return result;
        }
    } // namespace

    //! Records VHACD's progress. VHACD dispatches its messages from whichever thread polls
    //! IsReady(), not from its worker, so this needs no synchronization of its own.
    class DecompositionSession::ProgressForwarder : public VHACD::IVHACD::IUserCallback
    {
    public:
        explicit ProgressForwarder(float& progress)
            : m_progress(progress)
        {
        }

        void Update(const double overallProgress,
            [[maybe_unused]] const double stageProgress,
            [[maybe_unused]] const double operationProgress,
            [[maybe_unused]] const char* const stage,
            [[maybe_unused]] const char* const operation) override
        {
            m_progress = static_cast<float>(overallProgress);
            ++m_updateCount;
        }

        //! Zero until VHACD has reported anything, which it only does once the run proper
        //! has begun. Cancel needs that distinction; see below.
        AZ::u32 GetUpdateCount() const
        {
            return m_updateCount;
        }

    private:
        float& m_progress;
        AZ::u32 m_updateCount = 0;
    };

    DecompositionSession::DecompositionSession(
        const AZStd::vector<AZ::Vector3>& vertices,
        const AZStd::vector<AZ::u32>& indices,
        const DecompositionParams& params)
    {
        if (!IsUsableSoup(vertices, indices))
        {
            return;
        }

        m_progressForwarder = AZStd::make_unique<ProgressForwarder>(m_progress);

        VHACD::IVHACD::Parameters vhacdParams = ToVhacdParams(params);
        vhacdParams.m_callback = m_progressForwarder.get();

        VHACD::IVHACD* vhacd = VHACD::CreateVHACD_ASYNC();
        const AZStd::vector<float> points = FlattenPoints(vertices);
        // The async Compute copies the soup into its own buffers before starting, so the
        // locals here can go out of scope; it returns as soon as the thread is running.
        vhacd->Compute(
            points.data(), static_cast<uint32_t>(vertices.size()),
            indices.data(), static_cast<uint32_t>(indices.size() / 3),
            vhacdParams);
        m_vhacd = vhacd;
    }

    DecompositionSession::~DecompositionSession()
    {
        Cancel();
        if (auto* vhacd = static_cast<VHACD::IVHACD*>(m_vhacd))
        {
            vhacd->Clean();
            vhacd->Release();
            m_vhacd = nullptr;
        }
    }

    bool DecompositionSession::IsFinished()
    {
        auto* vhacd = static_cast<VHACD::IVHACD*>(m_vhacd);
        // IsReady also dispatches the pending progress messages, which is what advances
        // m_progress - so this call is the progress pump as well as the completion check.
        return vhacd == nullptr || vhacd->IsReady();
    }

    DecompositionResult DecompositionSession::TakeResult()
    {
        auto* vhacd = static_cast<VHACD::IVHACD*>(m_vhacd);
        if (vhacd == nullptr || !vhacd->IsReady())
        {
            return {};
        }

        DecompositionResult result = CollectHulls(*vhacd);
        vhacd->Clean();
        vhacd->Release();
        m_vhacd = nullptr;
        return result;
    }

    void DecompositionSession::Cancel()
    {
        auto* vhacd = static_cast<VHACD::IVHACD*>(m_vhacd);
        if (vhacd == nullptr)
        {
            return;
        }

        // VHACD clears its own cancel flag as the run starts (VHACD::Init), so a cancel
        // raised before then is dropped - and since Cancel joins the thread, the caller
        // would sit through the entire run it meant to stop. Waiting for the first
        // progress message is what makes cancelling a just-started run prompt: VHACD only
        // reports once it is past Init, and that first report costs milliseconds.
        constexpr AZStd::chrono::milliseconds pollInterval{ 1 };
        while (m_progressForwarder->GetUpdateCount() == 0 && !vhacd->IsReady())
        {
            AZStd::this_thread::sleep_for(pollInterval);
        }

        // Signals the run and joins VHACD's thread; a no-op once it has finished.
        vhacd->Cancel();
    }

    DecompositionResult DecomposeToHullPointClouds(
        const AZStd::vector<AZ::Vector3>& vertices,
        const AZStd::vector<AZ::u32>& indices,
        const DecompositionParams& params,
        const AZStd::function<void(float)>& progressCallback)
    {
        DecompositionSession session(vertices, indices, params);
        if (!session.IsValid())
        {
            return {};
        }

        // Polling is what pumps VHACD's progress messages, so a caller that wants progress
        // gets it at this interval; one that does not just waits.
        constexpr AZStd::chrono::milliseconds pollInterval{ 10 };
        while (!session.IsFinished())
        {
            if (progressCallback)
            {
                progressCallback(session.GetProgress());
            }
            AZStd::this_thread::sleep_for(pollInterval);
        }

        if (progressCallback)
        {
            progressCallback(session.GetProgress());
        }
        return session.TakeResult();
    }

} // namespace JoltPhysics::EditorConvexDecomposition
