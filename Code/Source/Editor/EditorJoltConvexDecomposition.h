#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/functional.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>

namespace JoltPhysics::EditorConvexDecomposition
{
    //! Knobs for a decomposition run, mapped onto VHACD's parameters.
    struct DecompositionParams
    {
        AZ::u32 m_maxHulls = 16; //!< Maximum hulls to produce (VHACD m_maxConvexHulls).
        AZ::u32 m_voxelResolution = 100000; //!< Voxelization resolution (VHACD m_resolution).
        AZ::u32 m_maxVerticesPerHull = 64; //!< Per-hull vertex cap (VHACD m_maxNumVerticesPerCH; Jolt hulls cap at 256).
        double m_concavity = 0.001; //!< Maximum concavity error (VHACD m_concavity).
    };

    struct DecompositionResult
    {
        AZStd::vector<AZStd::vector<AZ::Vector3>> m_hulls; //!< One point cloud per convex hull.

        bool Succeeded() const
        {
            return !m_hulls.empty();
        }
    };

    //! A decomposition running on VHACD's own background thread (VHACD::CreateVHACD_ASYNC).
    //!
    //! Poll IsFinished() from wherever the caller already ticks. Polling is also what
    //! dispatches VHACD's progress messages - it marshals them to the polling thread - so
    //! GetProgress() only advances while something is asking, and no callback ever runs on
    //! the worker thread.
    //!
    //! Cancelling is real rather than abandonment: VHACD is signalled and the run stops at
    //! its next check, which the destructor waits for. That wait is why a session must not
    //! outlive an editor shutdown it is not part of - drop it in Deactivate.
    class DecompositionSession
    {
    public:
        //! Starts the run. The input is copied by VHACD, so the caller's buffers need not
        //! outlive this call. Rejected input (degenerate or non-triangulated) leaves the
        //! session invalid rather than starting anything.
        DecompositionSession(
            const AZStd::vector<AZ::Vector3>& vertices,
            const AZStd::vector<AZ::u32>& indices,
            const DecompositionParams& params);
        ~DecompositionSession();

        DecompositionSession(const DecompositionSession&) = delete;
        DecompositionSession& operator=(const DecompositionSession&) = delete;

        //! False when the input was rejected and no run was started.
        bool IsValid() const
        {
            return m_vhacd != nullptr;
        }

        //! True once the run is done. Dispatches pending progress messages, so call it
        //! before reading GetProgress(). An invalid session is finished immediately.
        bool IsFinished();

        //! Overall progress in [0, 100], as of the last IsFinished() call.
        float GetProgress() const
        {
            return m_progress;
        }

        //! The hulls VHACD produced. Empty unless the run finished; moves the result out,
        //! so it answers once.
        DecompositionResult TakeResult();

        //! Signals VHACD to stop and waits for its thread to exit. Safe to call twice, and
        //! on a finished or invalid session. Returns in milliseconds rather than at the
        //! end of the run - VHACD checks the signal between stages and inside its clipping
        //! loops - so this is a real stop, not an abandonment.
        void Cancel();

    private:
        class ProgressForwarder;

        void* m_vhacd = nullptr; //!< VHACD::IVHACD*, kept opaque so VHACD.h stays out of this header.
        AZStd::unique_ptr<ProgressForwarder> m_progressForwarder;
        float m_progress = 0.0f;
    };

    //! Runs VHACD (volumetric hierarchical approximate convex decomposition) over a
    //! triangle soup and returns one point cloud per convex hull. Editor-only: the
    //! clouds are baked into the same hull-group blob the runtime already decodes, so
    //! the runtime never links the decomposer. CPU-only, and deterministic for a given
    //! input and parameters.
    //!
    //! Blocks until the run finishes (on top of DecompositionSession, which does the work
    //! on VHACD's thread), so callers that must stay responsive - the editor bake - should
    //! drive a session themselves instead. progressCallback, when given, receives overall
    //! progress in [0, 100].
    DecompositionResult DecomposeToHullPointClouds(
        const AZStd::vector<AZ::Vector3>& vertices,
        const AZStd::vector<AZ::u32>& indices,
        const DecompositionParams& params,
        const AZStd::function<void(float)>& progressCallback = {});

} // namespace JoltPhysics::EditorConvexDecomposition
