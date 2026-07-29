#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/functional.h>

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

    //! Runs VHACD (volumetric hierarchical approximate convex decomposition) over a
    //! triangle soup and returns one point cloud per convex hull. Editor-only: the
    //! clouds are baked into the same hull-group blob the runtime already decodes, so
    //! the runtime never links the decomposer. Synchronous and CPU-only (deterministic);
    //! call it from a worker thread for large meshes. progressCallback, when given,
    //! receives overall progress in [0, 100].
    DecompositionResult DecomposeToHullPointClouds(
        const AZStd::vector<AZ::Vector3>& vertices,
        const AZStd::vector<AZ::u32>& indices,
        const DecompositionParams& params,
        const AZStd::function<void(float)>& progressCallback = {});

} // namespace JoltPhysics::EditorConvexDecomposition
