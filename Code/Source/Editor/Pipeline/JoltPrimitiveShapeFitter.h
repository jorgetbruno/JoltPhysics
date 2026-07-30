#pragma once

#include <AzCore/Math/Transform.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/optional.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>

#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics::Pipeline
{
    //! Which primitive to fit to a mesh node at export time (BestFit tries all three
    //! and keeps the smallest bounding volume).
    enum class PrimitiveFitTarget : AZ::u8
    {
        BestFit,
        Sphere,
        Box,
        Capsule,
    };

    struct PrimitiveFitResult
    {
        AZStd::shared_ptr<Physics::ShapeConfiguration> m_shapeConfig;
        AZ::Transform m_transform = AZ::Transform::CreateIdentity(); //!< Where the fitted shape sits in the node's frame.
    };

    //! Fits a primitive collider to a point cloud via principal component analysis.
    //! Deterministic and cheap; deliberately less tight than PhysX's volume-minimizing
    //! optimizer - precision geometry belongs to the convex/decompose export modes.
    //! Returns nullopt for degenerate input (too few points, or zero extent).
    AZStd::optional<PrimitiveFitResult> FitPrimitiveToPoints(
        const AZStd::vector<AZ::Vector3>& points, PrimitiveFitTarget target);

} // namespace JoltPhysics::Pipeline
