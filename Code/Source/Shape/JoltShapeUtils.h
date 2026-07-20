#pragma once

#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace JoltPhysics
{
    class JoltShapeUtils
    {
    public:
        static AZStd::shared_ptr<Physics::Shape> CreateShape(
            const Physics::ColliderConfiguration& colliderConfiguration,
            const Physics::ShapeConfiguration& shapeConfiguration);

        static JPH::RefConst<JPH::Shape> CreateJoltShape(
            const AzPhysics::RigidBodyConfiguration& configuration);

        static JPH::RefConst<JPH::Shape> CreateJoltShapeFromStatic(
            const AzPhysics::StaticRigidBodyConfiguration& configuration);

        static JPH::RefConst<JPH::Shape> CreateJoltShapeFromConfig(
            const Physics::ShapeConfiguration& shapeConfiguration);

        //! Returns the collider configuration of the first collider/shape pair found in the
        //! variant data, or nullptr if there is none. Per-body settings (collision filtering,
        //! sensor flag) are currently taken from the first collider only.
        static const Physics::ColliderConfiguration* GetFirstColliderConfiguration(
            const AzPhysics::ShapeVariantData& colliderAndShapeData);

        //! Returns all collider/shape pairs from the variant data (empty when none).
        static AzPhysics::ShapeColliderPairList GetColliderPairList(
            const AzPhysics::ShapeVariantData& colliderAndShapeData);

        static JPH::RefConst<JPH::Shape> CreateBoxShape(
            const Physics::BoxShapeConfiguration& config);

        static JPH::RefConst<JPH::Shape> CreateSphereShape(
            const Physics::SphereShapeConfiguration& config);

        static JPH::RefConst<JPH::Shape> CreateCapsuleShape(
            const Physics::CapsuleShapeConfiguration& config);
    };

} // namespace JoltPhysics
