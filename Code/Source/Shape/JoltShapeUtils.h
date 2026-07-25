#pragma once

#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>

#include <Shape/JoltCylinderShapeConfiguration.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace JPH
{
    class MutableCompoundShape;
}

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

        //! Warns when the colliders of a compound body disagree on the trigger flag.
        //! Jolt sensors are per-body, so the first collider's flag decides for the whole
        //! body and the rest are ignored; without this the mismatch is silent and the
        //! body simply behaves unlike the authored colliders. See DIVERGENCES.md.
        static void WarnOnMixedTriggerFlags(
            const AzPhysics::ShapeColliderPairList& colliderPairs, const AZStd::string& debugName);

        //! Returns the prebuilt Physics::Shape objects from the variant data (empty when
        //! the variant holds shape configurations instead), in compound sub-shape order.
        static AZStd::vector<AZStd::shared_ptr<Physics::Shape>> GetPrebuiltShapes(
            const AzPhysics::ShapeVariantData& colliderAndShapeData);

        //! Returns a MutableCompoundShape equivalent to the given shape, so sub-shapes can
        //! be attached/detached on a live body. A compound shape has its sub-shapes copied
        //! across in order (keeping sub-shape indices, and the per-collider material indices
        //! that follow them, valid); any other shape becomes sub-shape 0.
        static JPH::Ref<JPH::MutableCompoundShape> MakeMutableCompound(const JPH::Shape* shape);

        //! Maps a sub-shape id from a scene query or contact back to the collider index it
        //! came from, so a hit can be attributed to the collider that produced it.
        //!
        //! Pass the body's *base* shape - the compound built from the colliders, before any
        //! center-of-mass wrapper. That wrapper can be left on: a RotatedTranslatedShape
        //! forwards the sub-shape id to its inner shape untouched, so ids stay valid either
        //! way. Returns 0 for a non-compound shape, which is the only collider such a body
        //! has.
        static size_t GetColliderIndexFromSubShapeId(const JPH::Shape* baseShape, const JPH::SubShapeID& subShapeId);

        static JPH::RefConst<JPH::Shape> CreateBoxShape(
            const Physics::BoxShapeConfiguration& config);

        static JPH::RefConst<JPH::Shape> CreateSphereShape(
            const Physics::SphereShapeConfiguration& config);

        static JPH::RefConst<JPH::Shape> CreateCapsuleShape(
            const Physics::CapsuleShapeConfiguration& config);

        static JPH::RefConst<JPH::Shape> CreateCylinderShape(
            const JoltCylinderShapeConfiguration& config);
    };

} // namespace JoltPhysics
