#pragma once

#include <AzFramework/Physics/Configuration/CollisionConfiguration.h>
#include <AzFramework/Physics/Shape.h>

#include <AzCore/std/limits.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/CollisionGroup.h>
#include <Jolt/Physics/Collision/GroupFilter.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

namespace JoltPhysics
{
    namespace BroadPhaseLayers
    {
        static constexpr JPH::BroadPhaseLayer NonMoving(0);
        static constexpr JPH::BroadPhaseLayer Moving(1);
        static constexpr JPH::uint NumLayers = 2;
    }

    namespace ObjectLayers
    {
        static constexpr JPH::ObjectLayer NonMoving = 0;
        static constexpr JPH::ObjectLayer Moving = 1;
        static constexpr JPH::ObjectLayer NumLayers = 2;
    }

    class BroadPhaseLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
    {
    public:
        BroadPhaseLayerInterfaceImpl() = default;

        void Initialize();

        JPH::uint GetNumBroadPhaseLayers() const override;
        JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override;

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
        const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override;
#endif

    private:
        JPH::BroadPhaseLayer m_objectToBroadPhase[ObjectLayers::NumLayers];
    };

    class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter
    {
    public:
        ObjectVsBroadPhaseLayerFilterImpl() = default;

        void Initialize();

        bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override;
    };

    class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter
    {
    public:
        ObjectLayerPairFilterImpl() = default;

        void Initialize();

        bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override;
    };

    //! Group filter implementing AzPhysics layer/group filtering.
    //!
    //! Both values it needs travel in JPH::CollisionGroup itself - the body's collision
    //! group mask as the group id and its collision layer index as the subgroup id - so
    //! the filter is stateless and never has to inspect the other body's filter object.
    //! That matters because Jolt installs its own GroupFilterTable on bodies it builds
    //! itself (ragdolls use one to disable parent/child collisions), and reading such a
    //! filter as if it were this one is undefined behaviour.
    //!
    //! A body carrying a foreign filter has no group id of ours, which reads as
    //! cInvalidGroup (all bits set) and therefore collides with every layer. Its subgroup
    //! id means something else entirely to its own filter - a ragdoll part stores its
    //! joint index there - so a body whose collision group mask excludes some layers can
    //! still filter such a part out by coincidence. Ragdoll parts do not carry collision
    //! layers yet; see DIVERGENCES.md.
    class AzPhysicsGroupFilter final : public JPH::GroupFilter
    {
    public:
        AzPhysicsGroupFilter()
        {
            // The single instance lives in static storage; mark it embedded so Jolt's
            // reference counting never tries to delete it.
            SetEmbedded();
        }

        bool CanCollide(const JPH::CollisionGroup& inGroup1, const JPH::CollisionGroup& inGroup2) const override
        {
            const AZ::u32 layerMask1 = inGroup1.GetGroupID();
            const AZ::u32 layerMask2 = inGroup2.GetGroupID();
            // A foreign filter's subgroup id is not a layer index at all, so keep the
            // shift in range rather than letting it run off the end of the mask.
            const AZ::u32 layerBit1 = 1u << (inGroup1.GetSubGroupID() & 31u);
            const AZ::u32 layerBit2 = 1u << (inGroup2.GetSubGroupID() & 31u);
            // AzPhysics semantics: two bodies collide only if each one's group mask
            // contains the other one's layer.
            return (layerMask1 & layerBit2) != 0 && (layerMask2 & layerBit1) != 0;
        }

        //! The single shared instance; the filter holds no per-body state, so one serves
        //! every body and nothing is allocated per collider.
        static AzPhysicsGroupFilter* Get()
        {
            static AzPhysicsGroupFilter s_instance;
            return &s_instance;
        }
    };

    //! Builds the JPH collision group for a body from an AzPhysics collider configuration.
    //! The collision layer index becomes the subgroup id and the collision group mask
    //! is carried by an AzPhysicsGroupFilter.
    JPH::CollisionGroup CreateCollisionGroupFromConfig(const Physics::ColliderConfiguration& colliderConfiguration);

} // namespace JoltPhysics
