#pragma once

#include <AzFramework/Physics/Configuration/CollisionConfiguration.h>

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

    //! JPH::GroupFilter carrying an AzPhysics collision-group layer mask.
    //! The owning body's collision layer index travels in JPH::CollisionGroup's subgroup id.
    class AzPhysicsGroupFilter final : public JPH::GroupFilter
    {
    public:
        explicit AzPhysicsGroupFilter(AZ::u32 layerMask)
            : m_layerMask(layerMask)
        {
        }

        bool CanCollide(const JPH::CollisionGroup& inGroup1, const JPH::CollisionGroup& inGroup2) const override
        {
            const auto* filter1 = static_cast<const AzPhysicsGroupFilter*>(inGroup1.GetGroupFilter());
            const auto* filter2 = static_cast<const AzPhysicsGroupFilter*>(inGroup2.GetGroupFilter());
            const AZ::u32 layer1 = inGroup1.GetSubGroupID();
            const AZ::u32 layer2 = inGroup2.GetSubGroupID();
            // AzPhysics semantics: two bodies collide only if each one's group mask
            // contains the other one's layer.
            return (filter1->m_layerMask & (1u << layer2)) != 0 && (filter2->m_layerMask & (1u << layer1)) != 0;
        }

        AZ::u32 GetLayerMask() const
        {
            return m_layerMask;
        }

    private:
        AZ::u32 m_layerMask = 0;
    };

    //! Builds the JPH collision group for a body from an AzPhysics collider configuration.
    //! The collision layer index becomes the subgroup id and the collision group mask
    //! is carried by an AzPhysicsGroupFilter.
    JPH::CollisionGroup CreateCollisionGroupFromConfig(const Physics::ColliderConfiguration& colliderConfiguration);

} // namespace JoltPhysics
