#include <System/CollisionLayerFilters.h>
#include <System/JoltSystem.h>

#include <AzCore/std/limits.h>

namespace JoltPhysics
{
    void BroadPhaseLayerInterfaceImpl::Initialize()
    {
        m_objectToBroadPhase[ObjectLayers::NonMoving] = BroadPhaseLayers::NonMoving;
        m_objectToBroadPhase[ObjectLayers::Moving] = BroadPhaseLayers::Moving;
    }

    JPH::uint BroadPhaseLayerInterfaceImpl::GetNumBroadPhaseLayers() const
    {
        return BroadPhaseLayers::NumLayers;
    }

    JPH::BroadPhaseLayer BroadPhaseLayerInterfaceImpl::GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const
    {
        JPH_ASSERT(inLayer < ObjectLayers::NumLayers);
        return m_objectToBroadPhase[inLayer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* BroadPhaseLayerInterfaceImpl::GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const
    {
        switch (static_cast<JPH::BroadPhaseLayer::Type>(inLayer))
        {
        case static_cast<JPH::BroadPhaseLayer::Type>(BroadPhaseLayers::NonMoving):
            return "NonMoving";
        case static_cast<JPH::BroadPhaseLayer::Type>(BroadPhaseLayers::Moving):
            return "Moving";
        default:
            JPH_ASSERT(false);
            return "Invalid";
        }
    }
#endif

    void ObjectVsBroadPhaseLayerFilterImpl::Initialize()
    {
    }

    bool ObjectVsBroadPhaseLayerFilterImpl::ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const
    {
        switch (inLayer1)
        {
        case ObjectLayers::NonMoving:
            return inLayer2 == BroadPhaseLayers::Moving;
        case ObjectLayers::Moving:
            return true;
        default:
            JPH_ASSERT(false);
            return false;
        }
    }

    void ObjectLayerPairFilterImpl::Initialize()
    {
    }

    bool ObjectLayerPairFilterImpl::ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const
    {
        switch (inObject1)
        {
        case ObjectLayers::NonMoving:
            return inObject2 == ObjectLayers::Moving;
        case ObjectLayers::Moving:
            return true;
        default:
            JPH_ASSERT(false);
            return false;
        }
    }

    JPH::CollisionGroup CreateCollisionGroupFromConfig(const Physics::ColliderConfiguration& colliderConfiguration)
    {
        AZ::u32 layerMask = AZStd::numeric_limits<AZ::u32>::max();

        if (auto* joltSystem = GetJoltSystem())
        {
            const AzPhysics::CollisionGroup group =
                joltSystem->GetJoltConfiguration().m_collisionConfig.m_collisionGroups.FindGroupById(
                    colliderConfiguration.m_collisionGroupId);
            layerMask = static_cast<AZ::u32>(group.GetMask());
        }

        // The filter is stateless: the mask and layer travel in the group itself.
        return JPH::CollisionGroup(
            AzPhysicsGroupFilter::Get(),
            layerMask /* group id */,
            colliderConfiguration.m_collisionLayer.GetIndex() /* subgroup id */);
    }

} // namespace JoltPhysics
