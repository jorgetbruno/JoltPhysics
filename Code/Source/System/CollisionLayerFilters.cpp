#include <System/CollisionLayerFilters.h>

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

} // namespace JoltPhysics
