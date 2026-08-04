#include <System/CollisionLayerFilters.h>
#include <System/JoltSystem.h>

#include <AzCore/std/limits.h>
#include <AzCore/std/parallel/lock.h>

namespace JoltPhysics
{
    void JoltObjectLayerRegistry::Initialize()
    {
        AZStd::lock_guard lock(m_registrationMutex);

        // Layer 0 and 1 are the defaults referenced by ObjectLayers::NonMoving/Moving:
        // no collider configuration, so they collide with everything.
        m_entries[ObjectLayers::NonMoving] = { AZStd::numeric_limits<AZ::u64>::max(), 0, false };
        m_entries[ObjectLayers::Moving] = { AZStd::numeric_limits<AZ::u64>::max(), 0, true };
        m_count.store(2, AZStd::memory_order_release);
    }

    JPH::ObjectLayer JoltObjectLayerRegistry::Acquire(
        AZ::u64 collidesWithMask, AZ::u8 collisionLayerIndex, bool isMoving, JoltBodyClass bodyClass)
    {
        AZStd::lock_guard lock(m_registrationMutex);

        const JPH::ObjectLayer count = m_count.load(AZStd::memory_order_acquire);
        for (JPH::ObjectLayer i = 0; i < count; ++i)
        {
            const Entry& entry = m_entries[i];
            if (entry.m_collidesWithMask == collidesWithMask && entry.m_collisionLayerIndex == collisionLayerIndex &&
                entry.m_isMoving == isMoving && entry.m_bodyClass == bodyClass)
            {
                return i;
            }
        }

        if (count >= MaxObjectLayers)
        {
            AZ_WarningOnce("JoltPhysics", false,
                "More than %d distinct combinations of collision layer and collision group are in use; further "
                "bodies collide with everything. Reduce the number of distinct collider filtering setups.",
                static_cast<int>(MaxObjectLayers));
            return isMoving ? ObjectLayers::Moving : ObjectLayers::NonMoving;
        }

        // Fill the entry before publishing it, so a reader that sees the new count also
        // sees a complete entry.
        m_entries[count] = { collidesWithMask, collisionLayerIndex, isMoving, bodyClass };
        m_count.store(static_cast<JPH::ObjectLayer>(count + 1), AZStd::memory_order_release);
        return count;
    }

    void BroadPhaseLayerInterfaceImpl::Initialize(const JoltObjectLayerRegistry* registry)
    {
        m_registry = registry;
    }

    JPH::uint BroadPhaseLayerInterfaceImpl::GetNumBroadPhaseLayers() const
    {
        return BroadPhaseLayers::NumLayers;
    }

    JPH::BroadPhaseLayer BroadPhaseLayerInterfaceImpl::GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const
    {
        // The broadphase only separates bodies that move from bodies that do not; the
        // collision layer and group are resolved later by the object layer filters.
        return (m_registry && m_registry->Get(inLayer).m_isMoving) ? BroadPhaseLayers::Moving : BroadPhaseLayers::NonMoving;
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

    void ObjectVsBroadPhaseLayerFilterImpl::Initialize(const JoltObjectLayerRegistry* registry)
    {
        m_registry = registry;
    }

    bool ObjectVsBroadPhaseLayerFilterImpl::ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const
    {
        if (!m_registry)
        {
            return true;
        }

        // A body that does not move only has to be tested against the tree of bodies that
        // do; two non-moving bodies never generate contacts.
        return m_registry->Get(inLayer1).m_isMoving || inLayer2 == BroadPhaseLayers::Moving;
    }

    void ObjectLayerPairFilterImpl::Initialize(const JoltObjectLayerRegistry* registry)
    {
        m_registry = registry;
    }

    bool ObjectLayerPairFilterImpl::ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const
    {
        if (!m_registry)
        {
            return true;
        }

        const JoltObjectLayerRegistry::Entry& entry1 = m_registry->Get(inObject1);
        const JoltObjectLayerRegistry::Entry& entry2 = m_registry->Get(inObject2);

        if (!entry1.m_isMoving && !entry2.m_isMoving)
        {
            return false;
        }

        // Characters and cloth do not meet. See JoltBodyClass for why; the short version is
        // that a character crushes cloth into faces Jolt cannot collide against, and the
        // engine's own cloth is not in the physics scene at all.
        const bool characterVsSoftBody =
            (entry1.m_bodyClass == JoltBodyClass::Character && entry2.m_bodyClass == JoltBodyClass::SoftBody) ||
            (entry1.m_bodyClass == JoltBodyClass::SoftBody && entry2.m_bodyClass == JoltBodyClass::Character);
        if (characterVsSoftBody)
        {
            return false;
        }

        // AzPhysics semantics: two bodies collide only if each one's collision group mask
        // contains the other one's collision layer.
        const AZ::u64 layerBit1 = AZ::u64(1) << entry1.m_collisionLayerIndex;
        const AZ::u64 layerBit2 = AZ::u64(1) << entry2.m_collisionLayerIndex;
        return (entry1.m_collidesWithMask & layerBit2) != 0 && (entry2.m_collidesWithMask & layerBit1) != 0;
    }

    JPH::ObjectLayer AcquireObjectLayer(
        const AzPhysics::CollisionLayer& collisionLayer,
        const AzPhysics::CollisionGroups::Id& collisionGroupId,
        bool isMoving,
        JoltBodyClass bodyClass)
    {
        auto* joltSystem = GetJoltSystem();
        if (!joltSystem)
        {
            return isMoving ? ObjectLayers::Moving : ObjectLayers::NonMoving;
        }

        const AzPhysics::CollisionGroup group =
            joltSystem->GetJoltConfiguration().m_collisionConfig.m_collisionGroups.FindGroupById(collisionGroupId);

        return joltSystem->GetObjectLayerRegistry().Acquire(
            group.GetMask(), static_cast<AZ::u8>(collisionLayer.GetIndex()), isMoving, bodyClass);
    }

    JPH::ObjectLayer AcquireObjectLayer(
        const Physics::ColliderConfiguration* colliderConfiguration, bool isMoving, JoltBodyClass bodyClass)
    {
        if (!colliderConfiguration)
        {
            return isMoving ? ObjectLayers::Moving : ObjectLayers::NonMoving;
        }

        return AcquireObjectLayer(
            colliderConfiguration->m_collisionLayer, colliderConfiguration->m_collisionGroupId, isMoving, bodyClass);
    }

    bool ObjectLayerMatchesQueryMask(JPH::ObjectLayer objectLayer, AZ::u64 collisionGroupMask)
    {
        auto* joltSystem = GetJoltSystem();
        if (!joltSystem)
        {
            return true;
        }

        const JoltObjectLayerRegistry::Entry& entry = joltSystem->GetObjectLayerRegistry().Get(objectLayer);
        return (collisionGroupMask & (AZ::u64(1) << entry.m_collisionLayerIndex)) != 0;
    }

} // namespace JoltPhysics
