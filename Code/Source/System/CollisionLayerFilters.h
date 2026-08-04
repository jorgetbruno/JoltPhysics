#pragma once

#include <AzFramework/Physics/Configuration/CollisionConfiguration.h>
#include <AzFramework/Physics/Shape.h>

#include <AzCore/std/limits.h>
#include <AzCore/std/parallel/atomic.h>
#include <AzCore/std/parallel/mutex.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

namespace JoltPhysics
{
    namespace BroadPhaseLayers
    {
        static constexpr JPH::BroadPhaseLayer NonMoving(0);
        static constexpr JPH::BroadPhaseLayer Moving(1);
        static constexpr JPH::uint NumLayers = 2;
    }

    //! What kind of body holds an object layer. Only used to keep characters out of soft
    //! bodies, but it belongs here rather than at any one call site, because that is the
    //! single place every collision path already consults - the character's own sweep, the
    //! ground query Jolt's rigid character runs and gives no filter hook for, the inner
    //! body's simulation contacts, and the broadphase.
    //!
    //! Why characters and cloth do not mix: a character controller does not negotiate with
    //! what it hits, it pushes itself out, and cloth has nothing to push back with. It gets
    //! crushed until a triangle has no area, and Jolt cannot collide against a face with no
    //! normal to separate along. The engine's own cloth settles it the same way - the
    //! NvCloth gem has no PhysX dependency at all, so its cloth is never in the physics
    //! scene and a character cannot reach it.
    //!
    //! Rigid bodies still collide with soft bodies both ways; only characters are excluded.
    enum class JoltBodyClass : AZ::u8
    {
        Rigid,
        SoftBody,
        Character,
    };

    //! Assigns a Jolt object layer to each distinct combination of AzPhysics collision
    //! layer, collision group mask, motion class (moving or not) and body class.
    //!
    //! AzPhysics filtering is per body - two bodies collide only if each one's group mask
    //! contains the other one's layer - which cannot be expressed as a function of two
    //! layer indices. Giving every combination its own object layer makes it expressible,
    //! so the filtering happens in Jolt's object layer filters (consulted early, and by
    //! scene queries) instead of in a collision group filter.
    //!
    //! That also leaves JPH::CollisionGroup free for what Jolt uses it for: ragdolls
    //! install a GroupFilterTable there to disable parent/child collisions.
    class JoltObjectLayerRegistry
    {
    public:
        //! Combinations are per distinct (layer, group, motion class) actually used, so a
        //! project needs far fewer than this; exceeding it falls back to "collides with
        //! everything" rather than failing to create the body.
        static constexpr JPH::ObjectLayer MaxObjectLayers = 512;

        //! Registers the two default layers; must be called before any body is created.
        void Initialize();

        //! The object layer for this combination, registering one if it is new.
        //! Safe to call from the main thread while jobs read; readers never block.
        JPH::ObjectLayer Acquire(
            AZ::u64 collidesWithMask,
            AZ::u8 collisionLayerIndex,
            bool isMoving,
            JoltBodyClass bodyClass = JoltBodyClass::Rigid);

        struct Entry
        {
            AZ::u64 m_collidesWithMask = AZStd::numeric_limits<AZ::u64>::max();
            AZ::u8 m_collisionLayerIndex = 0;
            bool m_isMoving = true;
            JoltBodyClass m_bodyClass = JoltBodyClass::Rigid;
        };

        const Entry& Get(JPH::ObjectLayer objectLayer) const
        {
            // Only layers handed out by Acquire ever reach the filters.
            return m_entries[objectLayer < m_count.load(AZStd::memory_order_acquire) ? objectLayer : 0];
        }

        JPH::ObjectLayer GetCount() const
        {
            return m_count.load(AZStd::memory_order_acquire);
        }

    private:
        //! Fixed storage: entries are never moved, so readers can index it without a lock
        //! while another thread registers a new combination.
        Entry m_entries[MaxObjectLayers];
        AZStd::atomic<JPH::ObjectLayer> m_count{ 0 };
        AZStd::mutex m_registrationMutex; //!< Guards registration only, never lookup.
    };

    namespace ObjectLayers
    {
        //! The two layers the registry always registers first, used for bodies that have
        //! no collider configuration to take a layer and group from. Both collide with
        //! everything.
        static constexpr JPH::ObjectLayer NonMoving = 0;
        static constexpr JPH::ObjectLayer Moving = 1;
    }

    class BroadPhaseLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
    {
    public:
        BroadPhaseLayerInterfaceImpl() = default;

        void Initialize(const JoltObjectLayerRegistry* registry);

        JPH::uint GetNumBroadPhaseLayers() const override;
        JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override;

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
        const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override;
#endif

    private:
        const JoltObjectLayerRegistry* m_registry = nullptr;
    };

    class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter
    {
    public:
        ObjectVsBroadPhaseLayerFilterImpl() = default;

        void Initialize(const JoltObjectLayerRegistry* registry);

        bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override;

    private:
        const JoltObjectLayerRegistry* m_registry = nullptr;
    };

    class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter
    {
    public:
        ObjectLayerPairFilterImpl() = default;

        void Initialize(const JoltObjectLayerRegistry* registry);

        bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override;

    private:
        const JoltObjectLayerRegistry* m_registry = nullptr;
    };

    //! The object layer for a body built from this collider configuration. Falls back to
    //! the default moving/non-moving layer when there is no configuration to read.
    JPH::ObjectLayer AcquireObjectLayer(
        const Physics::ColliderConfiguration* colliderConfiguration,
        bool isMoving,
        JoltBodyClass bodyClass = JoltBodyClass::Rigid);

    //! The object layer for a body whose layer and group are configured directly (the
    //! character controller carries them outside a collider configuration).
    JPH::ObjectLayer AcquireObjectLayer(
        const AzPhysics::CollisionLayer& collisionLayer,
        const AzPhysics::CollisionGroups::Id& collisionGroupId,
        bool isMoving,
        JoltBodyClass bodyClass = JoltBodyClass::Rigid);

    //! Whether a query with this collision group mask should see bodies on this layer.
    bool ObjectLayerMatchesQueryMask(JPH::ObjectLayer objectLayer, AZ::u64 collisionGroupMask);

} // namespace JoltPhysics
