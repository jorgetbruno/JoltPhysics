#pragma once

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

#include <AzFramework/Physics/Configuration/SystemConfiguration.h>
#include <AzFramework/Physics/Configuration/CollisionConfiguration.h>

namespace JoltPhysics
{
    class JoltSystemConfiguration : public AzPhysics::SystemConfiguration
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltSystemConfiguration, AZ::SystemAllocator);
        AZ_RTTI(JoltSystemConfiguration, "{7C8E3D5F-2A4B-4E9C-8D1F-6A3B5C7E9D2F}", AzPhysics::SystemConfiguration);
        static void Reflect(AZ::ReflectContext* context);

        //! Seeds the collision configuration, which AzFramework leaves entirely empty:
        //! CollisionLayers is 64 blank names and CollisionGroups an empty preset list.
        //! Without this the layer and group dropdowns have nothing to offer and every
        //! collider sits on an unnamed layer. PhysX seeds the same "Default" layer and
        //! "All"/"None" groups from its own configuration.
        JoltSystemConfiguration();
        ~JoltSystemConfiguration() override = default;

        //! Preset ids for the two groups every project starts with. Fixed rather than
        //! generated, because a collider stores the group *id* it was authored with - a
        //! fresh id each run would orphan every collider that referenced it.
        static const AzPhysics::CollisionGroups::Id AllGroupId;
        static const AzPhysics::CollisionGroups::Id NoneGroupId;

        static constexpr unsigned int DefaultMaxBodies = 65536;
        static constexpr unsigned int DefaultNumBodyMutexes = 128;
        static constexpr unsigned int DefaultMaxBodyPairs = 65536;
        static constexpr unsigned int DefaultMaxContactConstraints = 16384;
        static constexpr unsigned int DefaultTempAllocatorSize = 256 * 1024 * 1024;

        unsigned int m_maxBodies = DefaultMaxBodies;
        unsigned int m_numBodyMutexes = DefaultNumBodyMutexes;
        unsigned int m_maxBodyPairs = DefaultMaxBodyPairs;
        unsigned int m_maxContactConstraints = DefaultMaxContactConstraints;
        unsigned int m_tempAllocatorSize = DefaultTempAllocatorSize;
        unsigned int m_maxJobThreads = 0;

        //! Solver settings, mapped onto JPH::PhysicsSettings for every scene's physics
        //! system. Defaults match Jolt's. Unlike the capacity settings above (which
        //! only apply when a scene's physics system is created), these apply to live
        //! scenes immediately on UpdateConfiguration.
        unsigned int m_numVelocitySteps = 10; //!< Solver velocity iterations per step.
        unsigned int m_numPositionSteps = 2; //!< Solver position iterations per step.
        float m_baumgarte = 0.2f; //!< Fraction of penetration resolved per step.
        float m_speculativeContactDistance = 0.02f; //!< Radius around objects where speculative contacts form (m).
        float m_penetrationSlop = 0.02f; //!< Allowed body overlap, keeps contacts stable (m).
        float m_timeBeforeSleep = 0.5f; //!< Seconds below the sleep threshold before a body sleeps.
        float m_pointVelocitySleepThreshold = 0.03f; //!< Max velocity (m/s) of a body's sleep-test points for it to count as resting.
        bool m_allowSleeping = true; //!< Whether bodies may sleep at all.
        bool m_deterministicSimulation = true; //!< Deterministic stepping; turning it off buys performance.

        //! Extra work to suppress "ghost" contacts against the internal edges of a mesh
        //! or heightfield - the seams between triangles, which are not real surfaces but
        //! which a sliding body can still catch on. On by default: the artefact is a
        //! correctness problem that is very hard to diagnose from content, and Jolt only
        //! defaults it off because it costs a little performance.
        //!
        //! Jolt decides per contact pair with an OR (Body::GetEnhancedInternalEdgeRemovalWithBody),
        //! so this is applied to the bodies that move - dynamic/kinematic rigid bodies and
        //! characters. Static bodies gain nothing from carrying the flag themselves.
        bool m_enhancedInternalEdgeRemoval = true;

        //! How close two vertices must be before the edge-removal algorithm treats them as
        //! the same vertex, and therefore the edge between them as shared (m). Jolt stores
        //! this squared; it is exposed unsquared because that is the unit an author can
        //! reason about. The default is Jolt's own (1e-4 m, i.e. 1e-8 squared).
        float m_internalEdgeRemovalTolerance = 1.0e-4f;

        //! Jolt collision (sub-)steps per simulation update, for every scene. Raising
        //! it improves fast object behavior at proportional cost.
        //!
        //! System-wide by design: AzPhysics::SceneConfiguration is not polymorphic
        //! (AZ_TYPE_INFO only) and travels by value through the AzPhysics API, so a
        //! derived per-scene configuration would be sliced before any backend could
        //! read it. That is why there is no JoltSceneConfiguration.
        int m_collisionSteps = 1;
    };

} // namespace JoltPhysics
