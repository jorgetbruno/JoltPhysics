#pragma once

#include <AzCore/EBus/Event.h>
#include <AzCore/std/containers/span.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/containers/queue.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/functional.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>

#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/Collision/CollisionEvents.h>
#include <AzFramework/Physics/Common/PhysicsJoint.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <AzFramework/Physics/Configuration/SceneConfiguration.h>

#include <JoltPhysics/Configuration/JoltConfiguration.h>
#include <JoltPhysics/JoltSoftBodyBus.h>

#include <AzCore/std/parallel/mutex.h>
#include <AzCore/std/parallel/shared_mutex.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/SoftBody/SoftBodyContactListener.h>
#include <Jolt/Physics/SoftBody/SoftBodyManifold.h>

namespace JPH
{
    class PhysicsSystem;
    class BodyInterface;
    class Body;
    class TempAllocatorImpl;
    class JobSystemThreadPool;
}

namespace JoltPhysics
{
    class JoltContactListener;
    class JoltSoftBodyContactListener;
    class JoltBodyActivationListener;

    class JoltScene final : public AzPhysics::Scene
    {
    public:
        AZ_CLASS_ALLOCATOR_DECL
        AZ_RTTI(JoltScene, "{5F024E70-F590-4C6B-A150-996998F98D50}", AzPhysics::Scene)

        explicit JoltScene(const AzPhysics::SceneConfiguration& config,
                          const AzPhysics::SceneHandle& sceneHandle);
        ~JoltScene() override;

        void StartSimulation(float deltaTime) override;
        void FinishSimulation() override;
        void SetEnabled(bool enable) override;
        [[nodiscard]] bool IsEnabled() const override;
        [[nodiscard]] const AzPhysics::SceneConfiguration& GetConfiguration() const override;
        void UpdateConfiguration(const AzPhysics::SceneConfiguration& config) override;

        //! Applies the solver portion of the system configuration (JPH::PhysicsSettings
        //! and collision steps) to this scene's physics system. Called on scene
        //! creation and again whenever the system configuration changes, so solver
        //! edits reach live scenes. Capacity settings are Init-time only.
        void ApplySystemConfiguration(const JoltSystemConfiguration& config);

        //! Jolt collision sub-steps this scene runs per simulation update.
        [[nodiscard]] int GetCollisionSteps() const
        {
            return m_collisionSteps;
        }

        AzPhysics::SimulatedBodyHandle AddSimulatedBody(
            const AzPhysics::SimulatedBodyConfiguration* simulatedBodyConfig) override;
        AzPhysics::SimulatedBodyHandleList AddSimulatedBodies(
            const AzPhysics::SimulatedBodyConfigurationList& simulatedBodyConfigs) override;
        AzPhysics::SimulatedBody* GetSimulatedBodyFromHandle(
            AzPhysics::SimulatedBodyHandle bodyHandle) override;
        AzPhysics::SimulatedBodyList GetSimulatedBodiesFromHandle(
            const AzPhysics::SimulatedBodyHandleList& bodyHandles) override;
        void RemoveSimulatedBody(AzPhysics::SimulatedBodyHandle& bodyHandle) override;
        void RemoveSimulatedBodies(AzPhysics::SimulatedBodyHandleList& bodyHandles) override;
        void EnableSimulationOfBody(AzPhysics::SimulatedBodyHandle bodyHandle) override;
        void DisableSimulationOfBody(AzPhysics::SimulatedBodyHandle bodyHandle) override;

        AzPhysics::JointHandle AddJoint(const AzPhysics::JointConfiguration* jointConfig,
                                       AzPhysics::SimulatedBodyHandle parentBody,
                                       AzPhysics::SimulatedBodyHandle childBody) override;
        AzPhysics::Joint* GetJointFromHandle(AzPhysics::JointHandle jointHandle) override;
        void RemoveJoint(AzPhysics::JointHandle jointHandle) override;

        //! Fires with the handle of a breakable joint the scene just removed because its
        //! reaction exceeded the configured break force/torque. The joint is already gone
        //! when the event fires, so holders of the handle must drop it, not remove it.
        //! (AzPhysics has no joint events, so this lives on the Jolt scene.)
        using JointBreakEvent = AZ::Event<AzPhysics::JointHandle>;
        void RegisterJointBreakHandler(JointBreakEvent::Handler& handler)
        {
            handler.Connect(m_jointBreakEvent);
        }

        //! Snapshots the complete simulation state - body positions and velocities
        //! (including soft body vertices), contacts, constraints, and the characters,
        //! which Jolt's own SaveState does not cover - into an opaque blob, appended to
        //! outState. For rollback and replay, not persistence: the blob only restores
        //! into a scene with the same bodies, joints and characters in the same slots,
        //! and replaying deterministically additionally requires the same binary.
        //! (AzPhysics has no snapshot API, so this lives on the Jolt scene; also
        //! reachable through JoltPhysicsSystemRequests by scene handle.)
        bool SaveSimulationState(AZStd::vector<AZ::u8>& outState) const;

        //! Restores a snapshot taken by SaveSimulationState. Returns false when the blob
        //! does not match this scene (different bodies, characters, or a truncated or
        //! foreign blob) - and the scene may then be PARTIALLY restored, since Jolt
        //! applies what it reads as it reads it: treat a false return as "restore from a
        //! good snapshot or rebuild", not "carry on". On success entity transforms are
        //! synced immediately rather than on the next simulation step.
        bool RestoreSimulationState(AZStd::span<const AZ::u8> state);

        AzPhysics::SceneQueryHits QueryScene(const AzPhysics::SceneQueryRequest* request) override;
        bool QueryScene(const AzPhysics::SceneQueryRequest* request,
                       AzPhysics::SceneQueryHits& result) override;
        AzPhysics::SceneQueryHitsList QuerySceneBatch(
            const AzPhysics::SceneQueryRequests& requests) override;
        [[nodiscard]] bool QuerySceneAsync(AzPhysics::SceneQuery::AsyncRequestId requestId,
                                          const AzPhysics::SceneQueryRequest* request,
                                          AzPhysics::SceneQuery::AsyncCallback callback) override;
        [[nodiscard]] bool QuerySceneAsyncBatch(
            AzPhysics::SceneQuery::AsyncRequestId requestId,
            const AzPhysics::SceneQueryRequests& requests,
            AzPhysics::SceneQuery::AsyncBatchCallback callback) override;

        void SuppressCollisionEvents(const AzPhysics::SimulatedBodyHandle& bodyHandleA,
                                    const AzPhysics::SimulatedBodyHandle& bodyHandleB) override;
        void UnsuppressCollisionEvents(const AzPhysics::SimulatedBodyHandle& bodyHandleA,
                                      const AzPhysics::SimulatedBodyHandle& bodyHandleB) override;

        //! PhysX parity: bodies connected by a joint do not collide with each other.
        //! AddJoint registers the pair and RemoveJoint unregisters it; the contact
        //! listener rejects contact generation for registered pairs.
        void SetJointCollisionEnabled(JPH::BodyID bodyIdA, JPH::BodyID bodyIdB, bool enabled);
        //! True when the two bodies are connected by a joint (order-independent).
        bool AreBodiesJointed(JPH::BodyID bodyIdA, JPH::BodyID bodyIdB) const;

        void SetGravity(const AZ::Vector3& gravity) override;
        AZ::Vector3 GetGravity() const override;

        AzPhysics::SceneHandle GetSceneHandle() const { return m_sceneHandle; }
        const AZStd::vector<AZStd::pair<AZ::Crc32, AzPhysics::SimulatedBody*>>& GetSimulatedBodyList() const
        {
            return m_simulatedBodies;
        }

        void* GetNativePointer() const override;

        //! Maps a Jolt body id back to the scene's simulated body handle (for queries and events).
        AzPhysics::SimulatedBodyHandle GetBodyHandleFromJoltId(JPH::BodyID bodyId) const;

        //! Resolves the {friction, restitution} for a contact sub-shape: the per-collider
        //! material when the body is a compound (sub-shape index maps to collider order),
        //! otherwise the body's own material. Returns false when the body-level
        //! contact settings should be used as-is.
        bool GetMaterialForSubShape(
            const JPH::Body& body, const JPH::SubShapeID& subShapeId, float& outFriction, float& outRestitution);

        //! The collider configuration behind a contact or query sub-shape, or null when the
        //! body is not one this scene tracks per collider. Reads the live configuration, so
        //! a runtime change applies to existing bodies.
        const Physics::ColliderConfiguration* GetColliderConfigurationForSubShape(
            JPH::BodyID bodyId, const JPH::SubShapeID& subShapeId) const;

        //! Whether a sub-shape's collider takes part in simulation / in scene queries.
        //! Both default to true, so a body the scene does not track answers true and
        //! behaves exactly as it did before these flags were honored.
        bool IsColliderSimulated(JPH::BodyID bodyId, const JPH::SubShapeID& subShapeId) const;
        bool IsColliderInSceneQueries(JPH::BodyID bodyId, const JPH::SubShapeID& subShapeId) const;

        void InitializeJoltSystem();

        //! Queues a trigger enter/exit event from the Jolt contact listener (job threads).
        void QueueTriggerEvent(AzPhysics::TriggerEvent::Type type, JPH::BodyID triggerBodyId, JPH::BodyID otherBodyId);

        //! True the first time a body pair persists in a given step, so Persist is raised
        //! once per pair rather than once per touching sub-shape. Cleared each flush.
        bool TrackPairPersistedThisStep(JPH::BodyID bodyId1, JPH::BodyID bodyId2);

        //! Queues a collision begin/persist event from the Jolt contact listener (job threads).
        //! The contact list is built from the manifold here, while Jolt still has both
        //! bodies locked; the body pointers are resolved later on the main thread.
        void QueueCollisionEvent(AzPhysics::CollisionEvent::Type type,
            JPH::BodyID body1Id, JPH::BodyID body2Id, const JPH::ContactManifold& manifold);

        //! Queues a collision end event. No manifold is available on contact removal,
        //! so the resulting event carries an empty contact list.
        void QueueCollisionEndEvent(JPH::BodyID body1Id, JPH::BodyID body2Id);

        //! Queues a Begin or Persist event for a soft body touching another body, and
        //! records the pair as touching this step. Jolt reports soft body contacts through
        //! its own listener, once per soft body per step, with a per-particle manifold
        //! rather than a shape manifold - so the contacts are the colliding particles.
        void QueueSoftBodyCollisionEvent(
            JPH::BodyID softBodyId, JPH::BodyID otherBodyId, AZStd::vector<AzPhysics::Contact>&& contacts);

        //! Emits End events for soft body pairs that stopped touching. Soft body contacts
        //! have no removal callback - Jolt simply stops reporting them - so a pair that was
        //! not refreshed this step has ended.
        void FlushEndedSoftBodyContacts();

        //! Queues the per-particle detail of a soft body touching another body, dispatched
        //! on JoltSoftBodyNotificationBus after the step. The generic collision events
        //! above carry positions only; listeners on the notification bus get the particle
        //! indices as well.
        void QueueSoftBodyParticleContacts(
            JPH::BodyID softBodyId, JPH::BodyID otherBodyId, AZStd::vector<JoltSoftBodyParticleContact>&& contacts);

        //! Records the first/last touching sub-shape of a body pair so collision Begin/End
        //! fire once per pair. TrackContactAdded returns true on the first contact of the
        //! pair; TrackContactRemoved returns true when the last contact is removed (false
        //! if the pair was already cleared because a body was removed mid-contact).
        bool TrackContactAdded(JPH::BodyID bodyId1, JPH::BodyID bodyId2);
        bool TrackContactRemoved(JPH::BodyID bodyId1, JPH::BodyID bodyId2);

        //! Records the first/last overlapping sub-shape of a (sensor, other) pair so trigger
        //! Enter/Exit fire once per pair. TrackTriggerOverlapAdded returns true on the first
        //! overlap; TrackTriggerOverlapRemoved returns true when the last overlap ends
        //! (false if the pair was already cleared because a body was removed while overlapping).
        bool TrackTriggerOverlapAdded(JPH::BodyID sensorId, JPH::BodyID otherId);
        bool TrackTriggerOverlapRemoved(JPH::BodyID sensorId, JPH::BodyID otherId);

        //! Whether the body with the given Jolt id is a sensor (trigger). Lock-free;
        //! safe to call from the contact listener.
        bool IsSensorBody(JPH::BodyID bodyId) const
        {
            return m_sensorBodyIds.contains(bodyId.GetIndexAndSequenceNumber());
        }

        JPH::PhysicsSystem* GetJoltPhysicsSystem() { return m_physicsSystem.get(); }
        JPH::BodyInterface* GetBodyInterface() { return m_bodyInterface; }

        //! Resolves a simulated body handle to its Jolt body (nullptr for characters
        //! and invalid handles).
        JPH::Body* GetJoltBody(AzPhysics::SimulatedBodyHandle bodyHandle);

        float GetCurrentDeltaTime() const { return m_currentDeltaTime; }

    private:
        class QueuedActiveBodyIndices
        {
        public:
            void Insert(AzPhysics::SimulatedBodyIndex bodyIndex);
            void IncreaseCapacity(size_t extraSize);
            void Clear();
            void Apply(const AZStd::function<void(AzPhysics::SimulatedBodyIndex)>& applyFunction);

        private:
            AZStd::unordered_set<AzPhysics::SimulatedBodyIndex> m_uniqueIndices;
            AZStd::vector<AzPhysics::SimulatedBodyIndex> m_packedIndices;
        };

        void EnableSimulationOfBodyInternal(AzPhysics::SimulatedBody& body);
        void DisableSimulationOfBodyInternal(AzPhysics::SimulatedBody& body);

        //! Runs the scene queries queued by QuerySceneAsync/QuerySceneAsyncBatch and
        //! invokes their callbacks. Called once per FinishSimulation, so results reflect
        //! the state the step just produced.
        void FlushAsyncSceneQueries();

        void FlushQueuedEvents();
        //! Dispatches the queued per-particle soft body contacts on
        //! JoltSoftBodyNotificationBus, resolving each body's entity from its user data.
        void FlushSoftBodyParticleContacts();
        void ClearDeferredDeletions();
        void ProcessJointBreaking();
        void ProcessTriggerEvents();
        void ProcessCollisionEvents();

        //! Builds+queues a collision End event from two resolved body handles.
        void EnqueueCollisionEndEvent(AzPhysics::SimulatedBodyHandle handle1, AzPhysics::SimulatedBodyHandle handle2);
        //! Queues collision End events to every still-alive body currently in contact with
        //! the body being removed (Jolt's own OnContactRemoved for these pairs would arrive
        //! a step too late, after the removed body's id->handle mapping is gone).
        void FlushCollisionEndsForRemovedBody(JPH::BodyID removedBodyId, AzPhysics::SimulatedBodyHandle removedHandle);
        //! Queues trigger Exit events to every sensor the body being removed is overlapping
        //! (Jolt's own OnContactRemoved for these overlaps would arrive a step too late).
        void FlushTriggerExitsForRemovedBody(JPH::BodyID removedBodyId);
        //! Normalized, order-independent key for a pair of Jolt bodies.
        static AZ::u64 MakeContactPairKey(JPH::BodyID bodyId1, JPH::BodyID bodyId2);

        //! Normalized, order-independent key for a pair of simulated body handles.
        static AZ::u64 MakeBodyHandlePairKey(
            const AzPhysics::SimulatedBodyHandle& bodyHandleA, const AzPhysics::SimulatedBodyHandle& bodyHandleB);
        //! Drops every suppression entry referencing the given body (called when it is
        //! removed, so a recycled scene slot does not inherit stale suppressions).
        void RemoveCollisionSuppressionsForBody(const AzPhysics::SimulatedBodyHandle& bodyHandle);

        void SyncActiveBodyTransform(const AzPhysics::SimulatedBodyHandleList& activeBodyHandles);

        bool m_isEnabled = true;

        QueuedActiveBodyIndices m_queuedActiveBodyIndices;
        float m_accumulatedDeltaTime = 0.0f;

        AzPhysics::SceneConfiguration m_config;
        AzPhysics::SceneHandle m_sceneHandle;

        float m_currentDeltaTime = 0.0f;

        AZStd::vector<AZStd::pair<AZ::Crc32, AzPhysics::SimulatedBody*>> m_simulatedBodies;
        AZStd::vector<AzPhysics::SimulatedBody*> m_deferredDeletions;
        AZStd::queue<AzPhysics::SimulatedBodyIndex> m_freeSceneSlots;

        AZStd::unordered_map<AZ::u32, AzPhysics::SimulatedBodyHandle> m_bodyHandleByJoltId;
        AZStd::unordered_set<AZ::u32> m_sensorBodyIds;

        AZStd::mutex m_triggerEventMutex;
        AzPhysics::TriggerEventList m_queuedTriggerEvents;

        AZStd::mutex m_collisionEventMutex;

        //! Soft body pairs currently touching, and the step each was last reported on.
        struct SoftBodyContactPair
        {
            JPH::BodyID m_softBodyId;
            JPH::BodyID m_otherBodyId;
            AZ::u64 m_lastSeenStep = 0;
        };
        AZStd::unordered_map<AZ::u64, SoftBodyContactPair> m_softBodyContacts;
        AZStd::mutex m_softBodyContactsMutex;

        //! Per-particle contact payloads collected during the step, one entry per
        //! (soft body, other body) pair, dispatched after the step.
        struct QueuedSoftBodyParticleContacts
        {
            JPH::BodyID m_softBodyId;
            JPH::BodyID m_otherBodyId;
            AZStd::vector<JoltSoftBodyParticleContact> m_contacts;
        };
        AZStd::vector<QueuedSoftBodyParticleContacts> m_queuedSoftBodyParticleContacts;
        AZStd::mutex m_softBodyParticleContactsMutex;
        AZ::u64 m_simulationStep = 0;
        AzPhysics::CollisionEventList m_queuedCollisionEvents;
        //! Swapped with m_queuedCollisionEvents each flush instead of moving into a fresh
        //! local, so the buffer the contact callbacks fill keeps the capacity it grew to
        //! rather than reallocating from nothing every step.
        AzPhysics::CollisionEventList m_collisionEventScratch;

        //! Live sub-shape contact count per body pair (normalized key -> count). Lets
        //! collision Begin/End fire once per pair and lets a removed body synthesize End
        //! events for its partners. Touched from Jolt's contact callbacks (job threads).
        AZStd::mutex m_activeContactsMutex;
        AZStd::unordered_map<AZ::u64, int> m_activeContacts;

        //! Body pairs that have already queued a Persist event this step. A pair touching
        //! through several sub-shapes persists through each of them, and raising one event
        //! per manifold means a compound body pays for its own complexity every step while
        //! reporting the same collision several times - PhysX raises one per pair.
        AZStd::mutex m_persistedPairsMutex;
        AZStd::unordered_set<AZ::u64> m_persistedPairsThisStep;

        //! Live overlapping sub-shape count per directed (sensor, other) pair. Lets trigger
        //! Enter/Exit fire once per pair and lets a body removed while inside a sensor
        //! synthesize the matching Exit. Touched from Jolt's contact callbacks (job threads).
        AZStd::mutex m_triggerOverlapsMutex;
        AZStd::unordered_map<AZ::u64, int> m_activeTriggerOverlaps;

        //! A scene query queued by QuerySceneAsync/QuerySceneAsyncBatch. Exactly one of
        //! the two callbacks is set, which is what distinguishes a single from a batch
        //! request (a single request is held as a one-entry list).
        struct QueuedAsyncQuery
        {
            AzPhysics::SceneQuery::AsyncRequestId m_requestId = 0;
            AzPhysics::SceneQueryRequests m_requests;
            AzPhysics::SceneQuery::AsyncCallback m_callback;
            AzPhysics::SceneQuery::AsyncBatchCallback m_batchCallback;
        };
        AZStd::vector<QueuedAsyncQuery> m_queuedAsyncQueries;

        //! Body pairs whose collision events are suppressed (normalized handle-pair keys).
        //! The bodies still collide; only the event dispatch is filtered, matching the
        //! PhysX backend. Main-thread only: Suppress/Unsuppress are gameplay-side calls and
        //! the filtering happens in ProcessCollisionEvents, not in the contact callbacks.
        AZStd::unordered_set<AZ::u64> m_suppressedCollisionPairs;

        //! Body pairs connected by a joint (normalized BodyID pair keys). Read on
        //! narrow-phase worker threads by the contact listener and written from the
        //! game thread by Add/RemoveJoint, so it is guarded by a shared mutex.
        //! Counted, not a set: two joints between the same pair of bodies are two reasons
        //! to suppress their collision, and removing one of them must not re-enable it.
        //! A harness of two distance joints, or a breakable joint with a backup, is
        //! ordinary content.
        AZStd::unordered_map<AZ::u64, int> m_jointedBodyPairs;
        mutable AZStd::shared_mutex m_jointedBodyPairsMutex;
        //! Read before taking the lock: the narrowphase asks about every candidate pair,
        //! on every worker thread, and most scenes have no joints at all. Relaxed because
        //! it only decides whether to bother looking - the lock still orders the answer.
        AZStd::atomic<int> m_jointedBodyPairCount{ 0 };

        AZStd::vector<AZStd::pair<AZ::Crc32, AzPhysics::Joint*>> m_joints;
        AZStd::vector<AzPhysics::Joint*> m_deferredDeletionsJoints;
        AZStd::queue<AzPhysics::JointIndex> m_freeJointSlots;
        //! Count of live breakable joints, so the per-step break check can skip scenes
        //! that have none.
        AZ::u32 m_breakableJointCount = 0;
        JointBreakEvent m_jointBreakEvent;

        AzPhysics::SystemEvents::OnConfigurationChangedEvent::Handler m_physicsSystemConfigChanged;

        AZ::u32 m_raycastBufferSize = 32;
        AZ::u32 m_shapecastBufferSize = 32;
        AZ::u32 m_overlapBufferSize = 32;

        AZStd::unique_ptr<JPH::PhysicsSystem> m_physicsSystem;
        JPH::BodyInterface* m_bodyInterface = nullptr;

        AZStd::unique_ptr<JoltContactListener> m_contactListener;
        AZStd::unique_ptr<JoltSoftBodyContactListener> m_softBodyContactListener;
        AZStd::unique_ptr<JoltBodyActivationListener> m_activationListener;

        JPH::JobSystemThreadPool* m_jobSystem = nullptr;
        JPH::TempAllocatorImpl* m_tempAllocator = nullptr;
        int m_collisionSteps = 1;

        AZ::Vector3 m_gravity = AZ::Vector3(0.0f, 0.0f, -9.81f);
    };

    //! Bridges Jolt's separate soft-body contact path onto the scene's collision events.
    //! Soft body contacts never reach JPH::ContactListener - Jolt reports them here
    //! instead, once per soft body per step.
    class JoltSoftBodyContactListener : public JPH::SoftBodyContactListener
    {
    public:
        JoltSoftBodyContactListener(JoltScene* scene) : m_scene(scene) {}

        void OnSoftBodyContactAdded(const JPH::Body& inSoftBody, const JPH::SoftBodyManifold& inManifold) override;

    private:
        JoltScene* m_scene = nullptr;
    };

    class JoltContactListener : public JPH::ContactListener
    {
    public:
        JoltContactListener(JoltScene* scene) : m_scene(scene) {}

        JPH::ValidateResult OnContactValidate(
            const JPH::Body& inBody1,
            const JPH::Body& inBody2,
            JPH::RVec3Arg inBaseOffset,
            const JPH::CollideShapeResult& inCollisionResult) override;

        void OnContactAdded(
            const JPH::Body& inBody1,
            const JPH::Body& inBody2,
            const JPH::ContactManifold& inManifold,
            JPH::ContactSettings& ioSettings) override;

        void OnContactPersisted(
            const JPH::Body& inBody1,
            const JPH::Body& inBody2,
            const JPH::ContactManifold& inManifold,
            JPH::ContactSettings& ioSettings) override;

        void OnContactRemoved(const JPH::SubShapeIDPair& inSubShapePair) override;

    private:
        void ApplySubShapeMaterials(
            const JPH::Body& inBody1,
            const JPH::Body& inBody2,
            const JPH::ContactManifold& inManifold,
            JPH::ContactSettings& ioSettings);

        JoltScene* m_scene = nullptr;
    };

    class JoltBodyActivationListener : public JPH::BodyActivationListener
    {
    public:
        JoltBodyActivationListener(JoltScene* scene) : m_scene(scene) {}

        void OnBodyActivated(const JPH::BodyID& inBodyID, AZ::u64 inBodyUserData) override;
        void OnBodyDeactivated(const JPH::BodyID& inBodyID, AZ::u64 inBodyUserData) override;

    private:
        JoltScene* m_scene = nullptr;
    };

} // namespace JoltPhysics
