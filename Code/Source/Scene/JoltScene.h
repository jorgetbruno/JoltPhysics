#pragma once

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

#include <AzCore/std/parallel/mutex.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>

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
    class JoltBodyActivationListener;

    static constexpr unsigned int MaxBodies = 65536;
    static constexpr unsigned int NumBodyMutexes = 128;
    static constexpr unsigned int MaxBodyPairs = 65536;
    static constexpr unsigned int MaxContactConstraints = 16384;

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

        void FlushTransformSync();
        void InitializeJoltSystem();

        //! Queues a trigger enter/exit event from the Jolt contact listener (job threads).
        void QueueTriggerEvent(AzPhysics::TriggerEvent::Type type, JPH::BodyID triggerBodyId, JPH::BodyID otherBodyId);

        //! Queues a collision begin/persist event from the Jolt contact listener (job threads).
        //! The contact list is built from the manifold here, while Jolt still has both
        //! bodies locked; the body pointers are resolved later on the main thread.
        void QueueCollisionEvent(AzPhysics::CollisionEvent::Type type,
            JPH::BodyID body1Id, JPH::BodyID body2Id, const JPH::ContactManifold& manifold);

        //! Queues a collision end event. No manifold is available on contact removal,
        //! so the resulting event carries an empty contact list.
        void QueueCollisionEndEvent(JPH::BodyID body1Id, JPH::BodyID body2Id);

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

        void FlushQueuedEvents();
        void ClearDeferredDeletions();
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
        AzPhysics::CollisionEventList m_queuedCollisionEvents;

        //! Live sub-shape contact count per body pair (normalized key -> count). Lets
        //! collision Begin/End fire once per pair and lets a removed body synthesize End
        //! events for its partners. Touched from Jolt's contact callbacks (job threads).
        AZStd::mutex m_activeContactsMutex;
        AZStd::unordered_map<AZ::u64, int> m_activeContacts;

        //! Live overlapping sub-shape count per directed (sensor, other) pair. Lets trigger
        //! Enter/Exit fire once per pair and lets a body removed while inside a sensor
        //! synthesize the matching Exit. Touched from Jolt's contact callbacks (job threads).
        AZStd::mutex m_triggerOverlapsMutex;
        AZStd::unordered_map<AZ::u64, int> m_activeTriggerOverlaps;

        //! Body pairs whose collision events are suppressed (normalized handle-pair keys).
        //! The bodies still collide; only the event dispatch is filtered, matching the
        //! PhysX backend. Main-thread only: Suppress/Unsuppress are gameplay-side calls and
        //! the filtering happens in ProcessCollisionEvents, not in the contact callbacks.
        AZStd::unordered_set<AZ::u64> m_suppressedCollisionPairs;

        AZStd::vector<AZStd::pair<AZ::Crc32, AzPhysics::Joint*>> m_joints;
        AZStd::vector<AzPhysics::Joint*> m_deferredDeletionsJoints;
        AZStd::queue<AzPhysics::JointIndex> m_freeJointSlots;

        AzPhysics::SystemEvents::OnConfigurationChangedEvent::Handler m_physicsSystemConfigChanged;

        AZ::u32 m_raycastBufferSize = 32;
        AZ::u32 m_shapecastBufferSize = 32;
        AZ::u32 m_overlapBufferSize = 32;

        AZStd::unique_ptr<JPH::PhysicsSystem> m_physicsSystem;
        JPH::BodyInterface* m_bodyInterface = nullptr;

        AZStd::unique_ptr<JoltContactListener> m_contactListener;
        AZStd::unique_ptr<JoltBodyActivationListener> m_activationListener;

        JPH::JobSystemThreadPool* m_jobSystem = nullptr;
        JPH::TempAllocatorImpl* m_tempAllocator = nullptr;
        int m_collisionSteps = 1;

        AZ::Vector3 m_gravity = AZ::Vector3(0.0f, 0.0f, -9.81f);
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
