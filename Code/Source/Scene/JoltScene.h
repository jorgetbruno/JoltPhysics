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

        void FlushTransformSync();
        void InitializeJoltSystem();

        //! Queues a trigger enter/exit event from the Jolt contact listener (job threads).
        void QueueTriggerEvent(AzPhysics::TriggerEvent::Type type, JPH::BodyID triggerBodyId, JPH::BodyID otherBodyId);

        //! Whether the body with the given Jolt id is a sensor (trigger). Lock-free;
        //! safe to call from the contact listener.
        bool IsSensorBody(JPH::BodyID bodyId) const
        {
            return m_sensorBodyIds.contains(bodyId.GetIndexAndSequenceNumber());
        }

        JPH::PhysicsSystem* GetJoltPhysicsSystem() { return m_physicsSystem.get(); }
        JPH::BodyInterface* GetBodyInterface() { return m_bodyInterface; }

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
