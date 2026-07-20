#include <Scene/JoltScene.h>
#include <Scene/JoltSceneQueryHelpers.h>
#include <System/JoltSystem.h>
#include <RigidBody/JoltRigidBody.h>
#include <RigidBody/JoltStaticRigidBody.h>
#include <Utils/Conversions.h>

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Console/ILogger.h>

#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>

namespace JoltPhysics
{
    AZ_CLASS_ALLOCATOR_IMPL(JoltScene, AZ::SystemAllocator);

    JoltScene::JoltScene(const AzPhysics::SceneConfiguration& config,
                        const AzPhysics::SceneHandle& sceneHandle)
        : m_config(config)
        , m_sceneHandle(sceneHandle)
        , m_gravity(config.m_gravity)
    {
    }

    JoltScene::~JoltScene()
    {
        ClearDeferredDeletions();

        for (auto& [crc, body] : m_simulatedBodies)
        {
            delete body;
        }
        m_simulatedBodies.clear();

        for (auto& [crc, joint] : m_joints)
        {
            delete joint;
        }
        m_joints.clear();

        m_contactListener.reset();
        m_activationListener.reset();
        m_physicsSystem.reset();
    }

    void JoltScene::InitializeJoltSystem()
    {
        JoltSystem* joltSystem = GetJoltSystem();
        if (!joltSystem)
        {
            AZ_Error("JoltPhysics", false, "JoltSystem not available");
            return;
        }

        m_tempAllocator = joltSystem->GetJoltAllocator();
        m_jobSystem = joltSystem->GetJoltJobSystem();

        m_physicsSystem = AZStd::make_unique<JPH::PhysicsSystem>();
        m_physicsSystem->Init(
            MaxBodies,
            NumBodyMutexes,
            MaxBodyPairs,
            MaxContactConstraints,
            joltSystem->GetBroadPhaseLayerInterface(),
            joltSystem->GetObjectVsBroadPhaseLayerFilter(),
            joltSystem->GetObjectLayerPairFilter()
        );

        m_bodyInterface = &m_physicsSystem->GetBodyInterface();

        m_contactListener = AZStd::make_unique<JoltContactListener>(this);
        m_activationListener = AZStd::make_unique<JoltBodyActivationListener>(this);

        m_physicsSystem->SetContactListener(m_contactListener.get());
        m_physicsSystem->SetBodyActivationListener(m_activationListener.get());

        m_physicsSystem->SetGravity(Conversions::ToJolt(m_gravity));

        AZLOG_INFO("JoltPhysics: Scene '%s' initialized", m_config.m_sceneName.c_str());
    }

    void JoltScene::StartSimulation(float deltaTime)
    {
        if (!m_isEnabled || !m_physicsSystem)
        {
            return;
        }

        m_currentDeltaTime = deltaTime;

        m_physicsSystem->Update(deltaTime, m_collisionSteps, m_tempAllocator, m_jobSystem);
    }

    void JoltScene::FinishSimulation()
    {
        if (!m_isEnabled || !m_physicsSystem)
        {
            return;
        }

        FlushTransformSync();
        FlushQueuedEvents();
        ClearDeferredDeletions();
    }

    void JoltScene::SetEnabled(bool enable)
    {
        m_isEnabled = enable;
    }

    bool JoltScene::IsEnabled() const
    {
        return m_isEnabled;
    }

    const AzPhysics::SceneConfiguration& JoltScene::GetConfiguration() const
    {
        return m_config;
    }

    void JoltScene::UpdateConfiguration(const AzPhysics::SceneConfiguration& config)
    {
        m_config = config;
        m_gravity = config.m_gravity;

        if (m_physicsSystem)
        {
            m_physicsSystem->SetGravity(Conversions::ToJolt(m_gravity));
        }
    }

    AzPhysics::SimulatedBodyHandle JoltScene::AddSimulatedBody(
        const AzPhysics::SimulatedBodyConfiguration* simulatedBodyConfig)
    {
        if (!simulatedBodyConfig || !m_bodyInterface)
        {
            return AzPhysics::InvalidSimulatedBodyHandle;
        }

        AzPhysics::SimulatedBodyIndex bodyIndex;
        if (!m_freeSceneSlots.empty())
        {
            bodyIndex = m_freeSceneSlots.front();
            m_freeSceneSlots.pop();
        }
        else
        {
            bodyIndex = static_cast<AzPhysics::SimulatedBodyIndex>(m_simulatedBodies.size());
            m_simulatedBodies.emplace_back(AZ::Crc32(), nullptr);
        }

        AzPhysics::SimulatedBody* body = nullptr;

        if (const auto* rigidBodyConfig = azdynamic_cast<const AzPhysics::RigidBodyConfiguration*>(simulatedBodyConfig))
        {
            auto* rigidBody = aznew JoltRigidBody(*rigidBodyConfig);
            rigidBody->CreateInScene(this);
            body = rigidBody;
        }
        else if (const auto* staticBodyConfig = azdynamic_cast<const AzPhysics::StaticRigidBodyConfiguration*>(simulatedBodyConfig))
        {
            auto* staticBody = aznew JoltStaticRigidBody(*staticBodyConfig);
            staticBody->CreateInScene(this);
            body = staticBody;
        }

        if (!body)
        {
            m_freeSceneSlots.push(bodyIndex);
            return AzPhysics::InvalidSimulatedBodyHandle;
        }

        AZ::Crc32 bodyCrc(simulatedBodyConfig->m_debugName.c_str());
        AzPhysics::SimulatedBodyHandle handle(bodyCrc, bodyIndex);

        m_simulatedBodies[bodyIndex] = { bodyCrc, body };
        body->m_sceneOwner = m_sceneHandle;
        body->m_bodyHandle = handle;

        return handle;
    }

    AzPhysics::SimulatedBodyHandleList JoltScene::AddSimulatedBodies(
        const AzPhysics::SimulatedBodyConfigurationList& simulatedBodyConfigs)
    {
        AzPhysics::SimulatedBodyHandleList handles;
        handles.reserve(simulatedBodyConfigs.size());

        for (const auto& config : simulatedBodyConfigs)
        {
            handles.push_back(AddSimulatedBody(config.get()));
        }

        return handles;
    }

    AzPhysics::SimulatedBody* JoltScene::GetSimulatedBodyFromHandle(AzPhysics::SimulatedBodyHandle bodyHandle)
    {
        if (bodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
        {
            return nullptr;
        }

        const auto index = AZStd::get<AzPhysics::SimulatedBodyIndex>(bodyHandle);
        if (index < m_simulatedBodies.size())
        {
            return m_simulatedBodies[index].second;
        }

        return nullptr;
    }

    AzPhysics::SimulatedBodyList JoltScene::GetSimulatedBodiesFromHandle(
        const AzPhysics::SimulatedBodyHandleList& bodyHandles)
    {
        AzPhysics::SimulatedBodyList bodies;
        bodies.reserve(bodyHandles.size());

        for (const auto& handle : bodyHandles)
        {
            bodies.push_back(GetSimulatedBodyFromHandle(handle));
        }

        return bodies;
    }

    void JoltScene::RemoveSimulatedBody(AzPhysics::SimulatedBodyHandle& bodyHandle)
    {
        if (bodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
        {
            return;
        }

        const auto index = AZStd::get<AzPhysics::SimulatedBodyIndex>(bodyHandle);
        if (index < m_simulatedBodies.size() && m_simulatedBodies[index].second)
        {
            m_deferredDeletions.push_back(m_simulatedBodies[index].second);
            m_simulatedBodies[index] = { AZ::Crc32(), nullptr };
            m_freeSceneSlots.push(index);
        }

        bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
    }

    void JoltScene::RemoveSimulatedBodies(AzPhysics::SimulatedBodyHandleList& bodyHandles)
    {
        for (auto& handle : bodyHandles)
        {
            RemoveSimulatedBody(handle);
        }
    }

    void JoltScene::EnableSimulationOfBody(AzPhysics::SimulatedBodyHandle bodyHandle)
    {
        if (AzPhysics::SimulatedBody* body = GetSimulatedBodyFromHandle(bodyHandle))
        {
            EnableSimulationOfBodyInternal(*body);
        }
    }

    void JoltScene::DisableSimulationOfBody(AzPhysics::SimulatedBodyHandle bodyHandle)
    {
        if (AzPhysics::SimulatedBody* body = GetSimulatedBodyFromHandle(bodyHandle))
        {
            DisableSimulationOfBodyInternal(*body);
        }
    }

    void JoltScene::EnableSimulationOfBodyInternal([[maybe_unused]] AzPhysics::SimulatedBody& body)
    {
        // TODO: Implement body activation in Jolt
    }

    void JoltScene::DisableSimulationOfBodyInternal([[maybe_unused]] AzPhysics::SimulatedBody& body)
    {
        // TODO: Implement body deactivation in Jolt
    }

    AzPhysics::JointHandle JoltScene::AddJoint(
        [[maybe_unused]] const AzPhysics::JointConfiguration* jointConfig,
        [[maybe_unused]] AzPhysics::SimulatedBodyHandle parentBody,
        [[maybe_unused]] AzPhysics::SimulatedBodyHandle childBody)
    {
        // TODO: Implement joint creation
        return AzPhysics::InvalidJointHandle;
    }

    AzPhysics::Joint* JoltScene::GetJointFromHandle(AzPhysics::JointHandle jointHandle)
    {
        if (jointHandle == AzPhysics::InvalidJointHandle)
        {
            return nullptr;
        }

        const auto index = AZStd::get<AzPhysics::JointIndex>(jointHandle);
        if (index < m_joints.size())
        {
            return m_joints[index].second;
        }

        return nullptr;
    }

    void JoltScene::RemoveJoint(AzPhysics::JointHandle jointHandle)
    {
        if (jointHandle == AzPhysics::InvalidJointHandle)
        {
            return;
        }

        const auto index = AZStd::get<AzPhysics::JointIndex>(jointHandle);
        if (index < m_joints.size() && m_joints[index].second)
        {
            m_deferredDeletionsJoints.push_back(m_joints[index].second);
            m_joints[index] = { AZ::Crc32(), nullptr };
            m_freeJointSlots.push(index);
        }
    }

    AzPhysics::SceneQueryHits JoltScene::QueryScene(const AzPhysics::SceneQueryRequest* request)
    {
        AzPhysics::SceneQueryHits result;
        QueryScene(request, result);
        return result;
    }

    bool JoltScene::QueryScene(const AzPhysics::SceneQueryRequest* request, AzPhysics::SceneQueryHits& result)
    {
        if (!request || !m_physicsSystem)
        {
            return false;
        }

        return JoltSceneQueryHelpers::QueryScene(m_physicsSystem.get(), request, result);
    }

    AzPhysics::SceneQueryHitsList JoltScene::QuerySceneBatch(const AzPhysics::SceneQueryRequests& requests)
    {
        AzPhysics::SceneQueryHitsList results;
        results.reserve(requests.size());

        for (const auto& request : requests)
        {
            results.push_back(QueryScene(request.get()));
        }

        return results;
    }

    bool JoltScene::QuerySceneAsync(
        [[maybe_unused]] AzPhysics::SceneQuery::AsyncRequestId requestId,
        [[maybe_unused]] const AzPhysics::SceneQueryRequest* request,
        [[maybe_unused]] AzPhysics::SceneQuery::AsyncCallback callback)
    {
        // TODO: Implement async scene queries
        return false;
    }

    bool JoltScene::QuerySceneAsyncBatch(
        [[maybe_unused]] AzPhysics::SceneQuery::AsyncRequestId requestId,
        [[maybe_unused]] const AzPhysics::SceneQueryRequests& requests,
        [[maybe_unused]] AzPhysics::SceneQuery::AsyncBatchCallback callback)
    {
        // TODO: Implement async batch scene queries
        return false;
    }

    void JoltScene::SuppressCollisionEvents(
        [[maybe_unused]] const AzPhysics::SimulatedBodyHandle& bodyHandleA,
        [[maybe_unused]] const AzPhysics::SimulatedBodyHandle& bodyHandleB)
    {
        // TODO: Implement collision event suppression
    }

    void JoltScene::UnsuppressCollisionEvents(
        [[maybe_unused]] const AzPhysics::SimulatedBodyHandle& bodyHandleA,
        [[maybe_unused]] const AzPhysics::SimulatedBodyHandle& bodyHandleB)
    {
        // TODO: Implement collision event unsuppression
    }

    void JoltScene::SetGravity(const AZ::Vector3& gravity)
    {
        m_gravity = gravity;
        if (m_physicsSystem)
        {
            m_physicsSystem->SetGravity(Conversions::ToJolt(gravity));
        }
    }

    AZ::Vector3 JoltScene::GetGravity() const
    {
        return m_gravity;
    }

    void* JoltScene::GetNativePointer() const
    {
        return m_physicsSystem.get();
    }

    void JoltScene::FlushTransformSync()
    {
        for (auto& [crc, body] : m_simulatedBodies)
        {
            if (body)
            {
                if (auto* rigidBody = azdynamic_cast<JoltRigidBody*>(body))
                {
                    rigidBody->SyncTransform();
                }
            }
        }
    }

    void JoltScene::FlushQueuedEvents()
    {
        ProcessTriggerEvents();
        ProcessCollisionEvents();
        m_queuedActiveBodyIndices.Clear();
    }

    void JoltScene::ClearDeferredDeletions()
    {
        for (auto* body : m_deferredDeletions)
        {
            delete body;
        }
        m_deferredDeletions.clear();

        for (auto* joint : m_deferredDeletionsJoints)
        {
            delete joint;
        }
        m_deferredDeletionsJoints.clear();
    }

    void JoltScene::ProcessTriggerEvents()
    {
        // TODO: Process accumulated trigger events
    }

    void JoltScene::ProcessCollisionEvents()
    {
        // TODO: Process accumulated collision events
    }

    void JoltScene::SyncActiveBodyTransform(
        [[maybe_unused]] const AzPhysics::SimulatedBodyHandleList& activeBodyHandles)
    {
        // Handled in FlushTransformSync
    }

    void JoltScene::QueuedActiveBodyIndices::Insert(AzPhysics::SimulatedBodyIndex bodyIndex)
    {
        if (m_uniqueIndices.insert(bodyIndex).second)
        {
            m_packedIndices.push_back(bodyIndex);
        }
    }

    void JoltScene::QueuedActiveBodyIndices::IncreaseCapacity(size_t extraSize)
    {
        m_packedIndices.reserve(m_packedIndices.size() + extraSize);
    }

    void JoltScene::QueuedActiveBodyIndices::Clear()
    {
        m_uniqueIndices.clear();
        m_packedIndices.clear();
    }

    void JoltScene::QueuedActiveBodyIndices::Apply(
        const AZStd::function<void(AzPhysics::SimulatedBodyIndex)>& applyFunction)
    {
        for (const auto index : m_packedIndices)
        {
            applyFunction(index);
        }
    }

    JPH::ValidateResult JoltContactListener::OnContactValidate(
        [[maybe_unused]] const JPH::Body& inBody1,
        [[maybe_unused]] const JPH::Body& inBody2,
        [[maybe_unused]] JPH::RVec3Arg inBaseOffset,
        [[maybe_unused]] const JPH::CollideShapeResult& inCollisionResult)
    {
        return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
    }

    void JoltContactListener::OnContactAdded(
        [[maybe_unused]] const JPH::Body& inBody1,
        [[maybe_unused]] const JPH::Body& inBody2,
        [[maybe_unused]] const JPH::ContactManifold& inManifold,
        [[maybe_unused]] JPH::ContactSettings& ioSettings)
    {
        // TODO: Queue collision begin event
    }

    void JoltContactListener::OnContactPersisted(
        [[maybe_unused]] const JPH::Body& inBody1,
        [[maybe_unused]] const JPH::Body& inBody2,
        [[maybe_unused]] const JPH::ContactManifold& inManifold,
        [[maybe_unused]] JPH::ContactSettings& ioSettings)
    {
        // TODO: Queue collision persist event
    }

    void JoltContactListener::OnContactRemoved([[maybe_unused]] const JPH::SubShapeIDPair& inSubShapePair)
    {
        // TODO: Queue collision end event
    }

    void JoltBodyActivationListener::OnBodyActivated(
        [[maybe_unused]] const JPH::BodyID& inBodyID,
        [[maybe_unused]] AZ::u64 inBodyUserData)
    {
        // TODO: Handle body activation
    }

    void JoltBodyActivationListener::OnBodyDeactivated(
        [[maybe_unused]] const JPH::BodyID& inBodyID,
        [[maybe_unused]] AZ::u64 inBodyUserData)
    {
        // TODO: Handle body deactivation
    }

} // namespace JoltPhysics
