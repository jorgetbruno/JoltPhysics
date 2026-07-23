#include <Scene/JoltScene.h>
#include <Scene/JoltSceneQueryHelpers.h>
#include <Character/JoltCharacter.h>
#include <Joint/JoltJoint.h>
#include <Shape/JoltHeightfieldUtils.h>
#include <System/JoltSystem.h>
#include <RigidBody/JoltRigidBody.h>
#include <RigidBody/JoltStaticRigidBody.h>
#include <Utils/Conversions.h>

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Console/ILogger.h>

#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/Character.h>
#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>

#include <Jolt/Jolt.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>

namespace JoltPhysics
{
    AZ_CLASS_ALLOCATOR_IMPL(JoltScene, AZ::SystemAllocator);

    JoltScene::JoltScene(const AzPhysics::SceneConfiguration& config,
                        const AzPhysics::SceneHandle& sceneHandle)
        : AzPhysics::Scene(config)
        , m_config(config)
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

        // Move characters before the physics step so dynamic bodies respond to them
        // within the same step (Jolt CharacterVirtual updates are user-driven).
        for (auto& [crc, body] : m_simulatedBodies)
        {
            if (auto* character = azdynamic_cast<JoltCharacter*>(body))
            {
                character->ApplyRequestedVelocity(deltaTime);
                // Per-tick and per-step requests coincide in this backend: the scene
                // applies accumulated requests once per simulation step.
                character->ResetRequestedVelocityForTick();
            }
        }

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
        else if (const auto* characterConfig = azdynamic_cast<const Physics::CharacterConfiguration*>(simulatedBodyConfig))
        {
            auto* character = aznew JoltCharacter(*characterConfig);
            character->CreateInScene(this);
            body = character;
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

        if (auto* rigidBody = azrtti_cast<JoltRigidBody*>(body))
        {
            const AZ::u32 joltIdKey = rigidBody->GetBodyId().GetIndexAndSequenceNumber();
            m_bodyHandleByJoltId[joltIdKey] = handle;
            if (rigidBody->IsSensor())
            {
                m_sensorBodyIds.insert(joltIdKey);
            }
        }
        else if (auto* staticBody = azrtti_cast<JoltStaticRigidBody*>(body))
        {
            const AZ::u32 joltIdKey = staticBody->GetBodyId().GetIndexAndSequenceNumber();
            m_bodyHandleByJoltId[joltIdKey] = handle;
            if (staticBody->IsSensor())
            {
                m_sensorBodyIds.insert(joltIdKey);
            }
        }
        else if (auto* character = azrtti_cast<JoltCharacter*>(body))
        {
            // The character's kinematic inner body is what sensors and dynamic bodies
            // interact with; map it to the character's handle so events resolve.
            if (const JPH::BodyID innerBodyId = character->GetInnerBodyId(); !innerBodyId.IsInvalid())
            {
                m_bodyHandleByJoltId[innerBodyId.GetIndexAndSequenceNumber()] = handle;
            }
        }

        return handle;
    }

    AzPhysics::SimulatedBodyHandle JoltScene::GetBodyHandleFromJoltId(JPH::BodyID bodyId) const
    {
        if (auto found = m_bodyHandleByJoltId.find(bodyId.GetIndexAndSequenceNumber());
            found != m_bodyHandleByJoltId.end())
        {
            return found->second;
        }
        return AzPhysics::InvalidSimulatedBodyHandle;
    }

    JPH::Body* JoltScene::GetJoltBody(AzPhysics::SimulatedBodyHandle bodyHandle)
    {
        if (!m_physicsSystem)
        {
            return nullptr;
        }

        JPH::BodyID bodyId;
        if (auto* rigidBody = azdynamic_cast<JoltRigidBody*>(GetSimulatedBodyFromHandle(bodyHandle)))
        {
            bodyId = rigidBody->GetBodyId();
        }
        else if (auto* staticBody = azdynamic_cast<JoltStaticRigidBody*>(GetSimulatedBodyFromHandle(bodyHandle)))
        {
            bodyId = staticBody->GetBodyId();
        }
        else
        {
            return nullptr;
        }

        return m_physicsSystem->GetBodyLockInterfaceNoLock().TryGetBody(bodyId);
    }

    bool JoltScene::GetMaterialForSubShape(
        const JPH::Body& body, const JPH::SubShapeID& subShapeId, float& outFriction, float& outRestitution)
    {
        const AzPhysics::SimulatedBodyHandle bodyHandle = GetBodyHandleFromJoltId(body.GetID());
        if (bodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
        {
            return false;
        }

        const AZStd::vector<AZStd::pair<float, float>>* colliderMaterials = nullptr;
        if (auto* rigidBody = azrtti_cast<JoltRigidBody*>(GetSimulatedBodyFromHandle(bodyHandle)))
        {
            colliderMaterials = &rigidBody->GetColliderMaterials();
        }
        else if (auto* staticBody = azrtti_cast<JoltStaticRigidBody*>(GetSimulatedBodyFromHandle(bodyHandle)))
        {
            colliderMaterials = &staticBody->GetColliderMaterials();
        }

        // Only compounds (more than one collider) and heightfields need per-sub-shape overrides.
        if (!colliderMaterials || colliderMaterials->size() <= 1)
        {
            return false;
        }

        const JPH::Shape* shape = body.GetShape();

        // Heightfield bodies: per-triangle material from the provider data.
        if (const JPH::HeightFieldShape* heightFieldShape = JoltHeightfieldUtils::UnwrapHeightField(shape))
        {
            auto* staticBody = azrtti_cast<JoltStaticRigidBody*>(GetSimulatedBodyFromHandle(bodyHandle));
            if (!staticBody)
            {
                return false;
            }

            const auto& materialIndices = staticBody->GetHeightfieldMaterialIndices();
            const auto& materials = staticBody->GetColliderMaterials();
            if (materials.empty())
            {
                return false;
            }

            JPH::SubShapeID remainder;
            const AZ::u32 triangleId = subShapeId.PopID(heightFieldShape->GetSubShapeIDBitsRecursive(), remainder);
            const AZ::u32 square = triangleId >> 1;
            const AZ::u32 sampleCount = heightFieldShape->GetSampleCount();
            const AZ::u32 squareX = square % sampleCount;
            const AZ::u32 squareY = square / sampleCount;

            const size_t indexPosition = squareY * (sampleCount - 1) + squareX;
            const AZ::u8 materialIndex =
                indexPosition < materialIndices.size() ? materialIndices[indexPosition] : 0;
            const AZ::u8 clampedIndex = materialIndex < materials.size() ? materialIndex : 0;

            outFriction = materials[clampedIndex].first;
            outRestitution = materials[clampedIndex].second;
            return true;
        }

        if (!shape || shape->GetSubType() != JPH::EShapeSubType::StaticCompound)
        {
            return false;
        }

        JPH::SubShapeID remainder;
        const auto* compoundShape = static_cast<const JPH::CompoundShape*>(shape);
        const AZ::u32 subShapeIndex = compoundShape->GetSubShapeIndexFromID(subShapeId, remainder);
        if (subShapeIndex >= colliderMaterials->size())
        {
            return false;
        }

        outFriction = (*colliderMaterials)[subShapeIndex].first;
        outRestitution = (*colliderMaterials)[subShapeIndex].second;
        return true;
    }

    AzPhysics::SimulatedBodyHandleList JoltScene::AddSimulatedBodies(
        const AzPhysics::SimulatedBodyConfigurationList& simulatedBodyConfigs)
    {
        AzPhysics::SimulatedBodyHandleList handles;
        handles.reserve(simulatedBodyConfigs.size());

        for (const auto* config : simulatedBodyConfigs)
        {
            handles.push_back(AddSimulatedBody(config));
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
            AzPhysics::SimulatedBody* body = m_simulatedBodies[index].second;
            if (auto* rigidBody = azrtti_cast<JoltRigidBody*>(body))
            {
                const AZ::u32 joltIdKey = rigidBody->GetBodyId().GetIndexAndSequenceNumber();
                m_bodyHandleByJoltId.erase(joltIdKey);
                m_sensorBodyIds.erase(joltIdKey);
                rigidBody->RemoveFromJoltWorld();
            }
            else if (auto* staticBody = azrtti_cast<JoltStaticRigidBody*>(body))
            {
                const AZ::u32 joltIdKey = staticBody->GetBodyId().GetIndexAndSequenceNumber();
                m_bodyHandleByJoltId.erase(joltIdKey);
                m_sensorBodyIds.erase(joltIdKey);
                staticBody->RemoveFromJoltWorld();
            }
            else if (auto* character = azrtti_cast<JoltCharacter*>(body))
            {
                // CharacterVirtual's destructor removes its inner body from the physics system.
                if (const JPH::BodyID innerBodyId = character->GetInnerBodyId(); !innerBodyId.IsInvalid())
                {
                    m_bodyHandleByJoltId.erase(innerBodyId.GetIndexAndSequenceNumber());
                }
            }
            m_deferredDeletions.push_back(body);
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
        const AzPhysics::JointConfiguration* jointConfig,
        AzPhysics::SimulatedBodyHandle parentBody,
        AzPhysics::SimulatedBodyHandle childBody)
    {
        if (!jointConfig || !m_physicsSystem)
        {
            return AzPhysics::InvalidJointHandle;
        }

        JPH::Body* parentJoltBody = GetJoltBody(parentBody);
        JPH::Body* childJoltBody = GetJoltBody(childBody);
        if (!parentJoltBody || !childJoltBody)
        {
            return AzPhysics::InvalidJointHandle;
        }

        JPH::Constraint* constraint = CreateJoltConstraint(*jointConfig, *parentJoltBody, *childJoltBody);
        if (!constraint)
        {
            return AzPhysics::InvalidJointHandle;
        }
        m_physicsSystem->AddConstraint(constraint);

        AzPhysics::JointIndex jointIndex;
        if (!m_freeJointSlots.empty())
        {
            jointIndex = m_freeJointSlots.front();
            m_freeJointSlots.pop();
        }
        else
        {
            jointIndex = static_cast<AzPhysics::JointIndex>(m_joints.size());
            m_joints.emplace_back(AZ::Crc32(), nullptr);
        }

        auto* joint = aznew JoltJoint(this, parentBody, childBody, constraint);

        AZ::Crc32 jointCrc(jointConfig->m_debugName.c_str());
        AzPhysics::JointHandle handle(jointCrc, jointIndex);

        m_joints[jointIndex] = { jointCrc, joint };
        joint->m_sceneOwner = m_sceneHandle;
        joint->m_jointHandle = handle;

        return handle;
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
            if (auto* joint = azrtti_cast<JoltJoint*>(m_joints[index].second))
            {
                joint->RemoveFromJoltWorld();
            }
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

        return JoltSceneQueryHelpers::QueryScene(this, request, result);
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
        AzPhysics::TriggerEventList triggerEvents;
        {
            AZStd::lock_guard lock(m_triggerEventMutex);
            triggerEvents.swap(m_queuedTriggerEvents);
        }

        for (const AzPhysics::TriggerEvent& triggerEvent : triggerEvents)
        {
            if (triggerEvent.m_triggerBody)
            {
                triggerEvent.m_triggerBody->ProcessTriggerEvent(triggerEvent);
            }
        }
    }

    void JoltScene::QueueTriggerEvent(AzPhysics::TriggerEvent::Type type, JPH::BodyID triggerBodyId, JPH::BodyID otherBodyId)
    {
        AzPhysics::SimulatedBodyHandle triggerHandle = GetBodyHandleFromJoltId(triggerBodyId);
        AzPhysics::SimulatedBodyHandle otherHandle = GetBodyHandleFromJoltId(otherBodyId);
        if (triggerHandle == AzPhysics::InvalidSimulatedBodyHandle)
        {
            return;
        }

        AzPhysics::TriggerEvent triggerEvent;
        triggerEvent.m_type = type;
        triggerEvent.m_triggerBodyHandle = triggerHandle;
        triggerEvent.m_triggerBody = GetSimulatedBodyFromHandle(triggerHandle);
        triggerEvent.m_otherBodyHandle = otherHandle;
        triggerEvent.m_otherBody = GetSimulatedBodyFromHandle(otherHandle);

        AZStd::lock_guard lock(m_triggerEventMutex);
        m_queuedTriggerEvents.push_back(triggerEvent);
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
        const JPH::Body& inBody1,
        const JPH::Body& inBody2,
        const JPH::ContactManifold& inManifold,
        JPH::ContactSettings& ioSettings)
    {
        // TODO: Queue collision begin event

        ApplySubShapeMaterials(inBody1, inBody2, inManifold, ioSettings);

        const bool sensor1 = inBody1.IsSensor();
        const bool sensor2 = inBody2.IsSensor();

        if (sensor1)
        {
            m_scene->QueueTriggerEvent(AzPhysics::TriggerEvent::Type::Enter, inBody1.GetID(), inBody2.GetID());
        }
        if (sensor2)
        {
            m_scene->QueueTriggerEvent(AzPhysics::TriggerEvent::Type::Enter, inBody2.GetID(), inBody1.GetID());
        }
    }

    void JoltContactListener::OnContactPersisted(
        const JPH::Body& inBody1,
        const JPH::Body& inBody2,
        const JPH::ContactManifold& inManifold,
        JPH::ContactSettings& ioSettings)
    {
        // TODO: Queue collision persist event

        ApplySubShapeMaterials(inBody1, inBody2, inManifold, ioSettings);
    }

    void JoltContactListener::ApplySubShapeMaterials(
        const JPH::Body& inBody1,
        const JPH::Body& inBody2,
        const JPH::ContactManifold& inManifold,
        JPH::ContactSettings& ioSettings)
    {
        float friction1 = inBody1.GetFriction();
        float friction2 = inBody2.GetFriction();
        float restitution1 = inBody1.GetRestitution();
        float restitution2 = inBody2.GetRestitution();

        m_scene->GetMaterialForSubShape(inBody1, inManifold.mSubShapeID1, friction1, restitution1);
        m_scene->GetMaterialForSubShape(inBody2, inManifold.mSubShapeID2, friction2, restitution2);

        // Same combine rules Jolt applies by default (geometric mean friction, max restitution).
        ioSettings.mCombinedFriction = AZStd::sqrt(friction1 * friction2);
        ioSettings.mCombinedRestitution = AZStd::max(restitution1, restitution2);
    }

    void JoltContactListener::OnContactRemoved(const JPH::SubShapeIDPair& inSubShapePair)
    {
        // TODO: Queue collision end event

        const JPH::BodyID bodyId1 = inSubShapePair.GetBody1ID();
        const JPH::BodyID bodyId2 = inSubShapePair.GetBody2ID();

        if (m_scene->IsSensorBody(bodyId1))
        {
            m_scene->QueueTriggerEvent(AzPhysics::TriggerEvent::Type::Exit, bodyId1, bodyId2);
        }
        if (m_scene->IsSensorBody(bodyId2))
        {
            m_scene->QueueTriggerEvent(AzPhysics::TriggerEvent::Type::Exit, bodyId2, bodyId1);
        }
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
