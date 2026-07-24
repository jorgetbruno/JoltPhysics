#include <Scene/JoltScene.h>
#include <Scene/JoltSceneQueryHelpers.h>
#include <Character/JoltCharacter.h>
#include <Character/JoltRagdoll.h>
#include <Joint/JoltJoint.h>
#include <Material/JoltMaterialManager.h>
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

        // Rigid-body characters move during the physics step; refresh their ground state
        // now that the step has completed (virtual characters do this inside Move()).
        for (auto& [crc, body] : m_simulatedBodies)
        {
            if (auto* character = azdynamic_cast<JoltCharacter*>(body); character && character->IsRigidBodyCharacter())
            {
                character->PostSimulation();
            }
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
        else if (const auto* ragdollConfig = azdynamic_cast<const Physics::RagdollConfiguration*>(simulatedBodyConfig))
        {
            auto* ragdoll = aznew JoltRagdoll(*ragdollConfig);
            ragdoll->CreateInScene(this);
            body = ragdoll;
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
        // Bodies enter the world simulating, except ragdolls, which are built outside
        // the world and only join it on Ragdoll::EnableSimulation.
        body->m_simulating = azrtti_cast<JoltRagdoll*>(body) == nullptr;

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

        // Materials are read live from the Physics::Material objects (rather than values
        // baked at body creation) so runtime material changes - SetProperty on a material,
        // SetMaterial on a prebuilt shape - apply to contacts of existing bodies.
        JoltRigidBody* rigidBody = azrtti_cast<JoltRigidBody*>(GetSimulatedBodyFromHandle(bodyHandle));
        JoltStaticRigidBody* staticBody =
            rigidBody ? nullptr : azrtti_cast<JoltStaticRigidBody*>(GetSimulatedBodyFromHandle(bodyHandle));

        const size_t colliderCount =
            rigidBody ? rigidBody->GetColliderCount() : (staticBody ? staticBody->GetColliderCount() : 0);
        if (colliderCount == 0)
        {
            return false;
        }

        auto getColliderMaterial = [rigidBody, staticBody](size_t index)
        {
            return rigidBody ? rigidBody->GetColliderMaterial(index) : staticBody->GetColliderMaterial(index);
        };

        const JPH::Shape* shape = body.GetShape();

        // Heightfield bodies: per-triangle material from the provider data.
        if (const JPH::HeightFieldShape* heightFieldShape = JoltHeightfieldUtils::UnwrapHeightField(shape))
        {
            if (!staticBody)
            {
                return false;
            }

            const auto& materialIndices = staticBody->GetHeightfieldMaterialIndices();

            JPH::SubShapeID remainder;
            const AZ::u32 triangleId = subShapeId.PopID(heightFieldShape->GetSubShapeIDBitsRecursive(), remainder);
            const AZ::u32 square = triangleId >> 1;
            const AZ::u32 sampleCount = heightFieldShape->GetSampleCount();
            const AZ::u32 squareX = square % sampleCount;
            const AZ::u32 squareY = square / sampleCount;

            const size_t indexPosition = squareY * (sampleCount - 1) + squareX;
            const AZ::u8 materialIndex =
                indexPosition < materialIndices.size() ? materialIndices[indexPosition] : 0;
            const AZ::u8 clampedIndex = materialIndex < colliderCount ? materialIndex : 0;

            const auto heightfieldValues =
                JoltMaterialManager::GetFrictionRestitution(getColliderMaterial(clampedIndex).get());
            outFriction = heightfieldValues.first;
            outRestitution = heightfieldValues.second;
            return true;
        }

        // Compound bodies: material of the collider the touching sub-shape belongs to.
        if (shape && shape->GetSubType() == JPH::EShapeSubType::StaticCompound)
        {
            JPH::SubShapeID remainder;
            const auto* compoundShape = static_cast<const JPH::CompoundShape*>(shape);
            const AZ::u32 subShapeIndex = compoundShape->GetSubShapeIndexFromID(subShapeId, remainder);
            if (subShapeIndex >= colliderCount)
            {
                return false;
            }

            const auto subShapeValues =
                JoltMaterialManager::GetFrictionRestitution(getColliderMaterial(subShapeIndex).get());
            outFriction = subShapeValues.first;
            outRestitution = subShapeValues.second;
            return true;
        }

        // Single-collider bodies: the first (only) collider's material.
        const auto singleValues = JoltMaterialManager::GetFrictionRestitution(getColliderMaterial(0).get());
        outFriction = singleValues.first;
        outRestitution = singleValues.second;
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
                FlushCollisionEndsForRemovedBody(rigidBody->GetBodyId(), bodyHandle);
                FlushTriggerExitsForRemovedBody(rigidBody->GetBodyId());
                const AZ::u32 joltIdKey = rigidBody->GetBodyId().GetIndexAndSequenceNumber();
                m_bodyHandleByJoltId.erase(joltIdKey);
                m_sensorBodyIds.erase(joltIdKey);
                rigidBody->RemoveFromJoltWorld();
            }
            else if (auto* staticBody = azrtti_cast<JoltStaticRigidBody*>(body))
            {
                FlushCollisionEndsForRemovedBody(staticBody->GetBodyId(), bodyHandle);
                FlushTriggerExitsForRemovedBody(staticBody->GetBodyId());
                const AZ::u32 joltIdKey = staticBody->GetBodyId().GetIndexAndSequenceNumber();
                m_bodyHandleByJoltId.erase(joltIdKey);
                m_sensorBodyIds.erase(joltIdKey);
                staticBody->RemoveFromJoltWorld();
            }
            else if (auto* character = azrtti_cast<JoltCharacter*>(body))
            {
                // CharacterVirtual's destructor removes its inner body; the rigid-body
                // character must be removed from the physics system explicitly.
                if (const JPH::BodyID innerBodyId = character->GetInnerBodyId(); !innerBodyId.IsInvalid())
                {
                    m_bodyHandleByJoltId.erase(innerBodyId.GetIndexAndSequenceNumber());
                }
                character->RemoveFromScene();
            }
            else if (auto* ragdoll = azrtti_cast<JoltRagdoll*>(body))
            {
                ragdoll->RemoveFromScene();
            }
            RemoveCollisionSuppressionsForBody(bodyHandle);
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

    void JoltScene::EnableSimulationOfBodyInternal(AzPhysics::SimulatedBody& body)
    {
        if (body.m_simulating)
        {
            return;
        }

        if (auto* rigidBody = azrtti_cast<JoltRigidBody*>(&body))
        {
            rigidBody->SetSimulationEnabled(true);
        }
        else if (auto* staticBody = azrtti_cast<JoltStaticRigidBody*>(&body))
        {
            staticBody->SetSimulationEnabled(true);
        }
        else if (auto* ragdoll = azrtti_cast<JoltRagdoll*>(&body))
        {
            // Re-enable from wherever the ragdoll currently is.
            Physics::RagdollState currentState;
            ragdoll->GetState(currentState);
            ragdoll->EnableSimulation(currentState);
        }
        else
        {
            AZ_WarningOnce("JoltPhysics", false,
                "EnableSimulationOfBody: enabling is not supported for this body type (e.g. characters).");
            return;
        }

        body.m_simulating = true;
        m_simulatedBodySimulationEnabledEvent.Signal(m_sceneHandle, body.m_bodyHandle);
    }

    void JoltScene::DisableSimulationOfBodyInternal(AzPhysics::SimulatedBody& body)
    {
        if (!body.m_simulating)
        {
            return;
        }

        if (auto* rigidBody = azrtti_cast<JoltRigidBody*>(&body))
        {
            rigidBody->SetSimulationEnabled(false);
        }
        else if (auto* staticBody = azrtti_cast<JoltStaticRigidBody*>(&body))
        {
            staticBody->SetSimulationEnabled(false);
        }
        else if (auto* ragdoll = azrtti_cast<JoltRagdoll*>(&body))
        {
            ragdoll->DisableSimulation();
        }
        else
        {
            AZ_WarningOnce("JoltPhysics", false,
                "DisableSimulationOfBody: disabling is not supported for this body type (e.g. characters).");
            return;
        }

        body.m_simulating = false;
        m_simulatedBodySimulationDisabledEvent.Signal(m_sceneHandle, body.m_bodyHandle);
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
        const AzPhysics::SimulatedBodyHandle& bodyHandleA, const AzPhysics::SimulatedBodyHandle& bodyHandleB)
    {
        if (bodyHandleA == AzPhysics::InvalidSimulatedBodyHandle || bodyHandleB == AzPhysics::InvalidSimulatedBodyHandle)
        {
            return;
        }
        m_suppressedCollisionPairs.insert(MakeBodyHandlePairKey(bodyHandleA, bodyHandleB));
    }

    void JoltScene::UnsuppressCollisionEvents(
        const AzPhysics::SimulatedBodyHandle& bodyHandleA, const AzPhysics::SimulatedBodyHandle& bodyHandleB)
    {
        if (bodyHandleA == AzPhysics::InvalidSimulatedBodyHandle || bodyHandleB == AzPhysics::InvalidSimulatedBodyHandle)
        {
            return;
        }
        m_suppressedCollisionPairs.erase(MakeBodyHandlePairKey(bodyHandleA, bodyHandleB));
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

        for (AzPhysics::TriggerEvent& triggerEvent : triggerEvents)
        {
            // Re-resolve the bodies on the main thread: an event may have been queued for a
            // body that has since been removed (its slot is now null), so this avoids
            // dispatching through a dangling pointer.
            triggerEvent.m_triggerBody = GetSimulatedBodyFromHandle(triggerEvent.m_triggerBodyHandle);
            triggerEvent.m_otherBody = GetSimulatedBodyFromHandle(triggerEvent.m_otherBodyHandle);
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
        AzPhysics::CollisionEventList collisionEvents;
        {
            AZStd::lock_guard lock(m_collisionEventMutex);
            collisionEvents.swap(m_queuedCollisionEvents);
        }

        for (AzPhysics::CollisionEvent& collisionEvent : collisionEvents)
        {
            // Pairs registered with SuppressCollisionEvents still collide; only their
            // events are dropped. Filtering here (rather than in the contact callbacks)
            // keeps the suppression set main-thread only.
            if (!m_suppressedCollisionPairs.empty() &&
                m_suppressedCollisionPairs.contains(
                    MakeBodyHandlePairKey(collisionEvent.m_bodyHandle1, collisionEvent.m_bodyHandle2)))
            {
                continue;
            }

            // Body pointers are resolved here on the main thread; a body queued during
            // the step may have been removed by the time we dispatch, so re-resolve and
            // skip anything that has gone away.
            collisionEvent.m_body1 = GetSimulatedBodyFromHandle(collisionEvent.m_bodyHandle1);
            collisionEvent.m_body2 = GetSimulatedBodyFromHandle(collisionEvent.m_bodyHandle2);

            // Each body sees itself as body1 (matches the PhysX backend). Dispatch to
            // body1 as-is, then swap the perspective (bodies, handles, shapes and the
            // contact normals) and dispatch to body2.
            if (collisionEvent.m_body1)
            {
                collisionEvent.m_body1->ProcessCollisionEvent(collisionEvent);
            }

            if (collisionEvent.m_body2)
            {
                AZStd::swap(collisionEvent.m_body1, collisionEvent.m_body2);
                AZStd::swap(collisionEvent.m_bodyHandle1, collisionEvent.m_bodyHandle2);
                AZStd::swap(collisionEvent.m_shape1, collisionEvent.m_shape2);
                for (AzPhysics::Contact& contact : collisionEvent.m_contacts)
                {
                    contact.m_normal = -contact.m_normal;
                }
                collisionEvent.m_body1->ProcessCollisionEvent(collisionEvent);
            }
        }
    }

    void JoltScene::QueueCollisionEvent(AzPhysics::CollisionEvent::Type type,
        JPH::BodyID body1Id, JPH::BodyID body2Id, const JPH::ContactManifold& manifold)
    {
        const AzPhysics::SimulatedBodyHandle handle1 = GetBodyHandleFromJoltId(body1Id);
        const AzPhysics::SimulatedBodyHandle handle2 = GetBodyHandleFromJoltId(body2Id);
        if (handle1 == AzPhysics::InvalidSimulatedBodyHandle || handle2 == AzPhysics::InvalidSimulatedBodyHandle)
        {
            return;
        }

        AzPhysics::CollisionEvent collisionEvent;
        collisionEvent.m_type = type;
        collisionEvent.m_bodyHandle1 = handle1;
        collisionEvent.m_bodyHandle2 = handle2;

        // Build the contact list from the manifold. The normal points from shape 1
        // towards shape 2 (Jolt: "direction to move body 2 out of collision"); separation
        // is the negative of the penetration depth. Jolt does not expose the solved
        // impulse in this callback, so m_impulse is left zero.
        const JPH::Vec3 normal = manifold.mWorldSpaceNormal;
        const AZ::u32 pointCount = static_cast<AZ::u32>(manifold.mRelativeContactPointsOn1.size());
        collisionEvent.m_contacts.reserve(pointCount);
        for (AZ::u32 i = 0; i < pointCount; ++i)
        {
            AzPhysics::Contact contact;
            // Midpoint of the two surface points is a stable contact position whether or
            // not the shapes interpenetrate. Convert each world-space point through
            // Conversions::FromJolt so this stays correct under Jolt's double-precision
            // build (the contact points come back as RVec3).
            const AZ::Vector3 pointOn1 = Conversions::FromJolt(manifold.GetWorldSpaceContactPointOn1(i));
            const AZ::Vector3 pointOn2 = Conversions::FromJolt(manifold.GetWorldSpaceContactPointOn2(i));
            contact.m_position = 0.5f * (pointOn1 + pointOn2);
            contact.m_normal = Conversions::FromJolt(normal);
            contact.m_impulse = AZ::Vector3::CreateZero();
            contact.m_separation = -manifold.mPenetrationDepth;
            collisionEvent.m_contacts.push_back(contact);
        }

        AZStd::lock_guard lock(m_collisionEventMutex);
        m_queuedCollisionEvents.push_back(AZStd::move(collisionEvent));
    }

    void JoltScene::QueueCollisionEndEvent(JPH::BodyID body1Id, JPH::BodyID body2Id)
    {
        EnqueueCollisionEndEvent(GetBodyHandleFromJoltId(body1Id), GetBodyHandleFromJoltId(body2Id));
    }

    void JoltScene::EnqueueCollisionEndEvent(
        AzPhysics::SimulatedBodyHandle handle1, AzPhysics::SimulatedBodyHandle handle2)
    {
        if (handle1 == AzPhysics::InvalidSimulatedBodyHandle || handle2 == AzPhysics::InvalidSimulatedBodyHandle)
        {
            return;
        }

        AzPhysics::CollisionEvent collisionEvent;
        collisionEvent.m_type = AzPhysics::CollisionEvent::Type::End;
        collisionEvent.m_bodyHandle1 = handle1;
        collisionEvent.m_bodyHandle2 = handle2;

        AZStd::lock_guard lock(m_collisionEventMutex);
        m_queuedCollisionEvents.push_back(AZStd::move(collisionEvent));
    }

    AZ::u64 JoltScene::MakeContactPairKey(JPH::BodyID bodyId1, JPH::BodyID bodyId2)
    {
        const AZ::u32 key1 = bodyId1.GetIndexAndSequenceNumber();
        const AZ::u32 key2 = bodyId2.GetIndexAndSequenceNumber();
        const AZ::u32 low = AZStd::min(key1, key2);
        const AZ::u32 high = AZStd::max(key1, key2);
        return (static_cast<AZ::u64>(low) << 32) | high;
    }

    AZ::u64 JoltScene::MakeBodyHandlePairKey(
        const AzPhysics::SimulatedBodyHandle& bodyHandleA, const AzPhysics::SimulatedBodyHandle& bodyHandleB)
    {
        const auto indexA = static_cast<AZ::u32>(AZStd::get<AzPhysics::SimulatedBodyIndex>(bodyHandleA));
        const auto indexB = static_cast<AZ::u32>(AZStd::get<AzPhysics::SimulatedBodyIndex>(bodyHandleB));
        const AZ::u32 low = AZStd::min(indexA, indexB);
        const AZ::u32 high = AZStd::max(indexA, indexB);
        return (static_cast<AZ::u64>(low) << 32) | high;
    }

    void JoltScene::RemoveCollisionSuppressionsForBody(const AzPhysics::SimulatedBodyHandle& bodyHandle)
    {
        if (m_suppressedCollisionPairs.empty())
        {
            return;
        }

        const auto removedIndex = static_cast<AZ::u32>(AZStd::get<AzPhysics::SimulatedBodyIndex>(bodyHandle));
        for (auto it = m_suppressedCollisionPairs.begin(); it != m_suppressedCollisionPairs.end();)
        {
            const AZ::u32 keyLow = static_cast<AZ::u32>(*it >> 32);
            const AZ::u32 keyHigh = static_cast<AZ::u32>(*it & 0xFFFFFFFF);
            it = (keyLow == removedIndex || keyHigh == removedIndex) ? m_suppressedCollisionPairs.erase(it)
                                                                    : AZStd::next(it);
        }
    }

    bool JoltScene::TrackContactAdded(JPH::BodyID bodyId1, JPH::BodyID bodyId2)
    {
        AZStd::lock_guard lock(m_activeContactsMutex);
        int& count = m_activeContacts[MakeContactPairKey(bodyId1, bodyId2)];
        return (count++ == 0); // true only on the first sub-shape contact of the pair
    }

    bool JoltScene::TrackContactRemoved(JPH::BodyID bodyId1, JPH::BodyID bodyId2)
    {
        AZStd::lock_guard lock(m_activeContactsMutex);
        auto it = m_activeContacts.find(MakeContactPairKey(bodyId1, bodyId2));
        if (it == m_activeContacts.end())
        {
            // Already cleared (e.g. one of the bodies was removed): no End to raise.
            return false;
        }
        if (--it->second <= 0)
        {
            m_activeContacts.erase(it);
            return true; // last sub-shape contact removed -> the bodies fully separated
        }
        return false;
    }

    void JoltScene::FlushCollisionEndsForRemovedBody(JPH::BodyID removedBodyId, AzPhysics::SimulatedBodyHandle removedHandle)
    {
        const AZ::u32 removedKey = removedBodyId.GetIndexAndSequenceNumber();

        AZStd::vector<AzPhysics::SimulatedBodyHandle> partnerHandles;
        {
            AZStd::lock_guard lock(m_activeContactsMutex);
            for (auto it = m_activeContacts.begin(); it != m_activeContacts.end();)
            {
                const AZ::u32 keyLow = static_cast<AZ::u32>(it->first >> 32);
                const AZ::u32 keyHigh = static_cast<AZ::u32>(it->first & 0xFFFFFFFF);
                if (keyLow == removedKey || keyHigh == removedKey)
                {
                    const AZ::u32 partnerKey = (keyLow == removedKey) ? keyHigh : keyLow;
                    if (auto handleIt = m_bodyHandleByJoltId.find(partnerKey); handleIt != m_bodyHandleByJoltId.end())
                    {
                        partnerHandles.push_back(handleIt->second);
                    }
                    it = m_activeContacts.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }

        // Jolt only reports OnContactRemoved for these pairs on a later step, by which
        // point the removed body's id->handle mapping is gone; synthesize the End now so
        // the surviving partner still hears that the contact ended.
        for (const AzPhysics::SimulatedBodyHandle partnerHandle : partnerHandles)
        {
            EnqueueCollisionEndEvent(partnerHandle, removedHandle);
        }
    }

    bool JoltScene::TrackTriggerOverlapAdded(JPH::BodyID sensorId, JPH::BodyID otherId)
    {
        // Directed key: only the sensor is notified, so (sensor, other) and (other, sensor)
        // are distinct overlaps.
        const AZ::u64 key = (static_cast<AZ::u64>(sensorId.GetIndexAndSequenceNumber()) << 32)
            | otherId.GetIndexAndSequenceNumber();
        AZStd::lock_guard lock(m_triggerOverlapsMutex);
        int& count = m_activeTriggerOverlaps[key];
        return (count++ == 0); // true only on the first overlapping sub-shape of the pair
    }

    bool JoltScene::TrackTriggerOverlapRemoved(JPH::BodyID sensorId, JPH::BodyID otherId)
    {
        const AZ::u64 key = (static_cast<AZ::u64>(sensorId.GetIndexAndSequenceNumber()) << 32)
            | otherId.GetIndexAndSequenceNumber();
        AZStd::lock_guard lock(m_triggerOverlapsMutex);
        auto it = m_activeTriggerOverlaps.find(key);
        if (it == m_activeTriggerOverlaps.end())
        {
            return false; // already cleared (e.g. a body was removed while overlapping)
        }
        if (--it->second <= 0)
        {
            m_activeTriggerOverlaps.erase(it);
            return true;
        }
        return false;
    }

    void JoltScene::FlushTriggerExitsForRemovedBody(JPH::BodyID removedBodyId)
    {
        const AZ::u32 removedKey = removedBodyId.GetIndexAndSequenceNumber();

        AZStd::vector<JPH::BodyID> sensorsToExit; // sensors that must be told the removed body left
        {
            AZStd::lock_guard lock(m_triggerOverlapsMutex);
            for (auto it = m_activeTriggerOverlaps.begin(); it != m_activeTriggerOverlaps.end();)
            {
                const AZ::u32 sensorKey = static_cast<AZ::u32>(it->first >> 32);
                const AZ::u32 otherKey = static_cast<AZ::u32>(it->first & 0xFFFFFFFF);
                if (sensorKey == removedKey || otherKey == removedKey)
                {
                    // Only the sensor is notified of trigger events. If the removed body is
                    // the overlapping "other", synthesize Exit to the surviving sensor; if
                    // the removed body IS the sensor, there is no listener left to notify.
                    if (otherKey == removedKey && sensorKey != removedKey)
                    {
                        sensorsToExit.push_back(JPH::BodyID(sensorKey));
                    }
                    it = m_activeTriggerOverlaps.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }

        // Queued before RemoveSimulatedBody erases the removed body's id->handle mapping,
        // so the Exit still carries the correct "other" body handle.
        for (const JPH::BodyID sensorId : sensorsToExit)
        {
            QueueTriggerEvent(AzPhysics::TriggerEvent::Type::Exit, sensorId, removedBodyId);
        }
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
        ApplySubShapeMaterials(inBody1, inBody2, inManifold, ioSettings);

        const bool sensor1 = inBody1.IsSensor();
        const bool sensor2 = inBody2.IsSensor();

        // Trigger Enter fires once per (sensor, other) pair on the first overlapping
        // sub-shape, matching PhysX. The overlap is tracked so a body removed while inside
        // the sensor can still synthesize the matching Exit (see OnContactRemoved /
        // FlushTriggerExitsForRemovedBody).
        if (sensor1 && m_scene->TrackTriggerOverlapAdded(inBody1.GetID(), inBody2.GetID()))
        {
            m_scene->QueueTriggerEvent(AzPhysics::TriggerEvent::Type::Enter, inBody1.GetID(), inBody2.GetID());
        }
        if (sensor2 && m_scene->TrackTriggerOverlapAdded(inBody2.GetID(), inBody1.GetID()))
        {
            m_scene->QueueTriggerEvent(AzPhysics::TriggerEvent::Type::Enter, inBody2.GetID(), inBody1.GetID());
        }

        // A collision event is raised only when neither body is a trigger; overlaps
        // involving a sensor are reported through the trigger events above. Begin fires
        // once per body pair (on the first touching sub-shape), matching PhysX.
        if (!sensor1 && !sensor2 && m_scene->TrackContactAdded(inBody1.GetID(), inBody2.GetID()))
        {
            m_scene->QueueCollisionEvent(
                AzPhysics::CollisionEvent::Type::Begin, inBody1.GetID(), inBody2.GetID(), inManifold);
        }
    }

    void JoltContactListener::OnContactPersisted(
        const JPH::Body& inBody1,
        const JPH::Body& inBody2,
        const JPH::ContactManifold& inManifold,
        JPH::ContactSettings& ioSettings)
    {
        ApplySubShapeMaterials(inBody1, inBody2, inManifold, ioSettings);

        if (!inBody1.IsSensor() && !inBody2.IsSensor())
        {
            m_scene->QueueCollisionEvent(
                AzPhysics::CollisionEvent::Type::Persist, inBody1.GetID(), inBody2.GetID(), inManifold);
        }
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
        const JPH::BodyID bodyId1 = inSubShapePair.GetBody1ID();
        const JPH::BodyID bodyId2 = inSubShapePair.GetBody2ID();

        const bool sensor1 = m_scene->IsSensorBody(bodyId1);
        const bool sensor2 = m_scene->IsSensorBody(bodyId2);

        // Trigger Exit fires once per (sensor, other) pair when the last overlapping
        // sub-shape separates. If the overlap is already untracked (a body was removed
        // while inside the sensor, handled in FlushTriggerExitsForRemovedBody),
        // TrackTriggerOverlapRemoved returns false and we skip.
        if (sensor1 && m_scene->TrackTriggerOverlapRemoved(bodyId1, bodyId2))
        {
            m_scene->QueueTriggerEvent(AzPhysics::TriggerEvent::Type::Exit, bodyId1, bodyId2);
        }
        if (sensor2 && m_scene->TrackTriggerOverlapRemoved(bodyId2, bodyId1))
        {
            m_scene->QueueTriggerEvent(AzPhysics::TriggerEvent::Type::Exit, bodyId2, bodyId1);
        }

        // End fires once per body pair, when the last touching sub-shape separates. If the
        // pair is already untracked (a body was removed mid-contact, handled below in
        // FlushCollisionEndsForRemovedBody), TrackContactRemoved returns false and we skip.
        if (!sensor1 && !sensor2 && m_scene->TrackContactRemoved(bodyId1, bodyId2))
        {
            m_scene->QueueCollisionEndEvent(bodyId1, bodyId2);
        }
    }

    // Sleep-state notifications need no handling: transform sync polls the awake bodies
    // each FinishSimulation (see FlushTransformSync), and SimulatedBody::m_simulating
    // tracks world membership (Enable/DisableSimulationOfBody), not sleep state. The
    // listener stays registered as the hook for future sleep events.
    void JoltBodyActivationListener::OnBodyActivated(
        [[maybe_unused]] const JPH::BodyID& inBodyID,
        [[maybe_unused]] AZ::u64 inBodyUserData)
    {
    }

    void JoltBodyActivationListener::OnBodyDeactivated(
        [[maybe_unused]] const JPH::BodyID& inBodyID,
        [[maybe_unused]] AZ::u64 inBodyUserData)
    {
    }

} // namespace JoltPhysics
