#include <Character/JoltRagdoll.h>

#include <AzCore/std/algorithm.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

#include <Scene/JoltScene.h>
#include <Shape/JoltShapeUtils.h>
#include <System/CollisionLayerFilters.h>
#include <Utils/Conversions.h>

#include <Jolt/Jolt.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace JoltPhysics
{
    namespace
    {
        //! Builds a JPH::RagdollSettings (skeleton + one part per node) from the O3DE
        //! ragdoll configuration. Slice 1: bodies are dynamic capsules/shapes from the
        //! collider configuration, connected to their parent by a point constraint.
        JPH::Ref<JPH::RagdollSettings> BuildRagdollSettings(const Physics::RagdollConfiguration& config)
        {
            const size_t numNodes = config.m_nodes.size();
            if (numNodes == 0)
            {
                return nullptr;
            }

            JPH::Ref<JPH::Skeleton> skeleton = new JPH::Skeleton;
            for (size_t i = 0; i < numNodes; ++i)
            {
                int parentIndex = -1;
                if (i < config.m_parentIndices.size())
                {
                    const size_t parent = config.m_parentIndices[i];
                    if (parent < numNodes && parent != i)
                    {
                        parentIndex = static_cast<int>(parent);
                    }
                }
                skeleton->AddJoint(config.m_nodes[i].m_debugName.c_str(), parentIndex);
            }

            JPH::Ref<JPH::RagdollSettings> settings = new JPH::RagdollSettings;
            settings->mSkeleton = skeleton;
            settings->mParts.resize(numNodes);

            for (size_t i = 0; i < numNodes; ++i)
            {
                const Physics::RagdollNodeConfiguration& nodeConfig = config.m_nodes[i];
                JPH::RagdollSettings::Part& part = settings->mParts[i];

                // Shape from the matching collider node (first shape); fall back to a small
                // capsule so a missing collider does not abort creation of the whole ragdoll.
                JPH::RefConst<JPH::Shape> shape;
                if (const Physics::CharacterColliderNodeConfiguration* colliderNode =
                        config.m_colliders.FindNodeConfigByName(nodeConfig.m_debugName);
                    colliderNode != nullptr && !colliderNode->m_shapes.empty() && colliderNode->m_shapes.front().second)
                {
                    shape = JoltShapeUtils::CreateJoltShapeFromConfig(*colliderNode->m_shapes.front().second);
                }
                if (!shape)
                {
                    Physics::CapsuleShapeConfiguration fallback(0.3f, 0.05f);
                    shape = JoltShapeUtils::CreateJoltShapeFromConfig(fallback);
                }
                part.SetShape(shape);

                part.mMotionType = JPH::EMotionType::Dynamic;
                part.mObjectLayer = ObjectLayers::Moving;
                if (i < config.m_initialState.size())
                {
                    part.mPosition = Conversions::ToJoltR(config.m_initialState[i].m_position);
                    part.mRotation = Conversions::ToJolt(config.m_initialState[i].m_orientation);
                }
                if (nodeConfig.m_mass > 0.0f)
                {
                    part.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
                    part.mMassPropertiesOverride.mMass = nodeConfig.m_mass;
                }

                // Connect each non-root node to its parent. Slice 1 uses a point constraint
                // (a ball joint) at the node's world position -> a floppy ragdoll. Slice 2
                // will build the constraint from nodeConfig.m_jointConfig with proper limits.
                if (skeleton->GetJoint(static_cast<int>(i)).mParentJointIndex >= 0)
                {
                    JPH::Ref<JPH::PointConstraintSettings> constraint = new JPH::PointConstraintSettings;
                    constraint->mSpace = JPH::EConstraintSpace::WorldSpace;
                    const JPH::RVec3 pivot = (i < config.m_initialState.size())
                        ? Conversions::ToJoltR(config.m_initialState[i].m_position)
                        : JPH::RVec3::sZero();
                    constraint->mPoint1 = pivot;
                    constraint->mPoint2 = pivot;
                    part.mToParent = constraint;
                }
            }

            settings->Stabilize();
            settings->DisableParentChildCollisions();
            settings->CalculateBodyIndexToConstraintIndex();
            return settings;
        }
    } // namespace

    JoltRagdoll::JoltRagdoll(const Physics::RagdollConfiguration& configuration)
        : m_configuration(configuration)
    {
    }

    JoltRagdoll::~JoltRagdoll()
    {
        RemoveFromScene();
    }

    void JoltRagdoll::CreateInScene(JoltScene* scene)
    {
        m_scene = scene;
        if (!m_scene || !m_scene->GetJoltPhysicsSystem())
        {
            return;
        }

        m_settings = BuildRagdollSettings(m_configuration);
        if (!m_settings)
        {
            return;
        }

        m_ragdoll = m_settings->CreateRagdoll(/*collisionGroup*/ 0, /*userData*/ 0, m_scene->GetJoltPhysicsSystem());
        m_numNodes = m_configuration.m_nodes.size();
    }

    void JoltRagdoll::RemoveFromScene()
    {
        DisableSimulation();
    }

    void JoltRagdoll::EnableSimulation(const Physics::RagdollState& initialState)
    {
        if (!m_ragdoll || m_simulated)
        {
            return;
        }
        m_ragdoll->AddToPhysicsSystem(JPH::EActivation::Activate);
        m_simulated = true;
        if (!initialState.empty())
        {
            SetState(initialState);
        }
    }

    void JoltRagdoll::EnableSimulationQueued(const Physics::RagdollState& initialState)
    {
        EnableSimulation(initialState);
    }

    void JoltRagdoll::DisableSimulation()
    {
        if (!m_ragdoll || !m_simulated)
        {
            return;
        }
        m_ragdoll->RemoveFromPhysicsSystem();
        m_simulated = false;
    }

    void JoltRagdoll::DisableSimulationQueued()
    {
        DisableSimulation();
    }

    bool JoltRagdoll::IsSimulated() const
    {
        return m_simulated;
    }

    void JoltRagdoll::ReadNodeState(size_t nodeIndex, Physics::RagdollNodeState& nodeState) const
    {
        JPH::BodyInterface* bodyInterface = m_scene ? m_scene->GetBodyInterface() : nullptr;
        if (!m_ragdoll || !bodyInterface || nodeIndex >= m_numNodes)
        {
            return;
        }
        const JPH::BodyID bodyId = m_ragdoll->GetBodyID(static_cast<int>(nodeIndex));
        JPH::RVec3 position;
        JPH::Quat rotation;
        bodyInterface->GetPositionAndRotation(bodyId, position, rotation);
        nodeState.m_position = Conversions::FromJolt(position);
        nodeState.m_orientation = Conversions::FromJolt(rotation);
        nodeState.m_linearVelocity = Conversions::FromJolt(bodyInterface->GetLinearVelocity(bodyId));
        nodeState.m_angularVelocity = Conversions::FromJolt(bodyInterface->GetAngularVelocity(bodyId));
    }

    void JoltRagdoll::WriteNodeState(size_t nodeIndex, const Physics::RagdollNodeState& nodeState)
    {
        JPH::BodyInterface* bodyInterface = m_scene ? m_scene->GetBodyInterface() : nullptr;
        if (!m_ragdoll || !bodyInterface || nodeIndex >= m_numNodes)
        {
            return;
        }
        const JPH::BodyID bodyId = m_ragdoll->GetBodyID(static_cast<int>(nodeIndex));
        bodyInterface->SetPositionRotationAndVelocity(
            bodyId,
            Conversions::ToJoltR(nodeState.m_position),
            Conversions::ToJolt(nodeState.m_orientation),
            Conversions::ToJolt(nodeState.m_linearVelocity),
            Conversions::ToJolt(nodeState.m_angularVelocity));
    }

    void JoltRagdoll::GetState(Physics::RagdollState& ragdollState) const
    {
        ragdollState.resize(m_numNodes);
        for (size_t i = 0; i < m_numNodes; ++i)
        {
            ReadNodeState(i, ragdollState[i]);
        }
    }

    void JoltRagdoll::SetState(const Physics::RagdollState& ragdollState)
    {
        const size_t count = AZStd::min(ragdollState.size(), m_numNodes);
        for (size_t i = 0; i < count; ++i)
        {
            WriteNodeState(i, ragdollState[i]);
        }
    }

    void JoltRagdoll::SetStateQueued(const Physics::RagdollState& ragdollState)
    {
        SetState(ragdollState);
    }

    void JoltRagdoll::GetNodeState(size_t nodeIndex, Physics::RagdollNodeState& nodeState) const
    {
        ReadNodeState(nodeIndex, nodeState);
    }

    void JoltRagdoll::SetNodeState(size_t nodeIndex, const Physics::RagdollNodeState& nodeState)
    {
        WriteNodeState(nodeIndex, nodeState);
    }

    Physics::RagdollNode* JoltRagdoll::GetNode([[maybe_unused]] size_t nodeIndex) const
    {
        // Per-node RagdollNode access (with GetRigidBody/GetJoint) is a follow-up slice.
        return nullptr;
    }

    size_t JoltRagdoll::GetNumNodes() const
    {
        return m_numNodes;
    }

    AZ::EntityId JoltRagdoll::GetEntityId() const
    {
        return m_entityId;
    }

    AZ::Transform JoltRagdoll::GetTransform() const
    {
        return AZ::Transform::CreateFromQuaternionAndTranslation(GetOrientation(), GetPosition());
    }

    void JoltRagdoll::SetTransform([[maybe_unused]] const AZ::Transform& transform)
    {
        // The ragdoll is driven through its per-node state; moving it as a whole is not
        // supported here (use SetState with repositioned nodes instead).
        AZ_WarningOnce("JoltPhysics", false, "JoltRagdoll::SetTransform is a no-op; drive the ragdoll via SetState.");
    }

    AZ::Vector3 JoltRagdoll::GetPosition() const
    {
        if (!m_ragdoll || m_numNodes == 0)
        {
            return AZ::Vector3::CreateZero();
        }
        Physics::RagdollNodeState rootState;
        ReadNodeState(0, rootState);
        return rootState.m_position;
    }

    AZ::Quaternion JoltRagdoll::GetOrientation() const
    {
        if (!m_ragdoll || m_numNodes == 0)
        {
            return AZ::Quaternion::CreateIdentity();
        }
        Physics::RagdollNodeState rootState;
        ReadNodeState(0, rootState);
        return rootState.m_orientation;
    }

    AZ::Aabb JoltRagdoll::GetAabb() const
    {
        AZ::Aabb aabb = AZ::Aabb::CreateNull();
        Physics::RagdollNodeState nodeState;
        for (size_t i = 0; i < m_numNodes; ++i)
        {
            ReadNodeState(i, nodeState);
            aabb.AddPoint(nodeState.m_position);
        }
        return aabb;
    }

    AzPhysics::SceneQueryHit JoltRagdoll::RayCast([[maybe_unused]] const AzPhysics::RayCastRequest& request)
    {
        return AzPhysics::SceneQueryHit();
    }

    AZ::Crc32 JoltRagdoll::GetNativeType() const
    {
        return AZ_CRC_CE("JoltRagdoll");
    }

    void* JoltRagdoll::GetNativePointer() const
    {
        return m_ragdoll.GetPtr();
    }

} // namespace JoltPhysics
