#include <Character/JoltRagdoll.h>
#include <Character/JoltRagdollNode.h>

#include <AzCore/std/algorithm.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

#include <Joint/JoltJointConfiguration.h>
#include <Scene/JoltScene.h>
#include <Shape/JoltShapeUtils.h>
#include <System/CollisionLayerFilters.h>
#include <Utils/Conversions.h>

#include <Jolt/Jolt.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace JoltPhysics
{
    namespace
    {
        //! Builds a swing-twist constraint (the ragdoll joint) to the parent body from the
        //! node's joint configuration. The joint frames come from the config's parent/child
        //! local transforms; the swing/twist limits are read from a recognized Jolt joint
        //! config type, otherwise moderate defaults are used. Returns null when there is no
        //! joint config (the caller then falls back to a floppy point constraint).
        JPH::Ref<JPH::TwoBodyConstraintSettings> BuildRagdollJointSettings(const AzPhysics::JointConfiguration* jointConfig)
        {
            if (!jointConfig)
            {
                return nullptr;
            }

            auto axis = [](const AZ::Quaternion& rotation, const AZ::Vector3& localAxis)
            {
                return Conversions::ToJolt(rotation.TransformVector(localAxis));
            };

            // Returned as a raw pointer so the JPH::Ref<TwoBodyConstraintSettings> return
            // type adopts it via the base class (Ref<Derived> does not upcast implicitly).
            JPH::SwingTwistConstraintSettings* settings = new JPH::SwingTwistConstraintSettings;
            settings->mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
            settings->mPosition1 = Conversions::ToJoltR(jointConfig->m_parentLocalPosition);
            settings->mTwistAxis1 = axis(jointConfig->m_parentLocalRotation, AZ::Vector3::CreateAxisX());
            settings->mPlaneAxis1 = axis(jointConfig->m_parentLocalRotation, AZ::Vector3::CreateAxisY());
            settings->mPosition2 = Conversions::ToJoltR(jointConfig->m_childLocalPosition);
            settings->mTwistAxis2 = axis(jointConfig->m_childLocalRotation, AZ::Vector3::CreateAxisX());
            settings->mPlaneAxis2 = axis(jointConfig->m_childLocalRotation, AZ::Vector3::CreateAxisY());
            settings->mSwingType = JPH::ESwingType::Cone;

            // Default limits (degrees); refined from a recognized Jolt joint config.
            float normalHalfCone = 45.0f;
            float planeHalfCone = 45.0f;
            float twistMin = -45.0f;
            float twistMax = 45.0f;
            if (const auto* swingTwist = azrtti_cast<const JoltSwingTwistJointConfiguration*>(jointConfig))
            {
                normalHalfCone = swingTwist->m_normalHalfConeAngle;
                planeHalfCone = swingTwist->m_planeHalfConeAngle;
                twistMin = swingTwist->m_twistLower;
                twistMax = swingTwist->m_twistUpper;
            }
            else if (const auto* d6 = azrtti_cast<const JoltD6JointLimitConfiguration*>(jointConfig))
            {
                normalHalfCone = d6->m_swingLimitY;
                planeHalfCone = d6->m_swingLimitZ;
                twistMin = d6->m_twistLimitLower;
                twistMax = d6->m_twistLimitUpper;
            }
            else if (const auto* cone = azrtti_cast<const JoltConeJointConfiguration*>(jointConfig))
            {
                normalHalfCone = cone->m_halfConeAngle;
                planeHalfCone = cone->m_halfConeAngle;
            }
            settings->mNormalHalfConeAngle = AZ::DegToRad(normalHalfCone);
            settings->mPlaneHalfConeAngle = AZ::DegToRad(planeHalfCone);
            settings->mTwistMinAngle = AZ::DegToRad(twistMin);
            settings->mTwistMaxAngle = AZ::DegToRad(twistMax);
            return settings;
        }

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

                // Connect each non-root node to its parent with an articulated swing-twist
                // joint built from the node's joint configuration (limits included). When the
                // node has no joint config, fall back to a floppy point constraint at the
                // node's world position so the ragdoll still holds together.
                if (skeleton->GetJoint(static_cast<int>(i)).mParentJointIndex >= 0)
                {
                    JPH::Ref<JPH::TwoBodyConstraintSettings> constraint =
                        BuildRagdollJointSettings(nodeConfig.m_jointConfig.get());
                    if (!constraint)
                    {
                        JPH::Ref<JPH::PointConstraintSettings> point = new JPH::PointConstraintSettings;
                        point->mSpace = JPH::EConstraintSpace::WorldSpace;
                        const JPH::RVec3 pivot = (i < config.m_initialState.size())
                            ? Conversions::ToJoltR(config.m_initialState[i].m_position)
                            : JPH::RVec3::sZero();
                        point->mPoint1 = pivot;
                        point->mPoint2 = pivot;
                        constraint = point;
                    }
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

        // Wrap each ragdoll-owned body as a RagdollNode for per-node access, and wrap the
        // constraint attaching it to its parent (if any) as its joint.
        m_nodes.reserve(m_numNodes);
        for (size_t i = 0; i < m_numNodes; ++i)
        {
            auto node = AZStd::make_unique<JoltRagdollNode>();
            node->Setup(m_scene, m_ragdoll->GetBodyID(static_cast<int>(i)), m_entityId, &m_simulated);

            const int constraintIndex = m_settings->GetConstraintIndexForBodyIndex(static_cast<int>(i));
            if (constraintIndex >= 0)
            {
                if (JPH::TwoBodyConstraint* constraint = m_ragdoll->GetConstraint(constraintIndex))
                {
                    node->SetJoint(AZStd::make_unique<JoltJoint>(
                        m_scene, AzPhysics::InvalidSimulatedBodyHandle, AzPhysics::InvalidSimulatedBodyHandle, constraint));
                }
            }
            m_nodes.push_back(AZStd::move(node));
        }
    }

    void JoltRagdoll::DriveToPoseUsingKinematics(const Physics::RagdollState& targetPose, float deltaTime)
    {
        if (!m_ragdoll || !m_simulated || targetPose.empty() || deltaTime <= 0.0f)
        {
            return;
        }

        // World-space joint matrices are expressed relative to a root offset (kept close to
        // the origin for float precision), matching Jolt's lower-level pose API.
        const JPH::RVec3 rootOffset = Conversions::ToJoltR(targetPose[0].m_position);
        JPH::Array<JPH::Mat44> jointMatrices(m_numNodes); // JPH::Array respects Mat44's SIMD alignment
        for (size_t i = 0; i < m_numNodes; ++i)
        {
            const Physics::RagdollNodeState& nodeState = (i < targetPose.size()) ? targetPose[i] : targetPose.back();
            const JPH::Vec3 relativePosition = JPH::Vec3(Conversions::ToJoltR(nodeState.m_position) - rootOffset);
            jointMatrices[i] = JPH::Mat44::sRotationTranslation(Conversions::ToJolt(nodeState.m_orientation), relativePosition);
        }

        m_ragdoll->DriveToPoseUsingKinematics(rootOffset, jointMatrices.data(), deltaTime);
    }

    void JoltRagdoll::DriveToPoseUsingMotors(const Physics::RagdollState& targetPose)
    {
        if (!m_ragdoll || !m_settings || !m_simulated || targetPose.empty())
        {
            return;
        }

        bool anyMotorDriven = false;
        const size_t count = AZStd::min(targetPose.size(), m_numNodes);
        for (size_t i = 0; i < count; ++i)
        {
            // The root has no joint to a parent, so nothing to drive.
            const int constraintIndex = m_settings->GetConstraintIndexForBodyIndex(static_cast<int>(i));
            if (constraintIndex < 0)
            {
                continue;
            }

            JPH::TwoBodyConstraint* constraint = m_ragdoll->GetConstraint(constraintIndex);
            if (!constraint || constraint->GetSubType() != JPH::EConstraintSubType::SwingTwist)
            {
                // Only the articulated (swing-twist) joints have motors; the point
                // constraint fallback used for jointless nodes cannot be driven.
                continue;
            }
            auto* swingTwist = static_cast<JPH::SwingTwistConstraint*>(constraint);

            const Physics::RagdollNodeState& nodeState = targetPose[i];
            if (nodeState.m_strength <= 0.0f)
            {
                // Strength zero means "leave this joint to physics"; releasing the motor
                // is what makes a per-node animation/physics blend possible.
                swingTwist->SetSwingMotorState(JPH::EMotorState::Off);
                swingTwist->SetTwistMotorState(JPH::EMotorState::Off);
                continue;
            }

            // Motors drive the child's orientation relative to its parent, so the target
            // is the node's world orientation taken into the parent's frame.
            AZ::Quaternion parentOrientation = AZ::Quaternion::CreateIdentity();
            if (i < m_configuration.m_parentIndices.size())
            {
                const size_t parentIndex = m_configuration.m_parentIndices[i];
                if (parentIndex < targetPose.size())
                {
                    parentOrientation = targetPose[parentIndex].m_orientation;
                }
            }
            const AZ::Quaternion localOrientation =
                (parentOrientation.GetConjugate() * nodeState.m_orientation).GetNormalized();

            // m_strength is mapped to the motor's spring frequency (Hz) and m_dampingRatio
            // to its damping, so a stronger joint springs to the pose faster and a lower
            // damping ratio overshoots. See DIVERGENCES.md.
            const JPH::SpringSettings springSettings(
                JPH::ESpringMode::FrequencyAndDamping, nodeState.m_strength, AZStd::max(nodeState.m_dampingRatio, 0.0f));
            swingTwist->GetSwingMotorSettings().mSpringSettings = springSettings;
            swingTwist->GetTwistMotorSettings().mSpringSettings = springSettings;

            swingTwist->SetSwingMotorState(JPH::EMotorState::Position);
            swingTwist->SetTwistMotorState(JPH::EMotorState::Position);
            swingTwist->SetTargetOrientationBS(Conversions::ToJolt(localOrientation));
            anyMotorDriven = true;
        }

        // Motors cannot move bodies that have gone to sleep.
        if (anyMotorDriven)
        {
            m_ragdoll->Activate();
        }
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
        m_simulating = true; // keep the SimulatedBody-level flag in sync
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
        m_simulating = false; // keep the SimulatedBody-level flag in sync
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

    Physics::RagdollNode* JoltRagdoll::GetNode(size_t nodeIndex) const
    {
        return nodeIndex < m_nodes.size() ? m_nodes[nodeIndex].get() : nullptr;
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
