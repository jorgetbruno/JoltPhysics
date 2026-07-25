#pragma once

#include <AzFramework/Physics/Ragdoll.h>

#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <AzCore/std/containers/vector.h>

#include <Jolt/Jolt.h>
#include <Jolt/Core/Reference.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>

namespace JoltPhysics
{
    class JoltScene;
    class JoltRagdollNode;

    //! Physics::Ragdoll implementation backed by JPH::Ragdoll: a skeleton of dynamic
    //! bodies connected by constraints, built from a Physics::RagdollConfiguration.
    //!
    //! Slice 1 supports creating the ragdoll, enabling/disabling its simulation, and
    //! reading/writing per-node world state (position/orientation/velocity). Per-node
    //! articulation limits and RagdollNode/GetRigidBody access are follow-up slices; the
    //! nodes are currently connected by point constraints (a floppy ragdoll).
    class JoltRagdoll final : public Physics::Ragdoll
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltRagdoll, AZ::SystemAllocator);
        AZ_RTTI(JoltRagdoll, "{7F2E1A0B-3C4D-5E6F-8A9B-0C1D2E3F4A5B}", Physics::Ragdoll);

        explicit JoltRagdoll(const Physics::RagdollConfiguration& configuration);
        ~JoltRagdoll() override;

        //! Builds the JPH ragdoll (bodies + constraints); does not add it to the world yet.
        void CreateInScene(JoltScene* scene);

        //! The native settings, which carry the ragdoll's skeleton and the neutral pose
        //! of its parts. Used by JoltSkeletonMapper to map against an animation skeleton.
        const JPH::RagdollSettings* GetNativeSettings() const
        {
            return m_settings;
        }
        //! Removes the ragdoll's bodies from the physics world (if simulating).
        void RemoveFromScene();

        //! Hard-keys the ragdoll to the target pose: the bodies are switched to kinematic
        //! and given the velocity that reaches the corresponding node state within
        //! deltaTime. Nothing can push a kinematic body off the animation - it drives
        //! through obstacles and shoves dynamic objects out of the way.
        //! The bodies stay kinematic until another drive mode or ReleaseToPhysics is used.
        void DriveToPoseUsingKinematics(const Physics::RagdollState& targetPose, float deltaTime);

        //! Soft-keys the ragdoll toward the target pose: the bodies stay dynamic and are
        //! given the velocity that would reach the pose, capped so the drive has finite
        //! authority. The ragdoll follows the animation in open space yet is stopped by
        //! walls and pushed by impacts.
        //!
        //! The caps are what make this soft: an uncapped velocity is recomputed from the
        //! full remaining distance every step, which overrides whatever the solver did and
        //! makes the ragdoll drive through obstacles exactly like hard keying.
        void DriveToPoseUsingVelocities(
            const Physics::RagdollState& targetPose,
            float deltaTime,
            float maxLinearSpeed = 10.0f,
            float maxAngularSpeed = 10.0f);

        //! Returns the bodies to dynamic, ending any hard keying so the ragdoll goes limp
        //! and falls under gravity again.
        void ReleaseToPhysics();

        //! Soft-keys the ragdoll toward the target pose ("powered ragdoll"): each joint's
        //! motor is driven towards the target orientation of its node relative to its
        //! parent, so the bodies stay dynamic and blend animation against physics -
        //! they collide, get pushed around, and spring back towards the pose.
        //!
        //! Per node, RagdollNodeState::m_strength is the motor's spring frequency in Hz
        //! (0 disables the motor, leaving that joint purely physical) and
        //! m_dampingRatio is its damping ratio, so the animation/physics blend can be
        //! varied per node and over time. Only the orientation of each node state is
        //! used; motors steer joints, they do not teleport bodies.
        void DriveToPoseUsingMotors(const Physics::RagdollState& targetPose);

        // Physics::Ragdoll
        void EnableSimulation(const Physics::RagdollState& initialState) override;
        void EnableSimulationQueued(const Physics::RagdollState& initialState) override;
        void DisableSimulation() override;
        void DisableSimulationQueued() override;
        bool IsSimulated() const override;
        void GetState(Physics::RagdollState& ragdollState) const override;
        void SetState(const Physics::RagdollState& ragdollState) override;
        void SetStateQueued(const Physics::RagdollState& ragdollState) override;
        void GetNodeState(size_t nodeIndex, Physics::RagdollNodeState& nodeState) const override;
        void SetNodeState(size_t nodeIndex, const Physics::RagdollNodeState& nodeState) override;
        Physics::RagdollNode* GetNode(size_t nodeIndex) const override;
        size_t GetNumNodes() const override;

        // AzPhysics::SimulatedBody
        AZ::EntityId GetEntityId() const override;
        AZ::Transform GetTransform() const override;
        void SetTransform(const AZ::Transform& transform) override;
        AZ::Vector3 GetPosition() const override;
        AZ::Quaternion GetOrientation() const override;
        AZ::Aabb GetAabb() const override;
        AzPhysics::SceneQueryHit RayCast(const AzPhysics::RayCastRequest& request) override;
        AZ::Crc32 GetNativeType() const override;
        void* GetNativePointer() const override;

    private:
        //! Applies the target pose as velocities on the bodies (Jolt's kinematic drive).
        //! Whether that is hard or soft keying is decided by the bodies' motion type.
        void DriveToPose(const Physics::RagdollState& targetPose, float deltaTime);
        //! Switches every ragdoll body to the given motion type; a no-op when unchanged.
        void SetBodiesMotionType(JPH::EMotionType motionType);

        //! Reads one node's world-space state from its Jolt body.
        void ReadNodeState(size_t nodeIndex, Physics::RagdollNodeState& nodeState) const;
        //! Writes one node's world-space state onto its Jolt body.
        void WriteNodeState(size_t nodeIndex, const Physics::RagdollNodeState& nodeState);

        Physics::RagdollConfiguration m_configuration;
        JoltScene* m_scene = nullptr;

        JPH::Ref<JPH::RagdollSettings> m_settings;
        JPH::Ref<JPH::Ragdoll> m_ragdoll;
        AZStd::vector<AZStd::unique_ptr<JoltRagdollNode>> m_nodes;

        AZ::EntityId m_entityId;
        size_t m_numNodes = 0;
        bool m_simulated = false;
        //! Motion type the bodies currently have; they are created dynamic and only become
        //! kinematic while hard keying.
        JPH::EMotionType m_bodyMotionType = JPH::EMotionType::Dynamic;
    };
} // namespace JoltPhysics
