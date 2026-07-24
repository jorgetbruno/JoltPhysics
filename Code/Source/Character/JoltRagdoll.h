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
        //! Removes the ragdoll's bodies from the physics world (if simulating).
        void RemoveFromScene();

        //! Hard-keys the ragdoll toward the target pose: drives every body kinematically so
        //! it reaches the corresponding node state within deltaTime (the bodies follow the
        //! animation and push dynamic objects). Jolt-specific animation-driving entry point.
        void DriveToPoseUsingKinematics(const Physics::RagdollState& targetPose, float deltaTime);

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
    };
} // namespace JoltPhysics
