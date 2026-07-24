#pragma once

#include <AzFramework/Physics/Ragdoll.h>

#include <AzCore/std/smart_ptr/unique_ptr.h>

#include <Joint/JoltJoint.h>
#include <RigidBody/JoltRigidBody.h>

#include <Jolt/Physics/Body/BodyID.h>

namespace JoltPhysics
{
    class JoltScene;

    //! Physics::RagdollNode for a single body of a JoltRagdoll. The body itself is owned
    //! by the JPH::Ragdoll; this node wraps it (via JoltRigidBody::AdoptBody) so callers
    //! can access it as an AzPhysics::RigidBody without owning its lifetime.
    class JoltRagdollNode final : public Physics::RagdollNode
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltRagdollNode, AZ::SystemAllocator);
        AZ_RTTI(JoltRagdollNode, "{8A3C1B2D-4E5F-6071-8293-A4B5C6D7E8F0}", Physics::RagdollNode);

        JoltRagdollNode() = default;

        //! Binds the node to a ragdoll-owned Jolt body. simulatedFlag points at the owning
        //! ragdoll's "is simulated" flag so IsSimulating() reflects the ragdoll state.
        void Setup(JoltScene* scene, const JPH::BodyID& bodyId, AZ::EntityId entityId, const bool* simulatedFlag);

        //! Wraps the ragdoll-owned constraint that attaches this node to its parent (null
        //! for the root). The JoltJoint does not own the constraint - the ragdoll does - so
        //! its RemoveFromJoltWorld is never called here.
        void SetJoint(AZStd::unique_ptr<JoltJoint> joint) { m_joint = AZStd::move(joint); }

        // Physics::RagdollNode
        AzPhysics::RigidBody& GetRigidBody() override { return m_rigidBody; }
        AzPhysics::Joint* GetJoint() override { return m_joint.get(); }
        bool IsSimulating() const override { return m_simulatedFlag != nullptr && *m_simulatedFlag; }

        // AzPhysics::SimulatedBody (delegated to the wrapped body)
        AZ::EntityId GetEntityId() const override { return m_rigidBody.GetEntityId(); }
        AZ::Transform GetTransform() const override { return m_rigidBody.GetTransform(); }
        void SetTransform(const AZ::Transform& transform) override { m_rigidBody.SetTransform(transform); }
        AZ::Vector3 GetPosition() const override { return m_rigidBody.GetPosition(); }
        AZ::Quaternion GetOrientation() const override { return m_rigidBody.GetOrientation(); }
        AZ::Aabb GetAabb() const override { return m_rigidBody.GetAabb(); }
        AzPhysics::SceneQueryHit RayCast(const AzPhysics::RayCastRequest& request) override { return m_rigidBody.RayCast(request); }
        AZ::Crc32 GetNativeType() const override { return AZ_CRC_CE("JoltRagdollNode"); }
        void* GetNativePointer() const override { return m_rigidBody.GetNativePointer(); }

    private:
        JoltRigidBody m_rigidBody;
        AZStd::unique_ptr<JoltJoint> m_joint;
        const bool* m_simulatedFlag = nullptr;
    };
} // namespace JoltPhysics
