#pragma once

#include <AzFramework/Physics/Common/PhysicsJoint.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Constraints/Constraint.h>

namespace JoltPhysics
{
    class JoltScene;

    //! AzPhysics::Joint implementation wrapping a JPH::Constraint.
    class JoltJoint : public AzPhysics::Joint
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltJoint, AZ::SystemAllocator);
        AZ_RTTI(JoltJoint, "{9C0D1E2F-3A4B-4567-C8D9-E0F1A2B3C4D5}", AzPhysics::Joint);

        JoltJoint(
            JoltScene* scene,
            AzPhysics::SimulatedBodyHandle parentBody,
            AzPhysics::SimulatedBodyHandle childBody,
            JPH::Constraint* constraint);
        ~JoltJoint() override = default;

        //! Removes the constraint from the physics system (deferred object deletion
        //! is handled by the scene).
        void RemoveFromJoltWorld();

        JPH::Constraint* GetConstraint() const { return m_constraint; }

        // AzPhysics::Joint
        AZ::Crc32 GetNativeType() const override;
        void* GetNativePointer() const override;
        AzPhysics::SimulatedBodyHandle GetParentBodyHandle() const override;
        AzPhysics::SimulatedBodyHandle GetChildBodyHandle() const override;
        void SetParentBody(AzPhysics::SimulatedBodyHandle parentBody) override;
        void SetChildBody(AzPhysics::SimulatedBodyHandle childBody) override;

    private:
        JoltScene* m_scene = nullptr;
        AzPhysics::SimulatedBodyHandle m_parentBody = AzPhysics::InvalidSimulatedBodyHandle;
        AzPhysics::SimulatedBodyHandle m_childBody = AzPhysics::InvalidSimulatedBodyHandle;
        JPH::Constraint* m_constraint = nullptr;
    };

    //! Builds the JPH constraint for an AzPhysics joint configuration. Returns nullptr
    //! for unsupported configuration types. Joint frames come from the configuration's
    //! parent/child local transforms; the joint-frame X axis is the primary axis
    //! (hinge/slider axis, twist axis), matching the PhysX convention.
    JPH::Constraint* CreateJoltConstraint(
        const AzPhysics::JointConfiguration& configuration, JPH::Body& parentBody, JPH::Body& childBody);
} // namespace JoltPhysics
