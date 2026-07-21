#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/TransformBus.h>

#include <AzFramework/Physics/Common/PhysicsJoint.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>

#include <JoltPhysics/JoltPhysicsBus.h>

namespace JPH
{
    class Constraint;
}

namespace JoltPhysics
{
    //! Mirrors PhysX::JointComponentConfiguration: the lead (parent) and follower
    //! (child) entities plus the joint frame in the follower entity's local space.
    struct JoltJointComponentConfiguration
    {
        AZ_CLASS_ALLOCATOR(JoltJointComponentConfiguration, AZ::SystemAllocator);
        AZ_TYPE_INFO(JoltJointComponentConfiguration, "{AD0E1F2A-4B5C-4678-D9E0-F1A2B3C4D5E6}");
        static void Reflect(AZ::ReflectContext* context);

        AZ::EntityId m_leadEntity; //!< Entity with the parent body of the joint.
        AZ::EntityId m_followerEntity; //!< Entity with the child body (own entity when invalid).
        AZ::Transform m_localTransformFromFollower = AZ::Transform::CreateIdentity(); //!< Joint frame in follower space.
    };

    //! Base class for the Jolt joint components: resolves the lead/follower bodies,
    //! computes the joint frames and creates the joint in the default scene.
    class JoltJointComponentBase
        : public AZ::Component
        , private AZ::TickBus::Handler
        , private JoltJointRequestBus::Handler
    {
    public:
        AZ_RTTI(JoltJointComponentBase, "{BE1F2A3B-5C6D-4789-E0F1-A2B3C4D5E6F7}", AZ::Component);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

    protected:
        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // JoltJointRequestBus (typed components override the meaningful parts)
        float GetPosition() const override { return 0.0f; }
        float GetVelocity() const override { return 0.0f; }
        AZ::Transform GetTransform() const override;
        void SetVelocity([[maybe_unused]] float velocity) override {}
        void SetMaximumForce([[maybe_unused]] float force) override {}
        AZStd::pair<float, float> GetLimits() const override { return { 0.0f, 0.0f }; }

        //! Assembles the backend joint configuration (frames are filled in by the base).
        virtual AZStd::unique_ptr<AzPhysics::JointConfiguration> BuildJointConfiguration() const = 0;

        void CreateJoint();
        void DestroyJoint();
        JPH::Constraint* GetNativeConstraint() const;
        AZ::Transform GetJointWorldTransform() const;

        JoltJointComponentConfiguration m_configuration;
        AzPhysics::JointHandle m_jointHandle = AzPhysics::InvalidJointHandle;
        AzPhysics::SceneHandle m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
    };
} // namespace JoltPhysics
