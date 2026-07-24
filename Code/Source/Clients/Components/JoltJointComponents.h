#pragma once

#include <Clients/Components/JoltJointComponentBase.h>
#include <Joint/JoltJointConfiguration.h>

namespace JoltPhysics
{
    //! Fixed joint: welds the follower body to the lead body.
    class JoltFixedJointComponent
        : public JoltJointComponentBase
    {
    public:
        AZ_COMPONENT(JoltFixedJointComponent, "{CF2A3B4C-6D7E-4890-F1A2-B3C4D5E6F7A8}", JoltJointComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        JoltFixedJointComponent() = default;

        JointGenericProperties& GetGenericProperties()
        {
            return m_genericProperties;
        }

    protected:
        AZStd::unique_ptr<AzPhysics::JointConfiguration> BuildJointConfiguration() const override;

        JointGenericProperties m_genericProperties;
    };

    //! Ball-and-socket joint: free rotation inside a swing cone, twist free.
    class JoltBallJointComponent
        : public JoltJointComponentBase
    {
    public:
        AZ_COMPONENT(JoltBallJointComponent, "{D03B4C5D-7E8F-4901-A2B3-C4D5E6F7A8B9}", JoltJointComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        JoltBallJointComponent() = default;

        JointGenericProperties& GetGenericProperties()
        {
            return m_genericProperties;
        }
        JointLimitProperties& GetLimitProperties()
        {
            return m_limitProperties;
        }

    protected:
        AZStd::unique_ptr<AzPhysics::JointConfiguration> BuildJointConfiguration() const override;
        AZStd::pair<float, float> GetLimits() const override;

        JointGenericProperties m_genericProperties;
        JointLimitProperties m_limitProperties;
    };

    //! Hinge joint: rotation about the joint-frame X axis with limits and a motor.
    class JoltHingeJointComponent
        : public JoltJointComponentBase
    {
    public:
        AZ_COMPONENT(JoltHingeJointComponent, "{E14C5D6E-8F9A-4012-B3C4-D5E6F7A8B9C0}", JoltJointComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        JoltHingeJointComponent() = default;

        JointGenericProperties& GetGenericProperties()
        {
            return m_genericProperties;
        }
        JointLimitProperties& GetLimitProperties()
        {
            return m_limitProperties;
        }
        JointMotorProperties& GetMotorProperties()
        {
            return m_motorProperties;
        }

    protected:
        AZStd::unique_ptr<AzPhysics::JointConfiguration> BuildJointConfiguration() const override;

        // JoltJointRequestBus
        float GetPosition() const override;
        float GetVelocity() const override;
        void SetVelocity(float velocity) override;
        void SetMaximumForce(float force) override;
        AZStd::pair<float, float> GetLimits() const override;

        JointGenericProperties m_genericProperties;
        JointLimitProperties m_limitProperties;
        JointMotorProperties m_motorProperties;
    };

    //! Prismatic joint: sliding along the joint-frame X axis with limits and a motor.
    class JoltPrismaticJointComponent
        : public JoltJointComponentBase
    {
    public:
        AZ_COMPONENT(JoltPrismaticJointComponent, "{F25D6E7F-9A0B-4123-C4D5-E6F7A8B9C0D1}", JoltJointComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        JoltPrismaticJointComponent() = default;

        JointGenericProperties& GetGenericProperties()
        {
            return m_genericProperties;
        }
        JointLimitProperties& GetLimitProperties()
        {
            return m_limitProperties;
        }
        JointMotorProperties& GetMotorProperties()
        {
            return m_motorProperties;
        }

    protected:
        AZStd::unique_ptr<AzPhysics::JointConfiguration> BuildJointConfiguration() const override;

        // JoltJointRequestBus
        float GetPosition() const override;
        float GetVelocity() const override;
        void SetVelocity(float velocity) override;
        void SetMaximumForce(float force) override;
        AZStd::pair<float, float> GetLimits() const override;

        JointGenericProperties m_genericProperties;
        JointLimitProperties m_limitProperties;
        JointMotorProperties m_motorProperties;
    };

    //! Distance joint: keeps the two bodies within [min, max] metres, optionally sprung.
    class JoltDistanceJointComponent
        : public JoltJointComponentBase
    {
    public:
        AZ_COMPONENT(JoltDistanceJointComponent, "{C3D4E5F6-A7B8-4293-A4B5-C6D7E8F1A2B3}", JoltJointComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        JoltDistanceJointComponent() = default;

        JointGenericProperties& GetGenericProperties() { return m_genericProperties; }
        void SetDistanceParams(float minDistance, float maxDistance, float springFrequency, float springDamping)
        {
            m_minDistance = minDistance;
            m_maxDistance = maxDistance;
            m_springFrequency = springFrequency;
            m_springDamping = springDamping;
        }

    protected:
        AZStd::unique_ptr<AzPhysics::JointConfiguration> BuildJointConfiguration() const override;

        JointGenericProperties m_genericProperties;
        float m_minDistance = 0.0f;
        float m_maxDistance = 1.0f;
        float m_springFrequency = 0.0f;
        float m_springDamping = 0.0f;
    };

    //! Cone joint: locks position, limits swing from the joint-frame X axis to a half-cone.
    class JoltConeJointComponent
        : public JoltJointComponentBase
    {
    public:
        AZ_COMPONENT(JoltConeJointComponent, "{D4E5F6A7-B8C9-43A4-B5C6-D7E8F1A2B3C4}", JoltJointComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        JoltConeJointComponent() = default;

        JointGenericProperties& GetGenericProperties() { return m_genericProperties; }
        void SetHalfConeAngle(float halfConeAngle) { m_halfConeAngle = halfConeAngle; }

    protected:
        AZStd::unique_ptr<AzPhysics::JointConfiguration> BuildJointConfiguration() const override;

        JointGenericProperties m_genericProperties;
        float m_halfConeAngle = 45.0f;
    };

    //! Swing-twist joint (e.g. a humanoid shoulder): separate swing half-cones and a twist range.
    class JoltSwingTwistJointComponent
        : public JoltJointComponentBase
    {
    public:
        AZ_COMPONENT(JoltSwingTwistJointComponent, "{E5F6A7B8-C9DA-44B5-C6D7-E8F1A2B3C4D5}", JoltJointComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        JoltSwingTwistJointComponent() = default;

        JointGenericProperties& GetGenericProperties() { return m_genericProperties; }
        void SetSwingTwistLimits(float normalHalfConeAngle, float planeHalfConeAngle, float twistLower, float twistUpper)
        {
            m_normalHalfConeAngle = normalHalfConeAngle;
            m_planeHalfConeAngle = planeHalfConeAngle;
            m_twistLower = twistLower;
            m_twistUpper = twistUpper;
        }

    protected:
        AZStd::unique_ptr<AzPhysics::JointConfiguration> BuildJointConfiguration() const override;

        JointGenericProperties m_genericProperties;
        float m_normalHalfConeAngle = 45.0f;
        float m_planeHalfConeAngle = 45.0f;
        float m_twistLower = -45.0f;
        float m_twistUpper = 45.0f;
    };

    //! 6-DOF joint with locked linear axes and swing/twist angular limits.
    class JoltD6JointComponent
        : public JoltJointComponentBase
    {
    public:
        AZ_COMPONENT(JoltD6JointComponent, "{036E7F80-A1B2-4234-D5E6-F7A8B9C0D1E2}", JoltJointComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        JoltD6JointComponent() = default;

        void SetD6Limits(float swingLimitY, float swingLimitZ, float twistLimitLower, float twistLimitUpper)
        {
            m_swingLimitY = swingLimitY;
            m_swingLimitZ = swingLimitZ;
            m_twistLimitLower = twistLimitLower;
            m_twistLimitUpper = twistLimitUpper;
        }

    protected:
        AZStd::unique_ptr<AzPhysics::JointConfiguration> BuildJointConfiguration() const override;

        float m_swingLimitY = 45.0f;
        float m_swingLimitZ = 45.0f;
        float m_twistLimitLower = -45.0f;
        float m_twistLimitUpper = 45.0f;
    };
} // namespace JoltPhysics
