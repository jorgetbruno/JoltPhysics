#pragma once

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/RTTI/RTTI.h>

#include <AzFramework/Physics/Configuration/JointConfiguration.h>

namespace JoltPhysics
{
    //! Mirrors PhysX::JointGenericProperties (fields kept API-compatible).
    struct JointGenericProperties
    {
        enum class GenericJointFlag : AZ::u16
        {
            None = 0,
            Breakable = 1,
            SelfCollide = 1 << 1
        };

        AZ_CLASS_ALLOCATOR(JointGenericProperties, AZ::SystemAllocator);
        AZ_TYPE_INFO(JointGenericProperties, "{1A2B3C4D-5E6F-4789-A0B1-C2D3E4F5A6B7}");
        static void Reflect(AZ::ReflectContext* context);

        bool IsFlagSet(GenericJointFlag flag) const
        {
            return (static_cast<AZ::u16>(m_flags) & static_cast<AZ::u16>(flag)) != 0;
        }

        GenericJointFlag m_flags = GenericJointFlag::None;
        float m_forceMax = 1.0f; //!< Max force the joint tolerates before breaking (Breakable only).
        float m_torqueMax = 1.0f; //!< Max torque the joint tolerates before breaking (Breakable only).
    };

    //! Mirrors PhysX::JointLimitProperties. Semantics of m_limitFirst/m_limitSecond
    //! depend on the joint type (documented per configuration).
    struct JointLimitProperties
    {
        AZ_CLASS_ALLOCATOR(JointLimitProperties, AZ::SystemAllocator);
        AZ_TYPE_INFO(JointLimitProperties, "{2B3C4D5E-6F7A-4890-B1C2-D3E4F5A6B7C8}");
        static void Reflect(AZ::ReflectContext* context);

        bool m_isLimited = true; //!< Whether the joint DOF is limited at all.
        bool m_isSoftLimit = false; //!< Soft limits use spring+damping, hard limits use tolerance.
        float m_damping = 20.0f; //!< Soft-limit damping.
        float m_limitFirst = 45.0f; //!< Hinge: lower angle (deg). Prismatic: min slide (m). Ball: cone half-angle about joint Y (deg).
        float m_limitSecond = 45.0f; //!< Hinge: upper angle (deg). Prismatic: max slide (m). Ball: cone half-angle about joint Z (deg).
        float m_stiffness = 100.0f; //!< Soft-limit spring strength.
        float m_tolerance = 0.1f; //!< Hard-limit: distance at which the limit becomes enforced.
    };

    //! Mirrors PhysX::JointMotorProperties.
    struct JointMotorProperties
    {
        AZ_CLASS_ALLOCATOR(JointMotorProperties, AZ::SystemAllocator);
        AZ_TYPE_INFO(JointMotorProperties, "{3C4D5E6F-7A8B-4901-C2D3-E4F5A6B7C8D9}");
        static void Reflect(AZ::ReflectContext* context);

        bool m_useMotor = false; //!< Enables joint actuation.
        float m_driveForceLimit = 1000.0f; //!< Max force/torque the motor applies.
    };

    //! Welds the two bodies rigidly together.
    struct JoltFixedJointConfiguration : public AzPhysics::JointConfiguration
    {
        AZ_CLASS_ALLOCATOR(JoltFixedJointConfiguration, AZ::SystemAllocator);
        AZ_RTTI(JoltFixedJointConfiguration, "{4D5E6F7A-8B9C-4012-D3E4-F5A6B7C8D9E0}", AzPhysics::JointConfiguration);
        static void Reflect(AZ::ReflectContext* context);

        JointGenericProperties m_genericProperties;
    };

    //! Ball-and-socket: free rotation inside a swing cone (limitFirst = cone half-angle
    //! about joint-frame Y, limitSecond = about joint-frame Z), twist stays free.
    struct JoltBallJointConfiguration : public AzPhysics::JointConfiguration
    {
        AZ_CLASS_ALLOCATOR(JoltBallJointConfiguration, AZ::SystemAllocator);
        AZ_RTTI(JoltBallJointConfiguration, "{5E6F7A8B-9C0D-4123-E4F5-A6B7C8D9E0F1}", AzPhysics::JointConfiguration);
        static void Reflect(AZ::ReflectContext* context);

        JointGenericProperties m_genericProperties;
        JointLimitProperties m_limitProperties;
    };

    //! Hinge: rotation about the joint-frame X axis, limited between limitFirst
    //! (lower, degrees) and limitSecond (upper, degrees); optional velocity/position motor.
    struct JoltHingeJointConfiguration : public AzPhysics::JointConfiguration
    {
        AZ_CLASS_ALLOCATOR(JoltHingeJointConfiguration, AZ::SystemAllocator);
        AZ_RTTI(JoltHingeJointConfiguration, "{6F7A8B9C-0D1E-4234-F5A6-B7C8D9E0F1A2}", AzPhysics::JointConfiguration);
        static void Reflect(AZ::ReflectContext* context);

        JointGenericProperties m_genericProperties;
        JointLimitProperties m_limitProperties;
        JointMotorProperties m_motorProperties;
    };

    //! Prismatic: no rotation, sliding along the joint-frame X axis, limited between
    //! limitFirst (min, meters) and limitSecond (max, meters); optional motor.
    struct JoltPrismaticJointConfiguration : public AzPhysics::JointConfiguration
    {
        AZ_CLASS_ALLOCATOR(JoltPrismaticJointConfiguration, AZ::SystemAllocator);
        AZ_RTTI(JoltPrismaticJointConfiguration, "{7A8B9C0D-1E2F-4345-A6B7-C8D9E0F1A2B3}", AzPhysics::JointConfiguration);
        static void Reflect(AZ::ReflectContext* context);

        JointGenericProperties m_genericProperties;
        JointLimitProperties m_limitProperties;
        JointMotorProperties m_motorProperties;
    };

    //! 6-DOF with linear axes locked, twist about joint X limited to
    //! [m_twistLimitLower, m_twistLimitUpper] degrees and swing about joint Y/Z limited
    //! to ±m_swingLimitY / ±m_swingLimitZ degrees (mirrors PhysX::D6JointLimitConfiguration).
    struct JoltD6JointLimitConfiguration : public AzPhysics::JointConfiguration
    {
        AZ_CLASS_ALLOCATOR(JoltD6JointLimitConfiguration, AZ::SystemAllocator);
        AZ_RTTI(JoltD6JointLimitConfiguration, "{8B9C0D1E-2F3A-4456-B7C8-D9E0F1A2B3C4}", AzPhysics::JointConfiguration);
        static void Reflect(AZ::ReflectContext* context);

        float m_swingLimitY = 45.0f; //!< Max angle in degrees from the joint-frame Y axis.
        float m_swingLimitZ = 45.0f; //!< Max angle in degrees from the joint-frame Z axis.
        float m_twistLimitLower = -45.0f; //!< Lower limit in degrees for rotation about the joint-frame X axis.
        float m_twistLimitUpper = 45.0f; //!< Upper limit in degrees for rotation about the joint-frame X axis.
    };
} // namespace JoltPhysics
