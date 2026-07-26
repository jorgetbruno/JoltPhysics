#pragma once

#include <AzCore/Math/Crc.h>
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/RTTI/RTTI.h>

#include <AzFramework/Physics/Common/PhysicsTypes.h>
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

        //! Edit-context helper: the break thresholds only show on a breakable joint.
        AZ::Crc32 GetBreakableVisibility() const;

        GenericJointFlag m_flags = GenericJointFlag::None;
        float m_forceMax = 1.0f; //!< Reaction force that breaks the joint (Breakable only; 0 = never on force).
        float m_torqueMax = 1.0f; //!< Reaction torque that breaks the joint (Breakable only; 0 = never on torque).
    };

    //! Mirrors PhysX::JointLimitProperties. Semantics of m_limitFirst/m_limitSecond
    //! depend on the joint type (documented per configuration).
    struct JointLimitProperties
    {
        AZ_CLASS_ALLOCATOR(JointLimitProperties, AZ::SystemAllocator);
        AZ_TYPE_INFO(JointLimitProperties, "{2B3C4D5E-6F7A-4890-B1C2-D3E4F5A6B7C8}");
        static void Reflect(AZ::ReflectContext* context);

        //! Edit-context helpers: limit fields only show when the limit is on, and the
        //! spring fields only when the limit is soft.
        AZ::Crc32 GetLimitVisibility() const;
        AZ::Crc32 GetSoftLimitVisibility() const;

        bool m_isLimited = true; //!< Whether the joint DOF is limited at all.
        bool m_isSoftLimit = false; //!< Soft limits push back with a spring instead of stopping dead.
        float m_damping = 20.0f; //!< Soft-limit damping coefficient (N s/m linear, N m s/rad angular).
        float m_limitFirst = 45.0f; //!< Hinge: lower angle (deg). Prismatic: min slide (m). Ball: cone half-angle about joint Y (deg).
        float m_limitSecond = 45.0f; //!< Hinge: upper angle (deg). Prismatic: max slide (m). Ball: cone half-angle about joint Z (deg).
        float m_stiffness = 100.0f; //!< Soft-limit spring stiffness (N/m linear, N m/rad angular).
        float m_tolerance = 0.1f; //!< Unused: kept so PhysX-era data loads. Jolt limits engage exactly at the limit.
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

    //! Distance joint: keeps the two attachment points within [m_minDistance, m_maxDistance]
    //! metres of each other. A non-zero spring frequency turns the limits into a soft spring.
    struct JoltDistanceJointConfiguration : public AzPhysics::JointConfiguration
    {
        AZ_CLASS_ALLOCATOR(JoltDistanceJointConfiguration, AZ::SystemAllocator);
        AZ_RTTI(JoltDistanceJointConfiguration, "{9A0B1C2D-3E4F-5061-7283-94A5B6C7D8E9}", AzPhysics::JointConfiguration);
        static void Reflect(AZ::ReflectContext* context);

        JointGenericProperties m_genericProperties;
        float m_minDistance = 0.0f; //!< Minimum separation (m).
        float m_maxDistance = 1.0f; //!< Maximum separation (m).
        float m_springFrequency = 0.0f; //!< Spring oscillation frequency (Hz); 0 = hard limits.
        float m_springDamping = 0.0f;   //!< Spring damping ratio (used when frequency > 0).
    };

    //! Cone joint: locks position and limits the swing away from the joint-frame X axis
    //! to m_halfConeAngle degrees; twist about X stays free.
    struct JoltConeJointConfiguration : public AzPhysics::JointConfiguration
    {
        AZ_CLASS_ALLOCATOR(JoltConeJointConfiguration, AZ::SystemAllocator);
        AZ_RTTI(JoltConeJointConfiguration, "{A1B2C3D4-E5F6-4071-8293-A4B5C6D7E8F1}", AzPhysics::JointConfiguration);
        static void Reflect(AZ::ReflectContext* context);

        JointGenericProperties m_genericProperties;
        float m_halfConeAngle = 45.0f; //!< Swing half-cone angle from the joint-frame X axis (degrees).
    };

    //! Swing-twist joint (e.g. a humanoid shoulder): twist about joint X limited to
    //! [m_twistLower, m_twistUpper] degrees, swing limited by separate half-cone angles
    //! about the joint-frame Y (normal) and Z (plane) axes.
    struct JoltSwingTwistJointConfiguration : public AzPhysics::JointConfiguration
    {
        AZ_CLASS_ALLOCATOR(JoltSwingTwistJointConfiguration, AZ::SystemAllocator);
        AZ_RTTI(JoltSwingTwistJointConfiguration, "{B2C3D4E5-F6A7-4182-93A4-B5C6D7E8F1A2}", AzPhysics::JointConfiguration);
        static void Reflect(AZ::ReflectContext* context);

        JointGenericProperties m_genericProperties;
        float m_normalHalfConeAngle = 45.0f; //!< Swing half-angle about joint-frame Y (degrees).
        float m_planeHalfConeAngle = 45.0f;  //!< Swing half-angle about joint-frame Z (degrees).
        float m_twistLower = -45.0f; //!< Lower twist limit about joint-frame X (degrees).
        float m_twistUpper = 45.0f;  //!< Upper twist limit about joint-frame X (degrees).
    };

    //! Gear: couples the rotation of two bodies so one turns the other, at a ratio set by
    //! their tooth counts. There is no gear geometry — nothing meshes and nothing is
    //! checked for interpenetration, so the "teeth" only express the ratio.
    //!
    //! **This constraint does not hold the bodies in place.** It couples rotation and
    //! nothing else, so each body needs its own hinge joint as well or it will simply
    //! drift off. Point the hinge joints at the same axes given here.
    //!
    //! Ratio: parentRotation = -(childTeeth / parentTeeth) * childRotation, so a small
    //! parent driving a large child turns the child more slowly, and in the opposite
    //! direction, as real gears do.
    struct JoltGearJointConfiguration : public AzPhysics::JointConfiguration
    {
        AZ_CLASS_ALLOCATOR(JoltGearJointConfiguration, AZ::SystemAllocator);
        AZ_RTTI(JoltGearJointConfiguration, "{687F775C-23E2-45FC-A703-1D1BD8BDAEBA}", AzPhysics::JointConfiguration);
        static void Reflect(AZ::ReflectContext* context);

        JointGenericProperties m_genericProperties;

        //! Tooth counts. Only their ratio matters; both must be non-zero.
        AZ::u32 m_parentNumTeeth = 1;
        AZ::u32 m_childNumTeeth = 1;

        //! The hinge joints holding each gear on its axle, if they have been created.
        //! Optional: Jolt uses them only to measure accumulated rotation error and correct
        //! the drift a velocity-level coupling otherwise builds up over time. The gearing
        //! itself works without them, but two gears left running will slowly lose phase.
        AzPhysics::JointHandle m_parentHingeJoint = AzPhysics::InvalidJointHandle;
        AzPhysics::JointHandle m_childHingeJoint = AzPhysics::InvalidJointHandle;
    };

    //! Rack and pinion: couples the rotation of the parent (the pinion, a gear on a hinge)
    //! to the translation of the child (the rack, a bar on a slider). Same caveat as the
    //! gear joint — it couples motion only, so the pinion still needs a hinge joint and
    //! the rack a prismatic joint.
    //!
    //! Ratio: pinionRotation = (2*pi * rackTeeth / (rackLength * pinionTeeth)) * rackTranslation.
    struct JoltRackAndPinionJointConfiguration : public AzPhysics::JointConfiguration
    {
        AZ_CLASS_ALLOCATOR(JoltRackAndPinionJointConfiguration, AZ::SystemAllocator);
        AZ_RTTI(JoltRackAndPinionJointConfiguration, "{55C43F2C-528E-4885-BD55-A84AC1F9344F}", AzPhysics::JointConfiguration);
        static void Reflect(AZ::ReflectContext* context);

        JointGenericProperties m_genericProperties;

        AZ::u32 m_pinionNumTeeth = 1; //!< Teeth on the parent gear. Must be non-zero.
        AZ::u32 m_rackNumTeeth = 1; //!< Teeth cut into the child bar.
        float m_rackLength = 1.0f; //!< Length of the child bar in metres. Must be non-zero.

        //! The pinion's hinge joint and the rack's prismatic joint, for drift correction.
        //! Optional, as with the gear joint.
        AzPhysics::JointHandle m_pinionHingeJoint = AzPhysics::InvalidJointHandle;
        AzPhysics::JointHandle m_rackSliderJoint = AzPhysics::InvalidJointHandle;
    };
} // namespace JoltPhysics
