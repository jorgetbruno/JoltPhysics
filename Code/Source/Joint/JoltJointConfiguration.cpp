#include <Joint/JoltJointConfiguration.h>

#include <AzCore/Serialization/SerializeContext.h>

#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void JointGenericProperties::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JointGenericProperties>()
                ->Version(1)
                ->Field("Flags", &JointGenericProperties::m_flags)
                ->Field("ForceMax", &JointGenericProperties::m_forceMax)
                ->Field("TorqueMax", &JointGenericProperties::m_torqueMax)
                ;
        }
    }

    void JointLimitProperties::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JointLimitProperties>()
                ->Version(1)
                ->Field("IsLimited", &JointLimitProperties::m_isLimited)
                ->Field("IsSoftLimit", &JointLimitProperties::m_isSoftLimit)
                ->Field("Damping", &JointLimitProperties::m_damping)
                ->Field("LimitFirst", &JointLimitProperties::m_limitFirst)
                ->Field("LimitSecond", &JointLimitProperties::m_limitSecond)
                ->Field("Stiffness", &JointLimitProperties::m_stiffness)
                ->Field("Tolerance", &JointLimitProperties::m_tolerance)
                ;
        }
    }

    void JointMotorProperties::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JointMotorProperties>()
                ->Version(1)
                ->Field("UseMotor", &JointMotorProperties::m_useMotor)
                ->Field("DriveForceLimit", &JointMotorProperties::m_driveForceLimit)
                ;
        }
    }

    void JoltFixedJointConfiguration::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<JointGenericProperties>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltFixedJointConfiguration, AzPhysics::JointConfiguration>()
                ->Version(1)
                ->Field("GenericProperties", &JoltFixedJointConfiguration::m_genericProperties)
                ;
        }
    }

    void JoltBallJointConfiguration::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<JointGenericProperties>(context);
        Internal::ReflectOnce<JointLimitProperties>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltBallJointConfiguration, AzPhysics::JointConfiguration>()
                ->Version(1)
                ->Field("GenericProperties", &JoltBallJointConfiguration::m_genericProperties)
                ->Field("LimitProperties", &JoltBallJointConfiguration::m_limitProperties)
                ;
        }
    }

    void JoltHingeJointConfiguration::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<JointGenericProperties>(context);
        Internal::ReflectOnce<JointLimitProperties>(context);
        Internal::ReflectOnce<JointMotorProperties>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltHingeJointConfiguration, AzPhysics::JointConfiguration>()
                ->Version(1)
                ->Field("GenericProperties", &JoltHingeJointConfiguration::m_genericProperties)
                ->Field("LimitProperties", &JoltHingeJointConfiguration::m_limitProperties)
                ->Field("MotorProperties", &JoltHingeJointConfiguration::m_motorProperties)
                ;
        }
    }

    void JoltPrismaticJointConfiguration::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<JointGenericProperties>(context);
        Internal::ReflectOnce<JointLimitProperties>(context);
        Internal::ReflectOnce<JointMotorProperties>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltPrismaticJointConfiguration, AzPhysics::JointConfiguration>()
                ->Version(1)
                ->Field("GenericProperties", &JoltPrismaticJointConfiguration::m_genericProperties)
                ->Field("LimitProperties", &JoltPrismaticJointConfiguration::m_limitProperties)
                ->Field("MotorProperties", &JoltPrismaticJointConfiguration::m_motorProperties)
                ;
        }
    }

    void JoltD6JointLimitConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltD6JointLimitConfiguration, AzPhysics::JointConfiguration>()
                ->Version(1)
                ->Field("SwingLimitY", &JoltD6JointLimitConfiguration::m_swingLimitY)
                ->Field("SwingLimitZ", &JoltD6JointLimitConfiguration::m_swingLimitZ)
                ->Field("TwistLimitLower", &JoltD6JointLimitConfiguration::m_twistLimitLower)
                ->Field("TwistLimitUpper", &JoltD6JointLimitConfiguration::m_twistLimitUpper)
                ;
        }
    }
} // namespace JoltPhysics
