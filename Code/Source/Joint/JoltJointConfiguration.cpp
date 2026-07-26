#include <Joint/JoltJointConfiguration.h>

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>

#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    AZ::Crc32 JointGenericProperties::GetBreakableVisibility() const
    {
        return IsFlagSet(GenericJointFlag::Breakable) ? AZ::Edit::PropertyVisibility::Show
                                                      : AZ::Edit::PropertyVisibility::Hide;
    }

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

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JointGenericProperties>("Generic Joint Properties", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &JointGenericProperties::m_flags,
                        "Flags", "Optional joint behaviours.")
                        ->EnumAttribute(JointGenericProperties::GenericJointFlag::None, "None")
                        ->EnumAttribute(JointGenericProperties::GenericJointFlag::Breakable, "Breakable")
                        ->EnumAttribute(JointGenericProperties::GenericJointFlag::SelfCollide, "Self-collide")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JointGenericProperties::m_forceMax,
                        "Max force", "Reaction force (N) at which the joint breaks and is removed. 0 = never break on force.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JointGenericProperties::GetBreakableVisibility)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JointGenericProperties::m_torqueMax,
                        "Max torque", "Reaction torque (N m) at which the joint breaks and is removed. 0 = never break on torque.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JointGenericProperties::GetBreakableVisibility)
                    ;
            }
        }
    }

    AZ::Crc32 JointLimitProperties::GetLimitVisibility() const
    {
        return m_isLimited ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
    }

    AZ::Crc32 JointLimitProperties::GetSoftLimitVisibility() const
    {
        return (m_isLimited && m_isSoftLimit) ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
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

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JointLimitProperties>("Joint Limit Properties", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JointLimitProperties::m_isLimited,
                        "Is limited", "Whether the joint's degree of freedom is limited.")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JointLimitProperties::m_limitFirst,
                        "Limit lower", "Lower limit. Hinge: angle (deg). Prismatic: min slide (m). Ball: cone half-angle about joint Y (deg).")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JointLimitProperties::GetLimitVisibility)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JointLimitProperties::m_limitSecond,
                        "Limit upper", "Upper limit. Hinge: angle (deg). Prismatic: max slide (m). Ball: cone half-angle about joint Z (deg).")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JointLimitProperties::GetLimitVisibility)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JointLimitProperties::m_isSoftLimit,
                        "Soft limit", "Soft limits push back with a spring + damping instead of stopping dead. Hinge and prismatic only.")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JointLimitProperties::GetLimitVisibility)
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JointLimitProperties::m_stiffness,
                        "Stiffness", "Soft-limit spring stiffness (N/m for prismatic, N m/rad for hinge).")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JointLimitProperties::GetSoftLimitVisibility)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JointLimitProperties::m_damping,
                        "Damping", "Soft-limit damping coefficient (N s/m for prismatic, N m s/rad for hinge).")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JointLimitProperties::GetSoftLimitVisibility)
                    ;
            }
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

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JointMotorProperties>("Joint Motor Properties", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JointMotorProperties::m_useMotor,
                        "Use motor", "Enables joint actuation (driven by a JointDriver / script).")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JointMotorProperties::m_driveForceLimit,
                        "Drive force limit", "Maximum force/torque the motor applies.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ;
            }
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

    void JoltDistanceJointConfiguration::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<JointGenericProperties>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltDistanceJointConfiguration, AzPhysics::JointConfiguration>()
                ->Version(1)
                ->Field("GenericProperties", &JoltDistanceJointConfiguration::m_genericProperties)
                ->Field("MinDistance", &JoltDistanceJointConfiguration::m_minDistance)
                ->Field("MaxDistance", &JoltDistanceJointConfiguration::m_maxDistance)
                ->Field("SpringFrequency", &JoltDistanceJointConfiguration::m_springFrequency)
                ->Field("SpringDamping", &JoltDistanceJointConfiguration::m_springDamping)
                ;
        }
    }

    void JoltConeJointConfiguration::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<JointGenericProperties>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltConeJointConfiguration, AzPhysics::JointConfiguration>()
                ->Version(1)
                ->Field("GenericProperties", &JoltConeJointConfiguration::m_genericProperties)
                ->Field("HalfConeAngle", &JoltConeJointConfiguration::m_halfConeAngle)
                ;
        }
    }

    void JoltSwingTwistJointConfiguration::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<JointGenericProperties>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltSwingTwistJointConfiguration, AzPhysics::JointConfiguration>()
                ->Version(1)
                ->Field("GenericProperties", &JoltSwingTwistJointConfiguration::m_genericProperties)
                ->Field("NormalHalfConeAngle", &JoltSwingTwistJointConfiguration::m_normalHalfConeAngle)
                ->Field("PlaneHalfConeAngle", &JoltSwingTwistJointConfiguration::m_planeHalfConeAngle)
                ->Field("TwistLower", &JoltSwingTwistJointConfiguration::m_twistLower)
                ->Field("TwistUpper", &JoltSwingTwistJointConfiguration::m_twistUpper)
                ;
        }
    }

    void JoltGearJointConfiguration::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<JointGenericProperties>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            // The hinge joint handles are deliberately not fields: a JointHandle is a
            // runtime index into a scene's joint list, meaningless once serialized.
            // Whatever creates the gear resolves them and fills them in.
            serializeContext->Class<JoltGearJointConfiguration, AzPhysics::JointConfiguration>()
                ->Version(1)
                ->Field("GenericProperties", &JoltGearJointConfiguration::m_genericProperties)
                ->Field("ParentNumTeeth", &JoltGearJointConfiguration::m_parentNumTeeth)
                ->Field("ChildNumTeeth", &JoltGearJointConfiguration::m_childNumTeeth)
                ;
        }
    }

    void JoltRackAndPinionJointConfiguration::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<JointGenericProperties>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltRackAndPinionJointConfiguration, AzPhysics::JointConfiguration>()
                ->Version(1)
                ->Field("GenericProperties", &JoltRackAndPinionJointConfiguration::m_genericProperties)
                ->Field("PinionNumTeeth", &JoltRackAndPinionJointConfiguration::m_pinionNumTeeth)
                ->Field("RackNumTeeth", &JoltRackAndPinionJointConfiguration::m_rackNumTeeth)
                ->Field("RackLength", &JoltRackAndPinionJointConfiguration::m_rackLength)
                ;
        }
    }
} // namespace JoltPhysics
