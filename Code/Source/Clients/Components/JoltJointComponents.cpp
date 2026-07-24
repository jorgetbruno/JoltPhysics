#include <Clients/Components/JoltJointComponents.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>

#include <Utils/Conversions.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace JoltPhysics
{
    // --- JoltFixedJointComponent ---

    void JoltFixedJointComponent::Reflect(AZ::ReflectContext* context)
    {
        JoltJointComponentBase::Reflect(context);
        JoltFixedJointConfiguration::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltFixedJointComponent, JoltJointComponentBase>()
                ->Version(1)
                ->Field("GenericProperties", &JoltFixedJointComponent::m_genericProperties)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltFixedJointComponent>("Jolt Fixed Joint", "Fixed joint simulated by the Jolt physics backend")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        // No AppearsInAddComponentMenu: the corresponding editor joint component
                        // owns the menu entry (PhysX-style editor/runtime split). The runtime
                        // component stays registered for old prefabs and BuildGameEntity.
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    AZStd::unique_ptr<AzPhysics::JointConfiguration> JoltFixedJointComponent::BuildJointConfiguration() const
    {
        auto configuration = AZStd::make_unique<JoltFixedJointConfiguration>();
        configuration->m_genericProperties = m_genericProperties;
        return configuration;
    }

    // --- JoltBallJointComponent ---

    void JoltBallJointComponent::Reflect(AZ::ReflectContext* context)
    {
        JoltJointComponentBase::Reflect(context);
        JoltBallJointConfiguration::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltBallJointComponent, JoltJointComponentBase>()
                ->Version(1)
                ->Field("GenericProperties", &JoltBallJointComponent::m_genericProperties)
                ->Field("LimitProperties", &JoltBallJointComponent::m_limitProperties)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltBallJointComponent>("Jolt Ball Joint", "Ball-and-socket joint simulated by the Jolt physics backend")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        // No AppearsInAddComponentMenu: the corresponding editor joint component
                        // owns the menu entry (PhysX-style editor/runtime split). The runtime
                        // component stays registered for old prefabs and BuildGameEntity.
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    AZStd::unique_ptr<AzPhysics::JointConfiguration> JoltBallJointComponent::BuildJointConfiguration() const
    {
        auto configuration = AZStd::make_unique<JoltBallJointConfiguration>();
        configuration->m_genericProperties = m_genericProperties;
        configuration->m_limitProperties = m_limitProperties;
        return configuration;
    }

    AZStd::pair<float, float> JoltBallJointComponent::GetLimits() const
    {
        return {
            AZ::DegToRad(m_limitProperties.m_limitFirst),
            AZ::DegToRad(m_limitProperties.m_limitSecond)
        };
    }

    // --- JoltHingeJointComponent ---

    void JoltHingeJointComponent::Reflect(AZ::ReflectContext* context)
    {
        JoltJointComponentBase::Reflect(context);
        JoltHingeJointConfiguration::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltHingeJointComponent, JoltJointComponentBase>()
                ->Version(1)
                ->Field("GenericProperties", &JoltHingeJointComponent::m_genericProperties)
                ->Field("LimitProperties", &JoltHingeJointComponent::m_limitProperties)
                ->Field("MotorProperties", &JoltHingeJointComponent::m_motorProperties)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltHingeJointComponent>("Jolt Hinge Joint", "Hinge joint simulated by the Jolt physics backend")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        // No AppearsInAddComponentMenu: the corresponding editor joint component
                        // owns the menu entry (PhysX-style editor/runtime split). The runtime
                        // component stays registered for old prefabs and BuildGameEntity.
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    AZStd::unique_ptr<AzPhysics::JointConfiguration> JoltHingeJointComponent::BuildJointConfiguration() const
    {
        auto configuration = AZStd::make_unique<JoltHingeJointConfiguration>();
        configuration->m_genericProperties = m_genericProperties;
        configuration->m_limitProperties = m_limitProperties;
        configuration->m_motorProperties = m_motorProperties;
        return configuration;
    }

    float JoltHingeJointComponent::GetPosition() const
    {
        if (auto* constraint = static_cast<JPH::HingeConstraint*>(GetNativeConstraint()))
        {
            return constraint->GetCurrentAngle();
        }
        return 0.0f;
    }

    float JoltHingeJointComponent::GetVelocity() const
    {
        // Angular velocity of the follower body projected onto the current world hinge axis.
        if (auto* constraint = static_cast<JPH::HingeConstraint*>(GetNativeConstraint()))
        {
            const AZ::Vector3 worldAxis = GetJointWorldTransform().GetRotation().TransformVector(AZ::Vector3::CreateAxisX());
            return constraint->GetBody2()->GetAngularVelocity().Dot(Conversions::ToJolt(worldAxis));
        }
        return 0.0f;
    }

    void JoltHingeJointComponent::SetVelocity(float velocity)
    {
        if (auto* constraint = static_cast<JPH::HingeConstraint*>(GetNativeConstraint()))
        {
            constraint->SetMotorState(JPH::EMotorState::Velocity);
            constraint->SetTargetAngularVelocity(velocity);
        }
    }

    void JoltHingeJointComponent::SetMaximumForce(float force)
    {
        if (auto* constraint = static_cast<JPH::HingeConstraint*>(GetNativeConstraint()))
        {
            constraint->GetMotorSettings().SetForceLimits(-force, force);
        }
    }

    AZStd::pair<float, float> JoltHingeJointComponent::GetLimits() const
    {
        return {
            AZ::DegToRad(m_limitProperties.m_limitFirst),
            AZ::DegToRad(m_limitProperties.m_limitSecond)
        };
    }

    // --- JoltPrismaticJointComponent ---

    void JoltPrismaticJointComponent::Reflect(AZ::ReflectContext* context)
    {
        JoltJointComponentBase::Reflect(context);
        JoltPrismaticJointConfiguration::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltPrismaticJointComponent, JoltJointComponentBase>()
                ->Version(1)
                ->Field("GenericProperties", &JoltPrismaticJointComponent::m_genericProperties)
                ->Field("LimitProperties", &JoltPrismaticJointComponent::m_limitProperties)
                ->Field("MotorProperties", &JoltPrismaticJointComponent::m_motorProperties)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltPrismaticJointComponent>("Jolt Prismatic Joint", "Prismatic joint simulated by the Jolt physics backend")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        // No AppearsInAddComponentMenu: the corresponding editor joint component
                        // owns the menu entry (PhysX-style editor/runtime split). The runtime
                        // component stays registered for old prefabs and BuildGameEntity.
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    AZStd::unique_ptr<AzPhysics::JointConfiguration> JoltPrismaticJointComponent::BuildJointConfiguration() const
    {
        auto configuration = AZStd::make_unique<JoltPrismaticJointConfiguration>();
        configuration->m_genericProperties = m_genericProperties;
        configuration->m_limitProperties = m_limitProperties;
        configuration->m_motorProperties = m_motorProperties;
        return configuration;
    }

    float JoltPrismaticJointComponent::GetPosition() const
    {
        if (auto* constraint = static_cast<JPH::SliderConstraint*>(GetNativeConstraint()))
        {
            return constraint->GetCurrentPosition();
        }
        return 0.0f;
    }

    float JoltPrismaticJointComponent::GetVelocity() const
    {
        // Linear velocity of the follower body projected onto the current world slider axis.
        if (auto* constraint = static_cast<JPH::SliderConstraint*>(GetNativeConstraint()))
        {
            const AZ::Vector3 worldAxis = GetJointWorldTransform().GetRotation().TransformVector(AZ::Vector3::CreateAxisX());
            return constraint->GetBody2()->GetLinearVelocity().Dot(Conversions::ToJolt(worldAxis));
        }
        return 0.0f;
    }

    void JoltPrismaticJointComponent::SetVelocity(float velocity)
    {
        if (auto* constraint = static_cast<JPH::SliderConstraint*>(GetNativeConstraint()))
        {
            constraint->SetMotorState(JPH::EMotorState::Velocity);
            constraint->SetTargetVelocity(velocity);
        }
    }

    void JoltPrismaticJointComponent::SetMaximumForce(float force)
    {
        if (auto* constraint = static_cast<JPH::SliderConstraint*>(GetNativeConstraint()))
        {
            constraint->GetMotorSettings().SetForceLimits(-force, force);
        }
    }

    AZStd::pair<float, float> JoltPrismaticJointComponent::GetLimits() const
    {
        return { m_limitProperties.m_limitFirst, m_limitProperties.m_limitSecond };
    }

    // --- JoltD6JointComponent ---

    void JoltD6JointComponent::Reflect(AZ::ReflectContext* context)
    {
        JoltJointComponentBase::Reflect(context);
        JoltD6JointLimitConfiguration::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltD6JointComponent, JoltJointComponentBase>()
                ->Version(1)
                ->Field("SwingLimitY", &JoltD6JointComponent::m_swingLimitY)
                ->Field("SwingLimitZ", &JoltD6JointComponent::m_swingLimitZ)
                ->Field("TwistLimitLower", &JoltD6JointComponent::m_twistLimitLower)
                ->Field("TwistLimitUpper", &JoltD6JointComponent::m_twistLimitUpper)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltD6JointComponent>("Jolt D6 Joint", "6-DOF joint simulated by the Jolt physics backend")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        // No AppearsInAddComponentMenu: the corresponding editor joint component
                        // owns the menu entry (PhysX-style editor/runtime split). The runtime
                        // component stays registered for old prefabs and BuildGameEntity.
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    AZStd::unique_ptr<AzPhysics::JointConfiguration> JoltD6JointComponent::BuildJointConfiguration() const
    {
        auto configuration = AZStd::make_unique<JoltD6JointLimitConfiguration>();
        configuration->m_swingLimitY = m_swingLimitY;
        configuration->m_swingLimitZ = m_swingLimitZ;
        configuration->m_twistLimitLower = m_twistLimitLower;
        configuration->m_twistLimitUpper = m_twistLimitUpper;
        return configuration;
    }

} // namespace JoltPhysics
