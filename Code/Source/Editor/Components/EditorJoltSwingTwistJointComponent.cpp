#include <Editor/Components/EditorJoltSwingTwistJointComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltJointComponents.h>
#include <Editor/Components/EditorJoltDebugDrawUtils.h>
#include <Joint/JoltJointConfiguration.h>
#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void EditorJoltSwingTwistJointComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltJointComponentBase::Reflect(context);
        Internal::ReflectOnce<JoltSwingTwistJointConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltSwingTwistJointComponent, EditorJoltJointComponentBase>()
                ->Version(1)
                ->Field("GenericProperties", &EditorJoltSwingTwistJointComponent::m_genericProperties)
                ->Field("NormalHalfConeAngle", &EditorJoltSwingTwistJointComponent::m_normalHalfConeAngle)
                ->Field("PlaneHalfConeAngle", &EditorJoltSwingTwistJointComponent::m_planeHalfConeAngle)
                ->Field("TwistLower", &EditorJoltSwingTwistJointComponent::m_twistLower)
                ->Field("TwistUpper", &EditorJoltSwingTwistJointComponent::m_twistUpper)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltSwingTwistJointComponent>(
                    "Jolt Swing-Twist Joint", "Swing-twist joint simulated by the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltSwingTwistJointComponent::m_genericProperties,
                        "Generic properties", "Break force/torque and generic joint flags.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltSwingTwistJointComponent::m_normalHalfConeAngle,
                        "Swing half-angle Y", "Swing half-cone angle about the joint-frame Y (normal) axis.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 180.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " deg")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltSwingTwistJointComponent::m_planeHalfConeAngle,
                        "Swing half-angle Z", "Swing half-cone angle about the joint-frame Z (plane) axis.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 180.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " deg")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltSwingTwistJointComponent::m_twistLower,
                        "Twist lower", "Lower twist limit about the joint-frame X axis.")
                        ->Attribute(AZ::Edit::Attributes::Min, -180.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 180.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " deg")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltSwingTwistJointComponent::m_twistUpper,
                        "Twist upper", "Upper twist limit about the joint-frame X axis.")
                        ->Attribute(AZ::Edit::Attributes::Min, -180.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 180.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " deg")
                    ;
            }
        }
    }

    void EditorJoltSwingTwistJointComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltSwingTwistJointComponent>())
        {
            component->GetConfiguration() = m_configuration;
            component->GetGenericProperties() = m_genericProperties;
            component->SetSwingTwistLimits(m_normalHalfConeAngle, m_planeHalfConeAngle, m_twistLower, m_twistUpper);
        }
    }

    void EditorJoltSwingTwistJointComponent::DrawJointLimits(
        AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& jointTransform) const
    {
        // Twist axis is X and plane axis Y, so the normal axis is Z. A swing about
        // the normal axis (Z) tilts the twist axis towards Y, and a swing about the
        // plane axis (Y) tilts it towards Z -- hence the pairing below.
        EditorDebugDraw::DrawLimitCone(
            debugDisplay, jointTransform, EditorDebugDraw::JointAxisLength, m_normalHalfConeAngle,
            m_planeHalfConeAngle);

        EditorDebugDraw::DrawLimitArc(
            debugDisplay, jointTransform, EditorDebugDraw::JointAxisLength * 0.5f, m_twistLower, m_twistUpper);
    }

} // namespace JoltPhysics
