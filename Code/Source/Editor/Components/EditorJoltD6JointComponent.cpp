#include <Editor/Components/EditorJoltD6JointComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltJointComponents.h>
#include <Editor/Components/EditorJoltDebugDrawUtils.h>
#include <Joint/JoltJointConfiguration.h>
#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void EditorJoltD6JointComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltJointComponentBase::Reflect(context);
        Internal::ReflectOnce<JoltD6JointLimitConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltD6JointComponent, EditorJoltJointComponentBase>()
                ->Version(1)
                ->Field("SwingLimitY", &EditorJoltD6JointComponent::m_swingLimitY)
                ->Field("SwingLimitZ", &EditorJoltD6JointComponent::m_swingLimitZ)
                ->Field("TwistLimitLower", &EditorJoltD6JointComponent::m_twistLimitLower)
                ->Field("TwistLimitUpper", &EditorJoltD6JointComponent::m_twistLimitUpper)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltD6JointComponent>(
                    "Jolt D6 Joint", "6-DOF joint simulated by the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltD6JointComponent::m_swingLimitY,
                        "Swing limit Y", "Max swing angle from the joint-frame Y axis.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 180.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " deg")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltD6JointComponent::m_swingLimitZ,
                        "Swing limit Z", "Max swing angle from the joint-frame Z axis.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 180.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " deg")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltD6JointComponent::m_twistLimitLower,
                        "Twist limit lower", "Lower twist limit about the joint-frame X axis.")
                        ->Attribute(AZ::Edit::Attributes::Min, -180.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 180.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " deg")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltD6JointComponent::m_twistLimitUpper,
                        "Twist limit upper", "Upper twist limit about the joint-frame X axis.")
                        ->Attribute(AZ::Edit::Attributes::Min, -180.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 180.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " deg")
                    ;
            }
        }
    }

    void EditorJoltD6JointComponent::Activate()
    {
        EditorJoltJointComponentBase::Activate();
        ConnectJointComponentMode<EditorJoltD6JointComponent>();
    }

    void EditorJoltD6JointComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltD6JointComponent>())
        {
            component->GetConfiguration() = m_configuration;
            component->SetD6Limits(m_swingLimitY, m_swingLimitZ, m_twistLimitLower, m_twistLimitUpper);
        }
    }

    void EditorJoltD6JointComponent::DrawJointLimits(
        AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& jointTransform) const
    {
        // The swing limits are named for the axis rotated about, so they cross over
        // when expressed as cone extents: rotating about Y moves the twist axis
        // towards Z, and rotating about Z moves it towards Y.
        EditorDebugDraw::DrawLimitCone(
            debugDisplay, jointTransform, EditorDebugDraw::JointAxisLength, m_swingLimitZ, m_swingLimitY);

        EditorDebugDraw::DrawLimitArc(
            debugDisplay, jointTransform, EditorDebugDraw::JointAxisLength * 0.5f, m_twistLimitLower,
            m_twistLimitUpper);
    }

} // namespace JoltPhysics
