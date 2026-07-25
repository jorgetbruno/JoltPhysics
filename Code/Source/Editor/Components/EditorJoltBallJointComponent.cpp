#include <Editor/Components/EditorJoltBallJointComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltJointComponents.h>
#include <Editor/Components/EditorJoltDebugDrawUtils.h>
#include <Joint/JoltJointConfiguration.h>

namespace JoltPhysics
{
    void EditorJoltBallJointComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltJointComponentBase::Reflect(context);
        JoltBallJointConfiguration::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltBallJointComponent, EditorJoltJointComponentBase>()
                ->Version(1)
                ->Field("GenericProperties", &EditorJoltBallJointComponent::m_genericProperties)
                ->Field("LimitProperties", &EditorJoltBallJointComponent::m_limitProperties)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltBallJointComponent>(
                    "Jolt Ball Joint", "Ball-and-socket joint simulated by the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltBallJointComponent::m_genericProperties,
                        "Generic properties", "Break force/torque and generic joint flags.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltBallJointComponent::m_limitProperties,
                        "Swing limit", "Swing cone half-angles about the joint-frame Y (lower) and Z (upper) axes, in degrees.")
                    ;
            }
        }
    }

    void EditorJoltBallJointComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltBallJointComponent>())
        {
            component->GetConfiguration() = m_configuration;
            component->GetGenericProperties() = m_genericProperties;
            component->GetLimitProperties() = m_limitProperties;
        }
    }

    void EditorJoltBallJointComponent::DrawJointLimits(
        AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& jointTransform) const
    {
        if (!m_limitProperties.m_isLimited)
        {
            return;
        }
        // For a ball joint the two limits are cone half-angles about the frame's
        // Y and Z axes (see JointLimitProperties), which is exactly what the cone
        // helper takes.
        EditorDebugDraw::DrawLimitCone(
            debugDisplay, jointTransform, EditorDebugDraw::JointAxisLength, m_limitProperties.m_limitFirst,
            m_limitProperties.m_limitSecond);
    }

} // namespace JoltPhysics
