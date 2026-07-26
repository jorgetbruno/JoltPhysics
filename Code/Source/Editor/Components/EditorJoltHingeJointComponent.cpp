#include <Editor/Components/EditorJoltHingeJointComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltJointComponents.h>
#include <Editor/Components/EditorJoltDebugDrawUtils.h>
#include <Joint/JoltJointConfiguration.h>
#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void EditorJoltHingeJointComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltJointComponentBase::Reflect(context);
        Internal::ReflectOnce<JoltHingeJointConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltHingeJointComponent, EditorJoltJointComponentBase>()
                ->Version(1)
                ->Field("GenericProperties", &EditorJoltHingeJointComponent::m_genericProperties)
                ->Field("LimitProperties", &EditorJoltHingeJointComponent::m_limitProperties)
                ->Field("MotorProperties", &EditorJoltHingeJointComponent::m_motorProperties)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltHingeJointComponent>(
                    "Jolt Hinge Joint", "Hinge joint simulated by the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltHingeJointComponent::m_genericProperties,
                        "Generic properties", "Break force/torque and generic joint flags.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltHingeJointComponent::m_limitProperties,
                        "Angular limit", "Rotation limit about the hinge (joint-frame X) axis, in degrees.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltHingeJointComponent::m_motorProperties,
                        "Motor", "Optional motor that actuates the hinge.")
                    ;
            }
        }
    }

    void EditorJoltHingeJointComponent::Activate()
    {
        EditorJoltJointComponentBase::Activate();
        ConnectJointComponentMode<EditorJoltHingeJointComponent>();
    }

    void EditorJoltHingeJointComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltHingeJointComponent>())
        {
            component->GetConfiguration() = m_configuration;
            component->GetGenericProperties() = m_genericProperties;
            component->GetLimitProperties() = m_limitProperties;
            component->GetMotorProperties() = m_motorProperties;
        }
    }

    void EditorJoltHingeJointComponent::DrawJointLimits(
        AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& jointTransform) const
    {
        if (!m_limitProperties.m_isLimited)
        {
            return;
        }
        // The hinge turns about the frame's X axis, with angles measured off +Y.
        EditorDebugDraw::DrawLimitArc(
            debugDisplay, jointTransform, EditorDebugDraw::JointAxisLength, m_limitProperties.m_limitFirst,
            m_limitProperties.m_limitSecond);
    }

} // namespace JoltPhysics
