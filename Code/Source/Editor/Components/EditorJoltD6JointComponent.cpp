#include <Editor/Components/EditorJoltD6JointComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltJointComponents.h>
#include <Joint/JoltJointConfiguration.h>

namespace JoltPhysics
{
    void EditorJoltD6JointComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltJointComponentBase::Reflect(context);
        JoltD6JointLimitConfiguration::Reflect(context);

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
                    ;
            }
        }
    }

    void EditorJoltD6JointComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltD6JointComponent>())
        {
            component->GetConfiguration() = m_configuration;
            component->SetD6Limits(m_swingLimitY, m_swingLimitZ, m_twistLimitLower, m_twistLimitUpper);
        }
    }

} // namespace JoltPhysics
