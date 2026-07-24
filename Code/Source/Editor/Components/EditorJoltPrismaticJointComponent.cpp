#include <Editor/Components/EditorJoltPrismaticJointComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltJointComponents.h>
#include <Joint/JoltJointConfiguration.h>

namespace JoltPhysics
{
    void EditorJoltPrismaticJointComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltJointComponentBase::Reflect(context);
        JoltPrismaticJointConfiguration::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltPrismaticJointComponent, EditorJoltJointComponentBase>()
                ->Version(1)
                ->Field("GenericProperties", &EditorJoltPrismaticJointComponent::m_genericProperties)
                ->Field("LimitProperties", &EditorJoltPrismaticJointComponent::m_limitProperties)
                ->Field("MotorProperties", &EditorJoltPrismaticJointComponent::m_motorProperties)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltPrismaticJointComponent>(
                    "Jolt Prismatic Joint", "Prismatic joint simulated by the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltPrismaticJointComponent::m_genericProperties,
                        "Generic properties", "Break force/torque and generic joint flags.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltPrismaticJointComponent::m_limitProperties,
                        "Slide limit", "Travel limit along the slide (joint-frame X) axis, in meters.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltPrismaticJointComponent::m_motorProperties,
                        "Motor", "Optional motor that actuates the slide.")
                    ;
            }
        }
    }

    void EditorJoltPrismaticJointComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltPrismaticJointComponent>())
        {
            component->GetConfiguration() = m_configuration;
            component->GetGenericProperties() = m_genericProperties;
            component->GetLimitProperties() = m_limitProperties;
            component->GetMotorProperties() = m_motorProperties;
        }
    }

} // namespace JoltPhysics
