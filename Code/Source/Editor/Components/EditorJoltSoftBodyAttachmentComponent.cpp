#include <Editor/Components/EditorJoltSoftBodyAttachmentComponent.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltSoftBodyAttachmentComponent.h>

namespace JoltPhysics
{
    void EditorJoltSoftBodyAttachmentComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext
                ->Class<EditorJoltSoftBodyAttachmentComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(1)
                ->Field("Targets", &EditorJoltSoftBodyAttachmentComponent::m_targets)
                ->Field("PushEntity", &EditorJoltSoftBodyAttachmentComponent::m_pushEntity)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltSoftBodyAttachmentComponent>(
                    "Jolt Soft Body Attachment",
                    "Fastens this entity's soft body to other entities, so cloth can hang off things that move")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltSoftBodyAttachmentComponent::m_targets,
                        "Fastenings", "Where the cloth is held. Cloth held in only one place flogs - a sail "
                        "wants its head and its foot.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltSoftBodyAttachmentComponent::m_pushEntity,
                        "Push", "Optional: the rigid body that carries what the cloth catches. For a sail, "
                        "the hull - the wind force on the canvas drives this body.")
                    ;
            }
        }
    }

    void EditorJoltSoftBodyAttachmentComponent::GetProvidedServices(
        AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltSoftBodyAttachmentService"));
    }

    void EditorJoltSoftBodyAttachmentComponent::GetIncompatibleServices(
        AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltSoftBodyAttachmentService"));
    }

    void EditorJoltSoftBodyAttachmentComponent::GetRequiredServices(
        AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("JoltSoftBodyService"));
    }

    void EditorJoltSoftBodyAttachmentComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltSoftBodyAttachmentComponent>())
        {
            component->GetTargets() = m_targets;
            component->GetPushEntity() = m_pushEntity;
        }
    }
} // namespace JoltPhysics
