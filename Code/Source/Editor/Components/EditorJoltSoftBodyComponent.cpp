#include <Editor/Components/EditorJoltSoftBodyComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltSoftBodyComponent.h>
#include <SoftBody/JoltSoftBodyRender.h>

namespace JoltPhysics
{
    void EditorJoltSoftBodyComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltSoftBodyComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(1)
                ->Field("Settings", &EditorJoltSoftBodyComponent::m_settings)
                ->Field("Visible", &EditorJoltSoftBodyComponent::m_visible)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltSoftBodyComponent>(
                    "Jolt Soft Body", "Deformable cloth or a pressurised body simulated by Jolt")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltSoftBodyComponent::m_settings,
                        "Soft body", "Soft body properties. The viewport shows the rest shape only - soft bodies are "
                        "simulated in game mode.")
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &EditorJoltSoftBodyComponent::m_visible,
                        "Visible", "Draw the soft body. A soft body has no mesh asset, so this drawing is the only "
                        "way to see it.")
                    ;
            }
        }
    }

    void EditorJoltSoftBodyComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltSoftBodyService"));
    }

    void EditorJoltSoftBodyComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltSoftBodyService"));
    }

    void EditorJoltSoftBodyComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void EditorJoltSoftBodyComponent::Activate()
    {
        AzToolsFramework::Components::EditorComponentBase::Activate();
        AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(GetEntityId());
    }

    void EditorJoltSoftBodyComponent::Deactivate()
    {
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();
        AzToolsFramework::Components::EditorComponentBase::Deactivate();
    }

    void EditorJoltSoftBodyComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltSoftBodyComponent>())
        {
            component->GetSettings() = m_settings;
            component->GetVisible() = m_visible;
        }
    }

    void EditorJoltSoftBodyComponent::DisplayEntityViewport(
        const AzFramework::ViewportInfo& /*viewportInfo*/, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        if (!m_visible)
        {
            return;
        }

        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        DrawSoftBodyPreview(debugDisplay, worldTransform, m_settings.m_shape, m_settings.m_size);
    }
} // namespace JoltPhysics
