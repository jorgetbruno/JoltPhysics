#include <Editor/Components/EditorJoltSoftBodyComponent.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Physics/SystemBus.h>

#include <Clients/Components/JoltSoftBodyComponent.h>
#include <SoftBody/JoltSoftBodyRender.h>

namespace JoltPhysics
{
    void EditorJoltSoftBodyComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltSoftBodyComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(2)
                ->Field("Settings", &EditorJoltSoftBodyComponent::m_settings)
                ->Field("Visible", &EditorJoltSoftBodyComponent::m_visible)
                ->Field("LivePreview", &EditorJoltSoftBodyComponent::m_livePreview)
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
                        "Soft body", "Soft body properties")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltSoftBodyComponent::OnConfigurationChanged)
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &EditorJoltSoftBodyComponent::m_visible,
                        "Visible", "Draw the soft body. A soft body has no mesh asset, so this drawing is the only "
                        "way to see it.")
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &EditorJoltSoftBodyComponent::m_livePreview,
                        "Live preview", "Simulate the soft body in the Edit viewport, draping over the editor "
                        "colliders' geometry. Off shows the rest shape instead.")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltSoftBodyComponent::OnConfigurationChanged)
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
        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
        RefreshPreviewBody();
    }

    void EditorJoltSoftBodyComponent::Deactivate()
    {
        DestroyPreviewBody();
        AZ::TransformNotificationBus::Handler::BusDisconnect();
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

    void EditorJoltSoftBodyComponent::SetLivePreviewEnabled(bool enabled)
    {
        m_livePreview = enabled;
        RefreshPreviewBody();
    }

    bool EditorJoltSoftBodyComponent::HasLivePreviewBody() const
    {
        return m_previewBody && m_previewBody->IsAttached();
    }

    AZ::u32 EditorJoltSoftBodyComponent::OnConfigurationChanged()
    {
        RefreshPreviewBody();
        return AZ::Edit::PropertyRefreshLevels::None;
    }

    void EditorJoltSoftBodyComponent::RefreshPreviewBody()
    {
        if (!m_livePreview)
        {
            DestroyPreviewBody();
            return;
        }

        if (m_previewBody)
        {
            // SetSettings sorts live fields from baked ones itself, so a damping tweak
            // keeps the drape while a resolution change rebuilds.
            m_previewBody->SetSettings(m_settings);
            return;
        }

        // A mesh-shaped preview needs the asset's triangles before the body can build.
        if (m_settings.m_shape == JoltSoftBodyShape::Mesh &&
            m_settings.m_meshAsset.GetId().IsValid() && !m_settings.m_meshAsset.IsReady())
        {
            m_settings.m_meshAsset.QueueLoad();
            m_settings.m_meshAsset.BlockUntilLoadComplete();
        }

        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        JoltSoftBodyConfiguration configuration;
        configuration.m_settings = m_settings;
        configuration.m_position = worldTransform.GetTranslation();
        configuration.m_orientation = worldTransform.GetRotation();
        configuration.m_entityId = GetEntityId();
        configuration.m_debugName = GetEntity() ? GetEntity()->GetName() : AZStd::string();

        // The preview simulates in the edit-mode scene next to the editor colliders'
        // static bodies. The scene is stepped by the system's normal tick, so nothing
        // here needs to drive time.
        AzPhysics::SceneHandle editorSceneHandle = AzPhysics::InvalidSceneHandle;
        Physics::EditorWorldBus::BroadcastResult(
            editorSceneHandle, &Physics::EditorWorldRequests::GetEditorSceneHandle);

        m_previewBody = AZStd::make_unique<JoltSoftBody>(configuration);
        if (!m_previewBody->Attach(editorSceneHandle))
        {
            // No editor world to host the preview (headless tools); the rest shape
            // still draws.
            m_previewBody.reset();
        }
    }

    void EditorJoltSoftBodyComponent::DestroyPreviewBody()
    {
        m_previewBody.reset();
    }

    void EditorJoltSoftBodyComponent::OnTransformChanged(const AZ::Transform& /*local*/, const AZ::Transform& world)
    {
        // Rebuilds the preview at the new placement, the same policy as the runtime
        // component: placement, not animation.
        if (m_previewBody)
        {
            m_previewBody->SetTransform(world);
        }
    }

    void EditorJoltSoftBodyComponent::DisplayEntityViewport(
        const AzFramework::ViewportInfo& /*viewportInfo*/, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        if (!m_visible)
        {
            return;
        }

        if (m_previewBody && m_previewBody->CopyVertexPositions(m_previewPositions))
        {
            DrawSoftBody(debugDisplay, m_previewPositions, m_previewBody->GetTriangleIndices());
            return;
        }

        // The rest-shape preview is generated from the procedural settings. A mesh's rest
        // shape is its render mesh, which the viewport already shows, and geometry supplied
        // at runtime does not exist until the level runs - drawing a guess for either would
        // be showing a shape the game will not use.
        if (m_settings.m_shape == JoltSoftBodyShape::Mesh || m_settings.m_shape == JoltSoftBodyShape::Custom)
        {
            return;
        }

        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        DrawSoftBodyPreview(debugDisplay, worldTransform, m_settings.m_shape, m_settings.m_size);
    }
} // namespace JoltPhysics
