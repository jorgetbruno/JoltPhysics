#include <Editor/JoltPhysicsEditorSystemComponent.h>

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include <AzCore/Interface/Interface.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzToolsFramework/API/ViewPaneOptions.h>

#include <Editor/ConfigurationWindow/JoltConfigurationWidget.h>
#include <Editor/ConfigurationWindow/JoltConfigurationWindowBus.h>
#include <Editor/PropertyHandlers/PropertyTypes.h>

#include <System/JoltSystem.h>

namespace JoltPhysics
{
    void JoltPhysicsEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltPhysicsEditorSystemComponent, AZ::Component>()
                ->Version(1)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltPhysicsEditorSystemComponent>(
                    "Jolt Physics Editor System",
                    "Provides Jolt Physics editor integration")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void JoltPhysicsEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltPhysicsEditorService"));
    }

    void JoltPhysicsEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltPhysicsEditorService"));
        incompatible.push_back(AZ_CRC_CE("PhysXEditorService"));
    }

    void JoltPhysicsEditorSystemComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("JoltPhysicsService"));
    }

    void JoltPhysicsEditorSystemComponent::GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        // The property manager owns the registry RegisterPropertyTypes writes to, so it
        // has to be up first. Dependent rather than required because this same module is
        // aliased as the Builders variant, where there is no property manager and a hard
        // requirement would stop the component activating at all.
        dependent.push_back(AZ_CRC_CE("PropertyManagerService"));
    }

    void JoltPhysicsEditorSystemComponent::Init()
    {
    }

    void JoltPhysicsEditorSystemComponent::Activate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusConnect();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
        Physics::EditorWorldBus::Handler::BusConnect();

        // The edit-mode physics scene ("EditorScene") handed out through
        // EditorWorldBus, mirroring PhysX: a query/body host for editor tools, not a
        // live simulation - nothing ticks it by default.
        if (auto* joltSystem = GetJoltSystem())
        {
            AzPhysics::SceneConfiguration editorWorldConfig = joltSystem->GetDefaultSceneConfiguration();
            editorWorldConfig.m_sceneName = AzPhysics::EditorPhysicsSceneName;
            m_editorSceneHandle = joltSystem->AddScene(editorWorldConfig);
        }
        else
        {
            AZ_WarningOnce("JoltPhysics", false,
                "JoltPhysicsEditorSystemComponent: no physics system active; the editor scene was not created.");
        }

        // The auto-fit behind the Animation Editor's ragdoll joint-limit widget. Editor
        // only, as PhysX splits it: it serves an authoring gesture and has no runtime
        // caller. Registration happens in the Registrar's constructor.
        m_editorJointHelpers = AZStd::make_unique<JoltEditorJointHelpers>();

        Editor::RegisterPropertyTypes();

        // The collision layer/group dropdowns rebuild their entries from the live
        // configuration in ReadValuesIntoGUI, but the inspector only calls that when
        // it refreshes - so a layer renamed in the configuration window would keep its
        // old name in panels that are already open until they were poked some other
        // way. Every configuration edit funnels through UpdateConfiguration, which
        // signals this event.
        m_onConfigurationChangedHandler = AzPhysics::SystemEvents::OnConfigurationChangedEvent::Handler(
            []([[maybe_unused]] const AzPhysics::SystemConfiguration* config)
            {
                AzToolsFramework::ToolsApplicationEvents::Bus::Broadcast(
                    &AzToolsFramework::ToolsApplicationEvents::InvalidatePropertyDisplay,
                    AzToolsFramework::Refresh_AttributesAndValues);
            });
        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            physicsSystem->RegisterSystemConfigurationChangedEvent(m_onConfigurationChangedHandler);
        }
    }

    void JoltPhysicsEditorSystemComponent::Deactivate()
    {
        if (auto* joltSystem = GetJoltSystem(); joltSystem && m_editorSceneHandle != AzPhysics::InvalidSceneHandle)
        {
            joltSystem->RemoveScene(m_editorSceneHandle);
        }
        m_editorSceneHandle = AzPhysics::InvalidSceneHandle;

        m_editorJointHelpers.reset();

        Physics::EditorWorldBus::Handler::BusDisconnect();
        m_onConfigurationChangedHandler.Disconnect();

        AzToolsFramework::UnregisterViewPane(Editor::ConfigurationWindowName);

        Editor::UnregisterPropertyTypes();

        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusDisconnect();
    }

    void JoltPhysicsEditorSystemComponent::NotifyRegisterViews()
    {
        AzToolsFramework::ViewPaneOptions options;
        options.paneRect = QRect(100, 100, 700, 600);
        options.showOnToolsToolbar = false;
        AzToolsFramework::RegisterViewPane<Editor::JoltConfigurationWidget>(
            Editor::ConfigurationWindowName, "Tools", options);
    }

    AzPhysics::SceneHandle JoltPhysicsEditorSystemComponent::GetEditorSceneHandle() const
    {
        return m_editorSceneHandle;
    }

    void JoltPhysicsEditorSystemComponent::OnStartPlayInEditorBegin()
    {
        // PhysX parity: the editor scene sleeps while the game world runs, so
        // edit-mode bodies never leak into play.
        if (auto* joltSystem = GetJoltSystem();
            joltSystem && m_editorSceneHandle != AzPhysics::InvalidSceneHandle)
        {
            if (AzPhysics::Scene* editorScene = joltSystem->GetScene(m_editorSceneHandle))
            {
                editorScene->SetEnabled(false);
            }
        }
    }

    void JoltPhysicsEditorSystemComponent::OnStopPlayInEditor()
    {
        if (auto* joltSystem = GetJoltSystem();
            joltSystem && m_editorSceneHandle != AzPhysics::InvalidSceneHandle)
        {
            if (AzPhysics::Scene* editorScene = joltSystem->GetScene(m_editorSceneHandle))
            {
                editorScene->SetEnabled(true);
            }
        }
    }

} // namespace JoltPhysics
