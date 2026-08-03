#pragma once

#include <AzCore/Component/Component.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/SystemBus.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzToolsFramework/Entity/EditorEntityContextBus.h>

#include <Editor/JoltEditorJointHelpers.h>

namespace JoltPhysics
{
    class JoltPhysicsEditorSystemComponent
        : public AZ::Component
        , public Physics::EditorWorldBus::Handler
        , private AzToolsFramework::EditorEntityContextNotificationBus::Handler
        , private AzToolsFramework::EditorEvents::Bus::Handler
    {
    public:
        AZ_COMPONENT(JoltPhysicsEditorSystemComponent, "{4E5F6A7B-8C9D-0E1F-2A3B-4C5D6E7F8A9B}");
        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        JoltPhysicsEditorSystemComponent() = default;
        ~JoltPhysicsEditorSystemComponent() override = default;

        // Physics::EditorWorldBus
        AzPhysics::SceneHandle GetEditorSceneHandle() const override;

    protected:
        void Init() override;
        void Activate() override;
        void Deactivate() override;

        void OnStartPlayInEditorBegin() override;
        void OnStopPlayInEditor() override;

        // AzToolsFramework::EditorEvents
        void NotifyRegisterViews() override;

    private:
        //! Refreshes open property grids when the physics configuration changes: the
        //! collision layer/group dropdowns list names from the live configuration, so
        //! a rename in the configuration window must reach panels already showing them.
        AzPhysics::SystemEvents::OnConfigurationChangedEvent::Handler m_onConfigurationChangedHandler;

        //! The edit-mode physics scene ("EditorScene") handed out through
        //! EditorWorldBus; disabled while the game world runs during play-in-editor.
        AzPhysics::SceneHandle m_editorSceneHandle = AzPhysics::InvalidSceneHandle;

        //! Answers AzPhysics::EditorJointHelpersInterface - the joint-limit auto-fit the
        //! Animation Editor offers. Owned here because nothing else in a Jolt project
        //! would register it.
        AZStd::unique_ptr<JoltEditorJointHelpers> m_editorJointHelpers;
    };

} // namespace JoltPhysics
