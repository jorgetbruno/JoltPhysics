#pragma once

#include <AzCore/Component/Component.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzToolsFramework/Entity/EditorEntityContextBus.h>

namespace JoltPhysics
{
    class JoltPhysicsEditorSystemComponent
        : public AZ::Component
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

    protected:
        void Init() override;
        void Activate() override;
        void Deactivate() override;

        void OnStartPlayInEditorBegin() override;
        void OnStopPlayInEditor() override;

        // AzToolsFramework::EditorEvents
        void NotifyRegisterViews() override;
    };

} // namespace JoltPhysics
