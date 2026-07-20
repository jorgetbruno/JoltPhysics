#include <Editor/JoltPhysicsEditorSystemComponent.h>

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

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

    void JoltPhysicsEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    void JoltPhysicsEditorSystemComponent::Init()
    {
    }

    void JoltPhysicsEditorSystemComponent::Activate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusConnect();
    }

    void JoltPhysicsEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusDisconnect();
    }

    void JoltPhysicsEditorSystemComponent::OnStartPlayInEditorBegin()
    {
        // Editor play mode starting
    }

    void JoltPhysicsEditorSystemComponent::OnStopPlayInEditor()
    {
        // Editor play mode stopped
    }

} // namespace JoltPhysics
