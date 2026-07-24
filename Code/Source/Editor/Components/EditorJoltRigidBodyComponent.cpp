#include <Editor/Components/EditorJoltRigidBodyComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltRigidBodyComponent.h>

namespace JoltPhysics
{
    void EditorJoltRigidBodyComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltRigidBodyComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(1)
                ->Field("RigidBodyConfiguration", &EditorJoltRigidBodyComponent::m_configuration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                // The per-field edit context for AzPhysics::RigidBodyConfiguration is
                // registered by the runtime JoltRigidBodyComponent::Reflect, which also
                // runs in the editor module (same dll); only the wrapper element is added here.
                editContext->Class<EditorJoltRigidBodyComponent>(
                    "Jolt Rigid Body", "Dynamic rigid body simulated by the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltRigidBodyComponent::m_configuration,
                        "Configuration", "Rigid body configuration")
                    ;
            }
        }
    }

    void EditorJoltRigidBodyComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltRigidBodyService"));
    }

    void EditorJoltRigidBodyComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltRigidBodyService"));
        incompatible.push_back(AZ_CRC_CE("JoltStaticRigidBodyService"));
    }

    void EditorJoltRigidBodyComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        required.push_back(AZ_CRC_CE("JoltColliderService"));
    }

    void EditorJoltRigidBodyComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltRigidBodyComponent>())
        {
            component->GetConfiguration() = m_configuration;
        }
    }

} // namespace JoltPhysics
