#include <Editor/Components/EditorJoltStaticRigidBodyComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltStaticRigidBodyComponent.h>

namespace JoltPhysics
{
    void EditorJoltStaticRigidBodyComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltStaticRigidBodyComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(1)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltStaticRigidBodyComponent>(
                    "Jolt Static Rigid Body", "Static (non-moving) rigid body for the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void EditorJoltStaticRigidBodyComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltStaticRigidBodyService"));
    }

    void EditorJoltStaticRigidBodyComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltStaticRigidBodyService"));
        incompatible.push_back(AZ_CRC_CE("JoltRigidBodyService"));
    }

    void EditorJoltStaticRigidBodyComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        required.push_back(AZ_CRC_CE("JoltColliderService"));
    }

    void EditorJoltStaticRigidBodyComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<JoltStaticRigidBodyComponent>();
    }

} // namespace JoltPhysics
