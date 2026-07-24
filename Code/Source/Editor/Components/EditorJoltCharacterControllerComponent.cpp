#include <Editor/Components/EditorJoltCharacterControllerComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltCharacterControllerComponent.h>

namespace JoltPhysics
{
    void EditorJoltCharacterControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->RegisterGenericType<AZStd::shared_ptr<Physics::ShapeConfiguration>>();

            serializeContext->Class<EditorJoltCharacterControllerComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(1)
                ->Field("CharacterConfiguration", &EditorJoltCharacterControllerComponent::m_characterConfig)
                ->Field("ShapeConfiguration", &EditorJoltCharacterControllerComponent::m_shapeConfig)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                // The Physics::CharacterConfiguration field-level edit context comes from
                // AzFramework; only the wrapper element is added here (mirrors the runtime
                // component, which also exposes just the character configuration).
                editContext->Class<EditorJoltCharacterControllerComponent>(
                    "Jolt Character Controller", "Character controller simulated by the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltCharacterControllerComponent::m_characterConfig,
                        "Character Configuration", "Configuration of the character controller")
                    ;
            }
        }
    }

    void EditorJoltCharacterControllerComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("PhysicsWorldBodyService"));
        // A character controller acts as a dynamic kinematic rigid body.
        provided.push_back(AZ_CRC_CE("PhysicsRigidBodyService"));
        provided.push_back(AZ_CRC_CE("PhysicsCharacterControllerService"));
    }

    void EditorJoltCharacterControllerComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("PhysicsRigidBodyService"));
        incompatible.push_back(AZ_CRC_CE("PhysicsCharacterControllerService"));
        incompatible.push_back(AZ_CRC_CE("NonUniformScaleService"));
    }

    void EditorJoltCharacterControllerComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void EditorJoltCharacterControllerComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltCharacterControllerComponent>())
        {
            component->GetCharacterConfiguration() = m_characterConfig;
            component->GetShapeConfiguration() = m_shapeConfig;
        }
    }

} // namespace JoltPhysics
