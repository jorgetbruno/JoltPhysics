#include <Editor/Components/EditorJoltVehicleComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltVehicleComponent.h>

namespace JoltPhysics
{
    void EditorJoltVehicleComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltVehicleComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(1)
                ->Field("VehicleConfiguration", &EditorJoltVehicleComponent::m_configuration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                // The JoltVehicleConfiguration field-level edit context is registered by the
                // runtime JoltVehicleComponent::Reflect, which also runs in this dll.
                editContext->Class<EditorJoltVehicleComponent>(
                    "Jolt Vehicle", "Four-wheeled vehicle simulated by the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltVehicleComponent::m_configuration,
                        "Vehicle Configuration", "Vehicle chassis, wheel and controller settings")
                    ;
            }
        }
    }

    void EditorJoltVehicleComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltVehicleService"));
    }

    void EditorJoltVehicleComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltVehicleService"));
    }

    void EditorJoltVehicleComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        required.push_back(AZ_CRC_CE("JoltRigidBodyService"));
    }

    void EditorJoltVehicleComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltVehicleComponent>())
        {
            component->GetConfiguration() = m_configuration;
        }
    }

} // namespace JoltPhysics
