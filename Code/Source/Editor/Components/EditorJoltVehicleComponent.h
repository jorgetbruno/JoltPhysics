#pragma once

#include <Vehicle/JoltVehicleConfiguration.h>

#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

namespace JoltPhysics
{
    //! Editor Jolt Vehicle: edit-time counterpart of JoltVehicleComponent.
    //! Spawns the runtime component via BuildGameEntity, copying the vehicle
    //! configuration.
    class EditorJoltVehicleComponent
        : public AzToolsFramework::Components::EditorComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltVehicleComponent, "{D0E1F2A3-B4C5-4678-C9D0-E1F2A3B4C5D6}", AzToolsFramework::Components::EditorComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        JoltVehicleConfiguration m_configuration;
    };
} // namespace JoltPhysics
