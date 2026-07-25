#pragma once

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>

#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

namespace JoltPhysics
{
    //! Editor Jolt Rigid Body: edit-time counterpart of JoltRigidBodyComponent
    //! (PhysX-style editor/runtime split). Spawns the runtime component via
    //! BuildGameEntity, copying the rigid body configuration.
    //!
    //! The body's shape is drawn by whichever collider components sit on the
    //! entity; what this component draws is the manually placed centre of mass,
    //! which is otherwise invisible.
    class EditorJoltRigidBodyComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , private AzFramework::EntityDebugDisplayEventBus::Handler
    {
    public:
        AZ_COMPONENT(EditorJoltRigidBodyComponent, "{E5F6A7B8-C9D0-4123-D4E5-F6A7B8C9D0E1}", AzToolsFramework::Components::EditorComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // EditorComponentBase
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        // AzFramework::EntityDebugDisplayEvents
        void DisplayEntityViewport(
            const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

        AzPhysics::RigidBodyConfiguration m_configuration;
    };
} // namespace JoltPhysics
