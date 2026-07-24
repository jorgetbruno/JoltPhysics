#pragma once

#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

namespace JoltPhysics
{
    //! Editor Jolt Static Rigid Body: edit-time counterpart of
    //! JoltStaticRigidBodyComponent (PhysX-style editor/runtime split). Spawns the
    //! runtime component via BuildGameEntity; static bodies take their shape from
    //! the entity's colliders, so there is no configuration to copy.
    class EditorJoltStaticRigidBodyComponent
        : public AzToolsFramework::Components::EditorComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltStaticRigidBodyComponent, "{F6A7B8C9-D0E1-4234-E5F6-A7B8C9D0E1F2}", AzToolsFramework::Components::EditorComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;
    };
} // namespace JoltPhysics
