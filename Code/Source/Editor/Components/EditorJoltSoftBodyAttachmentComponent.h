#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/vector.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

#include <Clients/Components/JoltSoftBodyAttachmentComponent.h>

namespace JoltPhysics
{
    //! Editor Jolt Soft Body Attachment: authors which entity the cloth hangs from and
    //! which rigid body carries what it catches, and spawns the runtime component.
    //!
    //! No edit-mode preview on purpose: the attachment welds particles by where they sit
    //! when both bodies exist, and neither exists until the level runs. Drawing a guessed
    //! weld line would show a rig the game will not use.
    class EditorJoltSoftBodyAttachmentComponent
        : public AzToolsFramework::Components::EditorComponentBase
    {
    public:
        AZ_EDITOR_COMPONENT(EditorJoltSoftBodyAttachmentComponent, "{8E4C2A70-5B19-4D86-9F3E-1A6D7C50B2E4}",
            AzToolsFramework::Components::EditorComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        AZStd::vector<JoltSoftBodyAttachTarget> m_targets;
        AZ::EntityId m_pushEntity;
    };
} // namespace JoltPhysics
