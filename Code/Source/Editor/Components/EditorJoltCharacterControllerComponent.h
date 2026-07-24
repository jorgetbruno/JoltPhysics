#pragma once

#include <AzFramework/Physics/Character.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

namespace JoltPhysics
{
    //! Editor Jolt Character Controller: edit-time counterpart of
    //! JoltCharacterControllerComponent. Spawns the runtime component via
    //! BuildGameEntity, copying the character and shape configurations.
    class EditorJoltCharacterControllerComponent
        : public AzToolsFramework::Components::EditorComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltCharacterControllerComponent, "{C9D0E1F2-A3B4-4567-B8C9-D0E1F2A3B4C5}", AzToolsFramework::Components::EditorComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        Physics::CharacterConfiguration m_characterConfig;
        AZStd::shared_ptr<Physics::ShapeConfiguration> m_shapeConfig;

        // Capsule dimensions (see JoltCharacterControllerComponent); defaults match the
        // runtime fallback capsule of 1.8 m height / 0.3 m radius.
        float m_height = 1.8f;
        float m_radius = 0.3f;
    };
} // namespace JoltPhysics
