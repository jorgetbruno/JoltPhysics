#pragma once

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Physics/Character.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

#include <AzToolsFramework/API/ComponentEntitySelectionBus.h>
#include <AzToolsFramework/ComponentMode/ComponentModeDelegate.h>
#include <AzToolsFramework/Manipulators/CapsuleManipulatorRequestBus.h>
#include <AzToolsFramework/Manipulators/RadiusManipulatorRequestBus.h>
#include <AzToolsFramework/Manipulators/ShapeManipulatorRequestBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

namespace JoltPhysics
{
    //! Editor Jolt Character Controller: edit-time counterpart of
    //! JoltCharacterControllerComponent. Spawns the runtime component via
    //! BuildGameEntity, copying the character and shape configurations, previews
    //! the capsule in the viewport, and edits it through AzToolsFramework's
    //! CapsuleComponentMode - the same mode the capsule collider uses.
    class EditorJoltCharacterControllerComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , private AzFramework::EntityDebugDisplayEventBus::Handler
        , private AzToolsFramework::CapsuleManipulatorRequestBus::Handler
        , private AzToolsFramework::RadiusManipulatorRequestBus::Handler
        , private AzToolsFramework::ShapeManipulatorRequestBus::Handler
        , private AzToolsFramework::EditorComponentSelectionRequestsBus::Handler
    {
    public:
        AZ_COMPONENT(EditorJoltCharacterControllerComponent, "{C9D0E1F2-A3B4-4567-B8C9-D0E1F2A3B4C5}", AzToolsFramework::Components::EditorComponentBase);

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

        // AzToolsFramework::CapsuleManipulatorRequestBus
        float GetHeight() const override;
        void SetHeight(float height) override;

        // AzToolsFramework::RadiusManipulatorRequestBus
        float GetRadius() const override;
        void SetRadius(float radius) override;

        // AzToolsFramework::ShapeManipulatorRequestBus - the character capsule is centred
        // on the entity and has no offset of its own, so these answer the identity and
        // SetTranslationOffset is deliberately inert. The mode still needs them to place
        // its manipulators.
        AZ::Vector3 GetTranslationOffset() const override;
        void SetTranslationOffset(const AZ::Vector3& translationOffset) override;
        AZ::Transform GetManipulatorSpace() const override;
        AZ::Quaternion GetRotationOffset() const override;

        // AzToolsFramework::EditorComponentSelectionRequestsBus - required by the
        // ComponentModeDelegate, and what the editor picks against in the viewport.
        AZ::Aabb GetEditorSelectionBoundsViewport(const AzFramework::ViewportInfo& viewportInfo) override;
        bool SupportsEditorRayIntersect() override;

        //! The capsule the runtime will actually build. An explicitly assigned
        //! m_shapeConfig wins over Height/Radius exactly as in
        //! JoltCharacterControllerComponent::CreateCharacter; returns false when some
        //! other shape drives the character, in which case there is no capsule to draw,
        //! pick against or edit.
        bool TryGetEffectiveCapsule(float& height, float& radius) const;

        //! Pushes a manipulator-driven change back into the property grid. Values only,
        //! not the whole tree, so the manipulators survive the drag.
        void OnShapeChangedByManipulator();

        Physics::CharacterConfiguration m_characterConfig;
        AZStd::shared_ptr<Physics::ShapeConfiguration> m_shapeConfig;

        // Capsule dimensions (see JoltCharacterControllerComponent); defaults match the
        // runtime fallback capsule of 1.8 m height / 0.3 m radius.
        float m_height = 1.8f;
        float m_radius = 0.3f;

        //! false = virtual character (default); true = rigid-body character.
        bool m_rigidBodyCharacter = false;

        //! How much of the scene gravity the character feels; carried into the runtime
        //! component by BuildGameEntity.
        float m_gravityMultiplier = 1.0f;

        //! Puts the Edit button on the component and enters component mode on double click.
        AzToolsFramework::ComponentModeFramework::ComponentModeDelegate m_componentModeDelegate;
    };
} // namespace JoltPhysics
