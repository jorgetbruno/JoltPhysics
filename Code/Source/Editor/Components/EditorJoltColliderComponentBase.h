#pragma once

#include <AzCore/Component/NonUniformScaleBus.h>
#include <AzCore/Component/TransformBus.h>

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <AzFramework/Physics/Shape.h>

#include <AzToolsFramework/API/ComponentEntitySelectionBus.h>
#include <AzToolsFramework/ComponentMode/ComponentModeDelegate.h>
#include <AzToolsFramework/Manipulators/ShapeManipulatorRequestBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

namespace JoltPhysics
{
    //! Editor-side base for Jolt collider components (mirrors the PhysX gem's
    //! editor/runtime component split). Derives from EditorComponentBase so it
    //! activates in the Edit viewport (unlike the plain AZ::Component runtime
    //! colliders, which only activate in the Game Entity Context), draws the
    //! collider shape there, and spawns the runtime collider via BuildGameEntity.
    //!
    //! It also carries the parts of viewport editing that every collider shares: the
    //! collider's translation/rotation offset, the selection bounds the editor picks
    //! against, and the ComponentModeDelegate that puts an "Edit" button on the
    //! component and enters component mode on double click. Each derived collider adds
    //! the manipulator bus for its own dimensions and names the component mode to use.
    class EditorJoltColliderComponentBase
        : public AzToolsFramework::Components::EditorComponentBase
        , private AzFramework::EntityDebugDisplayEventBus::Handler
        , private AZ::TransformNotificationBus::Handler
        , protected AzToolsFramework::ShapeManipulatorRequestBus::Handler
        , protected AzToolsFramework::EditorComponentSelectionRequestsBus::Handler
    {
    public:
        AZ_RTTI(EditorJoltColliderComponentBase, "{A1B2C3D4-E5F6-4789-90AB-CDEF01234567}", AzToolsFramework::Components::EditorComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

    protected:
        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // AzFramework::EntityDebugDisplayEvents
        void DisplayEntityViewport(
            const AzFramework::ViewportInfo& viewportInfo,
            AzFramework::DebugDisplayRequests& debugDisplay) override;

        // AzToolsFramework::ShapeManipulatorRequestBus - the collider's offset from the
        // entity, which is the one manipulator every collider shape has in common.
        AZ::Vector3 GetTranslationOffset() const override;
        void SetTranslationOffset(const AZ::Vector3& translationOffset) override;
        AZ::Transform GetManipulatorSpace() const override;
        AZ::Quaternion GetRotationOffset() const override;

        // AzToolsFramework::EditorComponentSelectionRequestsBus - required by the
        // ComponentModeDelegate, and what the editor picks against in the viewport.
        AZ::Aabb GetEditorSelectionBoundsViewport(const AzFramework::ViewportInfo& viewportInfo) override;
        bool SupportsEditorRayIntersect() override;

        //! Derived classes draw their shape in world space.
        virtual void DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const = 0;

        //! Bounds of the shape alone, centred on the collider's own frame. The base class
        //! places and orients it for selection, so this needs no entity transform.
        //!
        //! Defaults to a null Aabb, which is what the selection bus itself answers when a
        //! component has no bounds to offer. The compound colliders leave it at that:
        //! they have no geometry of their own, only their child entities' colliders,
        //! which draw and are picked as the separate entities they are.
        virtual AZ::Aabb GetLocalShapeBounds() const
        {
            return AZ::Aabb::CreateNull();
        }

        //! Called after a manipulator changes the shape, so the derived component can push
        //! the new value into the property grid. Refreshing values rather than the whole
        //! tree keeps the manipulators alive while they are being dragged.
        void OnShapeChangedByManipulator();

        // AZ::TransformNotificationBus - keeps the edit-mode body under the entity.
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        //! Collider/shape pairs for this component's static body in the editor scene
        //! (EditorWorldBus), with the entity's overall scale already applied - the same
        //! contract as the runtime GetShapeColliderPairs. The default is empty, which
        //! means no editor body: the compound colliders stay with it (their children are
        //! separate entities with colliders of their own), and so does the heightfield
        //! (its geometry lives with the terrain provider).
        virtual AzPhysics::ShapeColliderPairList GetEditorShapeColliderPairs() const
        {
            return {};
        }

        //! Wraps a copy of a derived component's shape configuration into a pair with
        //! this component's collider configuration, applying the entity's overall scale
        //! (world uniform scale x NonUniformScale) the way the runtime colliders do.
        AzPhysics::ShapeColliderPair MakeScaledEditorPair(
            AZStd::shared_ptr<Physics::ShapeConfiguration> shapeConfiguration) const;

        //! Recreates this collider's static body in the editor scene so editor-time
        //! physics queries hit what the viewport shows (PhysX's CreateStaticEditorCollider
        //! equivalent). Safe to call at any time; without an editor scene it just removes.
        void RebuildEditorCollider();
        void DestroyEditorCollider();

        //! ChangeNotify hook for the inspector: any collider or shape property edit
        //! rebuilds the editor body.
        AZ::u32 OnColliderConfigurationChangedInEditor();

        //! The collider frame in entity space: where the shape sits relative to the entity.
        AZ::Transform GetColliderLocalTransform() const;

        //! The collider frame in world space.
        AZ::Transform GetColliderWorldTransform() const;

        Physics::ColliderConfiguration& GetColliderConfiguration()
        {
            return m_colliderConfiguration;
        }
        const Physics::ColliderConfiguration& GetColliderConfiguration() const
        {
            return m_colliderConfiguration;
        }

        //! By value, not behind a shared_ptr, deliberately: the inspector then shows
        //! the fields directly instead of a one-element pointer container, and property
        //! writes hit the same layout the character controller and PhysX's editor
        //! components use - which is the layout the collision layer/group property
        //! handlers are proven against.
        Physics::ColliderConfiguration m_colliderConfiguration;

        //! Puts the Edit button on the component in the inspector and handles entering
        //! component mode. Derived classes connect it with their own component mode type.
        AzToolsFramework::ComponentModeFramework::ComponentModeDelegate m_componentModeDelegate;

    private:
        //! This collider's static body in the editor scene, and the scene it was added
        //! to (kept so removal survives the scene itself having gone first).
        AzPhysics::SimulatedBodyHandle m_editorBodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        AzPhysics::SceneHandle m_editorBodySceneHandle = AzPhysics::InvalidSceneHandle;

        //! The overall scale the editor body was last built with. A transform change
        //! that keeps it moves the body (cheap); one that changes it rebuilds the shape,
        //! which is what actually depends on scale.
        AZ::Vector3 m_editorBodyBuiltScale = AZ::Vector3::CreateOne();

        AZ::NonUniformScaleChangedEvent::Handler m_nonUniformScaleChangedHandler;
    };
} // namespace JoltPhysics
