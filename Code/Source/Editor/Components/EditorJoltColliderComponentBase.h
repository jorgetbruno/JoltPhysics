#pragma once

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
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
        //! component has no bounds to offer. The heightfield, mesh and compound colliders
        //! leave it at that: their geometry is not cheaply available here, which is the
        //! same reason they are not drawn in the viewport.
        virtual AZ::Aabb GetLocalShapeBounds() const
        {
            return AZ::Aabb::CreateNull();
        }

        //! Called after a manipulator changes the shape, so the derived component can push
        //! the new value into the property grid. Refreshing values rather than the whole
        //! tree keeps the manipulators alive while they are being dragged.
        void OnShapeChangedByManipulator();

        //! The collider frame in entity space: where the shape sits relative to the entity.
        AZ::Transform GetColliderLocalTransform() const;

        //! The collider frame in world space.
        AZ::Transform GetColliderWorldTransform() const;

        Physics::ColliderConfiguration& GetColliderConfiguration()
        {
            return *m_colliderConfiguration;
        }
        const Physics::ColliderConfiguration& GetColliderConfiguration() const
        {
            return *m_colliderConfiguration;
        }

        AZStd::shared_ptr<Physics::ColliderConfiguration> m_colliderConfiguration =
            AZStd::make_shared<Physics::ColliderConfiguration>();

        //! Puts the Edit button on the component in the inspector and handles entering
        //! component mode. Derived classes connect it with their own component mode type.
        AzToolsFramework::ComponentModeFramework::ComponentModeDelegate m_componentModeDelegate;
    };
} // namespace JoltPhysics
