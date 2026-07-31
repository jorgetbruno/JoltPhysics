#pragma once

#include <Editor/Components/EditorJoltColliderComponentBase.h>

#include <AzFramework/Physics/ShapeConfiguration.h>

#include <AzToolsFramework/Manipulators/BoxManipulatorRequestBus.h>

namespace JoltPhysics
{
    //! Editor Box Collider: edit-viewport box collider for the Jolt backend.
    //! Draws the box in the Edit viewport and spawns the runtime JoltBoxColliderComponent
    //! via BuildGameEntity (mirrors PhysX's EditorColliderComponent flow).
    class EditorJoltBoxColliderComponent
        : public EditorJoltColliderComponentBase
        , private AzToolsFramework::BoxManipulatorRequestBus::Handler
    {
    public:
        AZ_COMPONENT(EditorJoltBoxColliderComponent, "{B2C3D4E5-F6A7-4890-A1B2-C3D4E5F6A7B8}", EditorJoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    protected:
        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // EditorJoltColliderComponentBase
        void DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const override;
        AZ::Aabb GetLocalShapeBounds() const override;
        AzPhysics::ShapeColliderPairList GetEditorShapeColliderPairs() const override;

        // AzToolsFramework::BoxManipulatorRequestBus - drives AzToolsFramework's own
        // BoxComponentMode, so the box gets the same handles as an engine box shape.
        AZ::Vector3 GetDimensions() const override;
        void SetDimensions(const AZ::Vector3& dimensions) override;
        AZ::Transform GetCurrentLocalTransform() const override;

    private:
        Physics::BoxShapeConfiguration m_shapeConfiguration;
    };
} // namespace JoltPhysics
