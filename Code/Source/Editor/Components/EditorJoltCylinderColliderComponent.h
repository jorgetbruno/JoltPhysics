#pragma once

#include <Editor/Components/EditorJoltColliderComponentBase.h>

#include <AzToolsFramework/Manipulators/CylinderManipulatorRequestBus.h>
#include <AzToolsFramework/Manipulators/RadiusManipulatorRequestBus.h>

#include <Shape/JoltCylinderShapeConfiguration.h>

namespace JoltPhysics
{
    //! Editor Cylinder Collider: edit-viewport cylinder collider for the Jolt backend.
    //! Draws the cylinder in the Edit viewport and spawns the runtime
    //! JoltCylinderColliderComponent via BuildGameEntity.
    class EditorJoltCylinderColliderComponent
        : public EditorJoltColliderComponentBase
        , private AzToolsFramework::CylinderManipulatorRequestBus::Handler
        , private AzToolsFramework::RadiusManipulatorRequestBus::Handler
    {
    public:
        AZ_COMPONENT(EditorJoltCylinderColliderComponent, "{E5F6A7B8-C9D0-4314-C7D8-E9F0A1B2C3D4}", EditorJoltColliderComponentBase);

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

        // AzToolsFramework::CylinderManipulatorRequestBus
        float GetHeight() const override;
        void SetHeight(float height) override;

        // AzToolsFramework::RadiusManipulatorRequestBus
        float GetRadius() const override;
        void SetRadius(float radius) override;

    private:
        JoltCylinderShapeConfiguration m_shapeConfiguration;
    };
} // namespace JoltPhysics
