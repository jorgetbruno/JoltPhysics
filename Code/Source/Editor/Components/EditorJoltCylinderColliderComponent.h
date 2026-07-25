#pragma once

#include <Editor/Components/EditorJoltColliderComponentBase.h>

#include <Shape/JoltCylinderShapeConfiguration.h>

namespace JoltPhysics
{
    //! Editor Cylinder Collider: edit-viewport cylinder collider for the Jolt backend.
    //! Draws the cylinder in the Edit viewport and spawns the runtime
    //! JoltCylinderColliderComponent via BuildGameEntity.
    class EditorJoltCylinderColliderComponent
        : public EditorJoltColliderComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltCylinderColliderComponent, "{E5F6A7B8-C9D0-4314-C7D8-E9F0A1B2C3D4}", EditorJoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    protected:
        // EditorJoltColliderComponentBase
        void DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const override;

    private:
        AZStd::shared_ptr<JoltCylinderShapeConfiguration> m_shapeConfiguration =
            AZStd::make_shared<JoltCylinderShapeConfiguration>();
    };
} // namespace JoltPhysics
