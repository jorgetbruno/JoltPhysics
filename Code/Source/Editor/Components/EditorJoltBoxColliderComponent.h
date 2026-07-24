#pragma once

#include <Editor/Components/EditorJoltColliderComponentBase.h>

#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    //! Editor Box Collider: edit-viewport box collider for the Jolt backend.
    //! Draws the box in the Edit viewport and spawns the runtime JoltBoxColliderComponent
    //! via BuildGameEntity (mirrors PhysX's EditorColliderComponent flow).
    class EditorJoltBoxColliderComponent
        : public EditorJoltColliderComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltBoxColliderComponent, "{B2C3D4E5-F6A7-4890-A1B2-C3D4E5F6A7B8}", EditorJoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    protected:
        // EditorJoltColliderComponentBase
        void DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const override;

    private:
        AZStd::shared_ptr<Physics::BoxShapeConfiguration> m_shapeConfiguration =
            AZStd::make_shared<Physics::BoxShapeConfiguration>();
    };
} // namespace JoltPhysics
