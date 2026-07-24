#pragma once

#include <Editor/Components/EditorJoltColliderComponentBase.h>

#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    //! Editor Sphere Collider: edit-viewport sphere collider for the Jolt backend.
    //! Draws the sphere in the Edit viewport and spawns the runtime
    //! JoltSphereColliderComponent via BuildGameEntity.
    class EditorJoltSphereColliderComponent
        : public EditorJoltColliderComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltSphereColliderComponent, "{C3D4E5F6-A7B8-4901-B2C3-D4E5F6A7B8C9}", EditorJoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    protected:
        // EditorJoltColliderComponentBase
        void DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const override;

    private:
        AZStd::shared_ptr<Physics::SphereShapeConfiguration> m_shapeConfiguration =
            AZStd::make_shared<Physics::SphereShapeConfiguration>();
    };
} // namespace JoltPhysics
