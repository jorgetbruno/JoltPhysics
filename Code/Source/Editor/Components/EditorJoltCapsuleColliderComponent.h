#pragma once

#include <Editor/Components/EditorJoltColliderComponentBase.h>

#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    //! Editor Capsule Collider: edit-viewport capsule collider for the Jolt backend.
    //! Draws the capsule in the Edit viewport and spawns the runtime
    //! JoltCapsuleColliderComponent via BuildGameEntity.
    class EditorJoltCapsuleColliderComponent
        : public EditorJoltColliderComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltCapsuleColliderComponent, "{D4E5F6A7-B8C9-4012-C3D4-E5F6A7B8C9D0}", EditorJoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    protected:
        // EditorJoltColliderComponentBase
        void DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const override;

    private:
        AZStd::shared_ptr<Physics::CapsuleShapeConfiguration> m_shapeConfiguration =
            AZStd::make_shared<Physics::CapsuleShapeConfiguration>();
    };
} // namespace JoltPhysics
