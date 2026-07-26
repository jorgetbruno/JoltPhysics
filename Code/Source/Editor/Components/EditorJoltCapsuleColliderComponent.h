#pragma once

#include <Editor/Components/EditorJoltColliderComponentBase.h>

#include <AzFramework/Physics/ShapeConfiguration.h>

#include <AzToolsFramework/Manipulators/CapsuleManipulatorRequestBus.h>
#include <AzToolsFramework/Manipulators/RadiusManipulatorRequestBus.h>

namespace JoltPhysics
{
    //! Editor Capsule Collider: edit-viewport capsule collider for the Jolt backend.
    //! Draws the capsule in the Edit viewport and spawns the runtime
    //! JoltCapsuleColliderComponent via BuildGameEntity.
    class EditorJoltCapsuleColliderComponent
        : public EditorJoltColliderComponentBase
        , private AzToolsFramework::CapsuleManipulatorRequestBus::Handler
        , private AzToolsFramework::RadiusManipulatorRequestBus::Handler
    {
    public:
        AZ_COMPONENT(EditorJoltCapsuleColliderComponent, "{D4E5F6A7-B8C9-4012-C3D4-E5F6A7B8C9D0}", EditorJoltColliderComponentBase);

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

        // AzToolsFramework::CapsuleManipulatorRequestBus
        float GetHeight() const override;
        void SetHeight(float height) override;

        // AzToolsFramework::RadiusManipulatorRequestBus
        float GetRadius() const override;
        void SetRadius(float radius) override;

    private:
        Physics::CapsuleShapeConfiguration m_shapeConfiguration;
    };
} // namespace JoltPhysics
