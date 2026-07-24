#pragma once

#include <Editor/Components/EditorJoltColliderComponentBase.h>

namespace JoltPhysics
{
    //! Editor Jolt Heightfield Collider: edit-time counterpart of
    //! JoltHeightfieldColliderComponent. Spawns the runtime component via
    //! BuildGameEntity; the heightfield itself comes from the entity's
    //! Physics::HeightfieldProviderBus implementation, so there is no shape to
    //! configure or draw here.
    class EditorJoltHeightfieldColliderComponent
        : public EditorJoltColliderComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltHeightfieldColliderComponent, "{A7B8C9D0-E1F2-4345-F6A7-B8C9D0E1F2A3}", EditorJoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    protected:
        // EditorJoltColliderComponentBase (nothing cheap to draw; the terrain
        // provider already visualizes the surface)
        void DrawShape([[maybe_unused]] AzFramework::DebugDisplayRequests& debugDisplay) const override
        {
        }
    };
} // namespace JoltPhysics
