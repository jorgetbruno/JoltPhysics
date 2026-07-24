#pragma once

#include <Editor/Components/EditorJoltColliderComponentBase.h>

namespace JoltPhysics
{
    //! Editor Jolt Static Compound Collider: edit-time counterpart of
    //! JoltStaticCompoundColliderComponent. Spawns the runtime component via
    //! BuildGameEntity; the compound gathers its child entities' colliders at
    //! activation, so there is no shape of its own to configure or draw.
    class EditorJoltStaticCompoundColliderComponent
        : public EditorJoltColliderComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltStaticCompoundColliderComponent, "{B8C9D0E1-F2A3-4456-A7B8-C9D0E1F2A3B4}", EditorJoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    protected:
        // EditorJoltColliderComponentBase (children draw their own wireframes)
        void DrawShape([[maybe_unused]] AzFramework::DebugDisplayRequests& debugDisplay) const override
        {
        }
    };
} // namespace JoltPhysics
