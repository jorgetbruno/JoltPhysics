#pragma once

#include <Editor/Components/EditorJoltColliderComponentBase.h>

namespace JoltPhysics
{
    //! Editor Jolt Mutable Compound Collider: edit-time counterpart of
    //! JoltMutableCompoundColliderComponent. Spawns the runtime component via
    //! BuildGameEntity; the compound gathers its child entities' colliders at
    //! activation (and re-gathers as children change), so there is no shape of
    //! its own to configure or draw.
    class EditorJoltMutableCompoundColliderComponent
        : public EditorJoltColliderComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltMutableCompoundColliderComponent, "{E1F2A3B4-C5D6-4789-D0E1-F2A3B4C5D6E7}", EditorJoltColliderComponentBase);

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
