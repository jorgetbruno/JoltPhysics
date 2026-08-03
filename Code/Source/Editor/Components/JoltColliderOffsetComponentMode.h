#pragma once

#include <AzCore/std/smart_ptr/unique_ptr.h>

#include <AzToolsFramework/ComponentMode/EditorBaseComponentMode.h>

namespace AzToolsFramework
{
    class ShapeTranslationOffsetViewportEdit;
} // namespace AzToolsFramework

namespace JoltPhysics
{
    //! A drag handle for a collider's translation offset, and nothing else.
    //!
    //! Every collider carries that offset and every editor collider already serves it to
    //! manipulators through ShapeManipulatorRequestBus - but a manipulator only appears
    //! for a component that connects a ComponentModeDelegate, and the colliders whose
    //! geometry comes from a blob (the .joltmesh asset collider and the baked mesh
    //! collider) had none. Their offset was typed coordinates only, while a box on the
    //! same entity got a handle for exactly the same field. Nudging a shared prop
    //! collider - a door, a fence, a rock reused across entities - against its render
    //! mesh is routine, and it is the one edit those colliders do support.
    //!
    //! The dimension sub-mode the engine's shape modes pair this with is deliberately
    //! absent: a cooked hull has no radius or half-extent to drag. With one sub-mode
    //! there is no viewport cluster to switch between them either, so the mode is the
    //! handle.
    class JoltColliderOffsetComponentMode
        : public AzToolsFramework::ComponentModeFramework::EditorBaseComponentMode
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltColliderOffsetComponentMode, AZ::SystemAllocator);
        AZ_RTTI(
            JoltColliderOffsetComponentMode,
            "{2F8B4D16-90C7-4A5E-B3D2-6C1E0A97F4B8}",
            AzToolsFramework::ComponentModeFramework::EditorBaseComponentMode);

        static void Reflect(AZ::ReflectContext* context);

        JoltColliderOffsetComponentMode(const AZ::EntityComponentIdPair& entityComponentIdPair, AZ::Uuid componentType);
        JoltColliderOffsetComponentMode(const JoltColliderOffsetComponentMode&) = delete;
        JoltColliderOffsetComponentMode& operator=(const JoltColliderOffsetComponentMode&) = delete;
        ~JoltColliderOffsetComponentMode() override;

        // EditorBaseComponentMode overrides ...
        void Refresh() override;
        AZStd::string GetComponentModeName() const override;
        AZ::Uuid GetComponentModeType() const override;

    private:
        //! The engine's own offset handle, driven through ShapeManipulatorRequestBus -
        //! the bus EditorJoltColliderComponentBase already answers. Nothing here knows
        //! what shape it is offsetting, which is why one mode serves both mesh colliders.
        AZStd::unique_ptr<AzToolsFramework::ShapeTranslationOffsetViewportEdit> m_offsetViewportEdit;
    };
} // namespace JoltPhysics
