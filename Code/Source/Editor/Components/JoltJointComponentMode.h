#pragma once

#include <AzCore/Math/Transform.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>

#include <AzToolsFramework/ComponentMode/EditorBaseComponentMode.h>

namespace AzToolsFramework
{
    class RotationManipulators;
    class TranslationManipulators;
} // namespace AzToolsFramework

namespace JoltPhysics
{
    //! Drag handles for a joint's frame.
    //!
    //! One mode serves every joint type: it reads and writes the frame through
    //! JoltJointFrameRequestBus rather than knowing about hinges or cones, and the
    //! limits each joint draws follow along because they are drawn relative to that
    //! same frame.
    //!
    //! Translation and rotation handles are shown together rather than as separate
    //! sub-modes (PhysX cycles between them from a viewport UI cluster). The rotation
    //! circles are sized clear of the translation arrows so both stay clickable, which
    //! buys the whole frame in one gesture set without a mode-switching UI.
    //!
    //! The lead and follower entities stay property-grid fields - they name entities,
    //! which is not something a drag handle expresses.
    class JoltJointComponentMode
        : public AzToolsFramework::ComponentModeFramework::EditorBaseComponentMode
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltJointComponentMode, AZ::SystemAllocator);
        AZ_RTTI(
            JoltJointComponentMode,
            "{6E2C9E5A-1B7D-4C33-9A48-7F0D2E1B5C84}",
            AzToolsFramework::ComponentModeFramework::EditorBaseComponentMode);

        static void Reflect(AZ::ReflectContext* context);

        JoltJointComponentMode(const AZ::EntityComponentIdPair& entityComponentIdPair, AZ::Uuid componentType);
        JoltJointComponentMode(const JoltJointComponentMode&) = delete;
        JoltJointComponentMode& operator=(const JoltJointComponentMode&) = delete;
        ~JoltJointComponentMode() override;

        // EditorBaseComponentMode overrides ...
        void Refresh() override;
        AZStd::string GetComponentModeName() const override;
        AZ::Uuid GetComponentModeType() const override;

    private:
        //! Moves both manipulator groups onto the given frame. Both carry the full
        //! transform, so the translation arrows point along the joint's own axes rather
        //! than the follower's.
        void SetManipulatorFrame(const AZ::Transform& localFrame);

        //! Writes an edited frame back to the component and refreshes the inspector.
        //! Values only: rebuilding the property tree mid-drag would destroy the
        //! manipulator under the cursor.
        void WriteFrameToComponent(const AZ::Transform& localFrame);

        //! Records the completed drag as one undoable change. Taken on mouse up rather
        //! than per move, so a drag is a single undo step, and scoped so the batch
        //! always closes - an batch left open blocks saving the level.
        void RecordFrameEdit(const char* undoLabel);

        AZ::Transform GetJointLocalFrame() const;

        AZStd::unique_ptr<AzToolsFramework::TranslationManipulators> m_translationManipulators;
        AZStd::unique_ptr<AzToolsFramework::RotationManipulators> m_rotationManipulators;
    };
} // namespace JoltPhysics
