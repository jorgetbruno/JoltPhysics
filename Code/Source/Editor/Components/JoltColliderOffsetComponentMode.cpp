#include <Editor/Components/JoltColliderOffsetComponentMode.h>

#include <AzToolsFramework/ComponentModes/BaseShapeComponentMode.h>
#include <AzToolsFramework/ComponentModes/ShapeTranslationOffsetViewportEdit.h>
#include <AzToolsFramework/Manipulators/ManipulatorManager.h>

namespace JoltPhysics
{
    void JoltColliderOffsetComponentMode::Reflect(AZ::ReflectContext* context)
    {
        AzToolsFramework::ComponentModeFramework::ReflectEditorBaseComponentModeDescendant<JoltColliderOffsetComponentMode>(
            context);
    }

    JoltColliderOffsetComponentMode::JoltColliderOffsetComponentMode(
        const AZ::EntityComponentIdPair& entityComponentIdPair, AZ::Uuid componentType)
        : EditorBaseComponentMode(entityComponentIdPair, componentType)
    {
        m_offsetViewportEdit = AZStd::make_unique<AzToolsFramework::ShapeTranslationOffsetViewportEdit>();

        // Wires the handle to ShapeManipulatorRequestBus for this component, which
        // EditorJoltColliderComponentBase already implements: get/set the offset, the
        // manipulator space, the rotation offset. Nothing collider-specific is needed
        // here, which is the whole reason this mode is small.
        AzToolsFramework::InstallBaseShapeViewportEditFunctions(m_offsetViewportEdit.get(), entityComponentIdPair);

        m_offsetViewportEdit->Setup(AzToolsFramework::GetMainManipulatorManagerId());
        // After Setup, never before - the manipulators it hooks up to undo/redo and
        // property-grid refreshing do not exist until then.
        m_offsetViewportEdit->AddEntityComponentIdPair(entityComponentIdPair);
    }

    JoltColliderOffsetComponentMode::~JoltColliderOffsetComponentMode()
    {
        m_offsetViewportEdit->Teardown();
    }

    void JoltColliderOffsetComponentMode::Refresh()
    {
        // The offset moved from outside the viewport - the property grid, an undo, the
        // entity being moved - so the handle has to catch up.
        m_offsetViewportEdit->UpdateManipulators();
    }

    AZStd::string JoltColliderOffsetComponentMode::GetComponentModeName() const
    {
        return "Collider Offset Editing";
    }

    AZ::Uuid JoltColliderOffsetComponentMode::GetComponentModeType() const
    {
        return azrtti_typeid<JoltColliderOffsetComponentMode>();
    }
} // namespace JoltPhysics
