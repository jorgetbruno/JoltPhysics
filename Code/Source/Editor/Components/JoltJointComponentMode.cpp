#include <Editor/Components/JoltJointComponentMode.h>

#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzToolsFramework/Manipulators/ManipulatorManager.h>
#include <AzToolsFramework/Manipulators/RotationManipulators.h>
#include <AzToolsFramework/Manipulators/TranslationManipulators.h>

#include <Editor/Components/EditorJoltDebugDrawUtils.h>
#include <Editor/Components/JoltJointFrameRequestBus.h>

namespace JoltPhysics
{
    namespace
    {
        //! Sized against the drawn joint frame so the handles read as belonging to it.
        //! The rotation circles sit outside the translation arrows: both groups share an
        //! origin, and at equal size their bounds would cover each other.
        constexpr float TranslationAxisLength = EditorDebugDraw::JointAxisLength;
        constexpr float RotationCircleRadius = EditorDebugDraw::JointAxisLength * 1.6f;

    } // namespace

    void JoltJointComponentMode::Reflect(AZ::ReflectContext* context)
    {
        AzToolsFramework::ComponentModeFramework::ReflectEditorBaseComponentModeDescendant<JoltJointComponentMode>(context);
    }

    JoltJointComponentMode::JoltJointComponentMode(
        const AZ::EntityComponentIdPair& entityComponentIdPair, AZ::Uuid componentType)
        : EditorBaseComponentMode(entityComponentIdPair, componentType)
    {
        AZ::Transform frameSpace = AZ::Transform::CreateIdentity();
        JoltJointFrameRequestBus::EventResult(
            frameSpace, entityComponentIdPair, &JoltJointFrameRequests::GetJointFrameSpace);

        m_translationManipulators = AZStd::make_unique<AzToolsFramework::TranslationManipulators>(
            AzToolsFramework::TranslationManipulators::Dimensions::Three, frameSpace, AZ::Vector3::CreateOne());
        m_rotationManipulators = AZStd::make_unique<AzToolsFramework::RotationManipulators>(frameSpace);

        AzToolsFramework::TranslationManipulatorsViewCreateInfo translationView;
        translationView.linearAxisLength = TranslationAxisLength;
        translationView.linearConeLength = TranslationAxisLength * 0.3f;
        translationView.linearConeRadius = TranslationAxisLength * 0.1f;
        translationView.planarAxisLength = TranslationAxisLength * 0.4f;
        translationView.surfaceRadius = TranslationAxisLength * 0.1f;
        translationView.axis1Color = EditorDebugDraw::ManipulatorAxisColorX;
        translationView.axis2Color = EditorDebugDraw::ManipulatorAxisColorY;
        translationView.axis3Color = EditorDebugDraw::ManipulatorAxisColorZ;
        translationView.surfaceColor = EditorDebugDraw::ManipulatorSurfaceColor;
        m_translationManipulators->ConfigureView3d(translationView);

        m_rotationManipulators->SetLocalAxes(
            AZ::Vector3::CreateAxisX(), AZ::Vector3::CreateAxisY(), AZ::Vector3::CreateAxisZ());
        m_rotationManipulators->ConfigureView(
            RotationCircleRadius, EditorDebugDraw::ManipulatorAxisColorX,
            EditorDebugDraw::ManipulatorAxisColorY, EditorDebugDraw::ManipulatorAxisColorZ);

        const AZ::Transform localFrame = GetJointLocalFrame();
        SetManipulatorFrame(localFrame);

        // Every translation handle - the three axes, the three planes and the surface
        // drag - reports its result the same way, as a position in frame space.
        const auto onTranslated = [this](const AzToolsFramework::LinearManipulator::Action& action)
        {
            AZ::Transform frame = GetJointLocalFrame();
            frame.SetTranslation(action.LocalPosition());
            WriteFrameToComponent(frame);
            SetManipulatorFrame(frame);
        };
        const auto onTranslatedPlanar = [this](const AzToolsFramework::PlanarManipulator::Action& action)
        {
            AZ::Transform frame = GetJointLocalFrame();
            frame.SetTranslation(action.LocalPosition());
            WriteFrameToComponent(frame);
            SetManipulatorFrame(frame);
        };
        const auto onTranslatedSurface = [this](const AzToolsFramework::SurfaceManipulator::Action& action)
        {
            AZ::Transform frame = GetJointLocalFrame();
            frame.SetTranslation(action.LocalPosition());
            WriteFrameToComponent(frame);
            SetManipulatorFrame(frame);
        };
        const auto onTranslateFinished = [this]([[maybe_unused]] const auto& action)
        {
            RecordFrameEdit("Move Joint Frame");
        };

        m_translationManipulators->InstallLinearManipulatorMouseMoveCallback(onTranslated);
        m_translationManipulators->InstallLinearManipulatorMouseUpCallback(onTranslateFinished);
        m_translationManipulators->InstallPlanarManipulatorMouseMoveCallback(onTranslatedPlanar);
        m_translationManipulators->InstallPlanarManipulatorMouseUpCallback(onTranslateFinished);
        m_translationManipulators->InstallSurfaceManipulatorMouseMoveCallback(onTranslatedSurface);
        m_translationManipulators->InstallSurfaceManipulatorMouseUpCallback(onTranslateFinished);

        m_rotationManipulators->InstallMouseMoveCallback(
            [this](const AzToolsFramework::AngularManipulator::Action& action)
            {
                AZ::Transform frame = GetJointLocalFrame();
                frame.SetRotation(action.LocalOrientation().GetNormalized());
                WriteFrameToComponent(frame);
                SetManipulatorFrame(frame);
            });
        m_rotationManipulators->InstallLeftMouseUpCallback(
            [this]([[maybe_unused]] const AzToolsFramework::AngularManipulator::Action& action)
            {
                RecordFrameEdit("Rotate Joint Frame");
            });

        const AzToolsFramework::ManipulatorManagerId manipulatorManagerId =
            AzToolsFramework::GetMainManipulatorManagerId();
        m_translationManipulators->Register(manipulatorManagerId);
        m_translationManipulators->AddEntityComponentIdPair(entityComponentIdPair);
        m_rotationManipulators->Register(manipulatorManagerId);
        m_rotationManipulators->AddEntityComponentIdPair(entityComponentIdPair);
    }

    JoltJointComponentMode::~JoltJointComponentMode()
    {
        m_rotationManipulators->Unregister();
        m_translationManipulators->Unregister();
    }

    AZ::Transform JoltJointComponentMode::GetJointLocalFrame() const
    {
        AZ::Transform localFrame = AZ::Transform::CreateIdentity();
        JoltJointFrameRequestBus::EventResult(
            localFrame, GetEntityComponentIdPair(), &JoltJointFrameRequests::GetJointLocalFrame);
        return localFrame;
    }

    void JoltJointComponentMode::SetManipulatorFrame(const AZ::Transform& localFrame)
    {
        m_translationManipulators->SetLocalTransform(localFrame);
        m_translationManipulators->SetBoundsDirty();
        m_rotationManipulators->SetLocalTransform(localFrame);
        m_rotationManipulators->SetBoundsDirty();
    }

    void JoltJointComponentMode::WriteFrameToComponent(const AZ::Transform& localFrame)
    {
        JoltJointFrameRequestBus::Event(
            GetEntityComponentIdPair(), &JoltJointFrameRequests::SetJointLocalFrame, localFrame);
    }

    void JoltJointComponentMode::RecordFrameEdit(const char* undoLabel)
    {
        AzToolsFramework::ScopedUndoBatch undoBatch(undoLabel);
        AzToolsFramework::ScopedUndoBatch::MarkEntityDirty(GetEntityId());
    }

    void JoltJointComponentMode::Refresh()
    {
        // The frame moved from outside the viewport - the property grid, an undo, or the
        // follower entity being moved - so the handles have to catch up.
        AZ::Transform frameSpace = AZ::Transform::CreateIdentity();
        JoltJointFrameRequestBus::EventResult(
            frameSpace, GetEntityComponentIdPair(), &JoltJointFrameRequests::GetJointFrameSpace);

        m_translationManipulators->SetSpace(frameSpace);
        m_rotationManipulators->SetSpace(frameSpace);
        SetManipulatorFrame(GetJointLocalFrame());
    }

    AZStd::string JoltJointComponentMode::GetComponentModeName() const
    {
        return "Joint Frame Editing";
    }

    AZ::Uuid JoltJointComponentMode::GetComponentModeType() const
    {
        return azrtti_typeid<JoltJointComponentMode>();
    }
} // namespace JoltPhysics
