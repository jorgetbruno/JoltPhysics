#include <Editor/Components/JoltVehicleComponentMode.h>

#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzToolsFramework/Manipulators/ManipulatorManager.h>
#include <AzToolsFramework/Manipulators/TranslationManipulators.h>

#include <Editor/Components/EditorJoltDebugDrawUtils.h>
#include <Editor/Components/JoltVehicleWheelRequestBus.h>

namespace JoltPhysics
{
    namespace
    {
        //! Small next to a wheel, so four of these on a car read as handles on the
        //! wheels rather than as a thicket over the whole vehicle.
        constexpr float WheelHandleAxisLength = 0.35f;

    } // namespace

    void JoltVehicleComponentMode::Reflect(AZ::ReflectContext* context)
    {
        AzToolsFramework::ComponentModeFramework::ReflectEditorBaseComponentModeDescendant<JoltVehicleComponentMode>(
            context);
    }

    JoltVehicleComponentMode::JoltVehicleComponentMode(
        const AZ::EntityComponentIdPair& entityComponentIdPair, AZ::Uuid componentType)
        : EditorBaseComponentMode(entityComponentIdPair, componentType)
    {
        RebuildManipulators();
    }

    JoltVehicleComponentMode::~JoltVehicleComponentMode()
    {
        DestroyManipulators();
    }

    AZ::Transform JoltVehicleComponentMode::GetChassisSpace() const
    {
        AZ::Transform chassisSpace = AZ::Transform::CreateIdentity();
        JoltVehicleWheelRequestBus::EventResult(
            chassisSpace, GetEntityComponentIdPair(), &JoltVehicleWheelRequests::GetChassisSpace);
        return chassisSpace;
    }

    AZ::Vector3 JoltVehicleComponentMode::GetWheelPosition(AZ::u32 wheelIndex) const
    {
        AZ::Vector3 position = AZ::Vector3::CreateZero();
        JoltVehicleWheelRequestBus::EventResult(
            position, GetEntityComponentIdPair(), &JoltVehicleWheelRequests::GetWheelPosition, wheelIndex);
        return position;
    }

    void JoltVehicleComponentMode::DestroyManipulators()
    {
        for (auto& manipulators : m_wheelManipulators)
        {
            manipulators->Unregister();
        }
        m_wheelManipulators.clear();
    }

    void JoltVehicleComponentMode::RebuildManipulators()
    {
        DestroyManipulators();

        AZ::u32 wheelCount = 0;
        JoltVehicleWheelRequestBus::EventResult(
            wheelCount, GetEntityComponentIdPair(), &JoltVehicleWheelRequests::GetWheelCount);

        const AZ::Transform chassisSpace = GetChassisSpace();
        const AzToolsFramework::ManipulatorManagerId manipulatorManagerId =
            AzToolsFramework::GetMainManipulatorManagerId();

        for (AZ::u32 wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex)
        {
            auto manipulators = AZStd::make_unique<AzToolsFramework::TranslationManipulators>(
                AzToolsFramework::TranslationManipulators::Dimensions::Three, chassisSpace, AZ::Vector3::CreateOne());

            AzToolsFramework::TranslationManipulatorsViewCreateInfo view;
            view.linearAxisLength = WheelHandleAxisLength;
            view.linearConeLength = WheelHandleAxisLength * 0.3f;
            view.linearConeRadius = WheelHandleAxisLength * 0.1f;
            view.planarAxisLength = WheelHandleAxisLength * 0.4f;
            view.surfaceRadius = WheelHandleAxisLength * 0.1f;
            view.axis1Color = EditorDebugDraw::ManipulatorAxisColorX;
            view.axis2Color = EditorDebugDraw::ManipulatorAxisColorY;
            view.axis3Color = EditorDebugDraw::ManipulatorAxisColorZ;
            view.surfaceColor = EditorDebugDraw::ManipulatorSurfaceColor;
            manipulators->ConfigureView3d(view);

            manipulators->SetLocalPosition(GetWheelPosition(wheelIndex));

            // Every handle - the three axes, the three planes and the surface drag -
            // reports the same thing: the wheel's new position in chassis space.
            auto* manipulatorsPtr = manipulators.get();
            const auto onMoved = [this, wheelIndex, manipulatorsPtr](const AZ::Vector3& localPosition)
            {
                JoltVehicleWheelRequestBus::Event(
                    GetEntityComponentIdPair(), &JoltVehicleWheelRequests::SetWheelPosition, wheelIndex, localPosition);
                manipulatorsPtr->SetLocalPosition(localPosition);
                manipulatorsPtr->SetBoundsDirty();

                // Values only: rebuilding the property tree mid-drag would take the
                // manipulator out from under the cursor.
                AzToolsFramework::ToolsApplicationEvents::Bus::Broadcast(
                    &AzToolsFramework::ToolsApplicationEvents::InvalidatePropertyDisplay,
                    AzToolsFramework::Refresh_Values);
            };
            const auto onFinished = [this]([[maybe_unused]] const auto& action)
            {
                // One undo entry per drag, scoped so the batch always closes.
                AzToolsFramework::ScopedUndoBatch undoBatch("Move Vehicle Wheel");
                AzToolsFramework::ScopedUndoBatch::MarkEntityDirty(GetEntityId());
            };

            manipulators->InstallLinearManipulatorMouseMoveCallback(
                [onMoved](const AzToolsFramework::LinearManipulator::Action& action)
                {
                    onMoved(action.LocalPosition());
                });
            manipulators->InstallPlanarManipulatorMouseMoveCallback(
                [onMoved](const AzToolsFramework::PlanarManipulator::Action& action)
                {
                    onMoved(action.LocalPosition());
                });
            manipulators->InstallSurfaceManipulatorMouseMoveCallback(
                [onMoved](const AzToolsFramework::SurfaceManipulator::Action& action)
                {
                    onMoved(action.LocalPosition());
                });
            manipulators->InstallLinearManipulatorMouseUpCallback(onFinished);
            manipulators->InstallPlanarManipulatorMouseUpCallback(onFinished);
            manipulators->InstallSurfaceManipulatorMouseUpCallback(onFinished);

            manipulators->Register(manipulatorManagerId);
            manipulators->AddEntityComponentIdPair(GetEntityComponentIdPair());
            m_wheelManipulators.push_back(AZStd::move(manipulators));
        }
    }

    void JoltVehicleComponentMode::Refresh()
    {
        AZ::u32 wheelCount = 0;
        JoltVehicleWheelRequestBus::EventResult(
            wheelCount, GetEntityComponentIdPair(), &JoltVehicleWheelRequests::GetWheelCount);

        // A wheel added or removed in the property grid changes how many handles there
        // should be, so that case starts over rather than moving the ones that exist.
        if (wheelCount != m_wheelManipulators.size())
        {
            RebuildManipulators();
            return;
        }

        const AZ::Transform chassisSpace = GetChassisSpace();
        for (AZ::u32 wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex)
        {
            auto& manipulators = m_wheelManipulators[wheelIndex];
            manipulators->SetSpace(chassisSpace);
            manipulators->SetLocalPosition(GetWheelPosition(wheelIndex));
            manipulators->SetBoundsDirty();
        }
    }

    AZStd::string JoltVehicleComponentMode::GetComponentModeName() const
    {
        return "Vehicle Wheel Editing";
    }

    AZ::Uuid JoltVehicleComponentMode::GetComponentModeType() const
    {
        return azrtti_typeid<JoltVehicleComponentMode>();
    }
} // namespace JoltPhysics
