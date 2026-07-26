#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>

#include <AzToolsFramework/ComponentMode/EditorBaseComponentMode.h>

namespace AzToolsFramework
{
    class TranslationManipulators;
} // namespace AzToolsFramework

namespace JoltPhysics
{
    //! Drag handles for a vehicle's wheel positions.
    //!
    //! One translation gizmo per wheel, on the suspension attachment point, writing
    //! back through JoltVehicleWheelRequestBus. Placing wheels by eye against the
    //! chassis is the whole job: the numbers alone give no sense of whether a wheel
    //! pokes out of the bodywork or sits inside it.
    //!
    //! Everything else about a wheel - radius, width, suspension travel, steering lock -
    //! stays in the property grid. They are one-dimensional numbers that the drawn wheel
    //! already shows, and a handle per wheel per property would bury the vehicle.
    class JoltVehicleComponentMode
        : public AzToolsFramework::ComponentModeFramework::EditorBaseComponentMode
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltVehicleComponentMode, AZ::SystemAllocator);
        AZ_RTTI(
            JoltVehicleComponentMode,
            "{2F8B4D17-6C93-4A5E-B821-3D7E9F0A1C46}",
            AzToolsFramework::ComponentModeFramework::EditorBaseComponentMode);

        static void Reflect(AZ::ReflectContext* context);

        JoltVehicleComponentMode(const AZ::EntityComponentIdPair& entityComponentIdPair, AZ::Uuid componentType);
        JoltVehicleComponentMode(const JoltVehicleComponentMode&) = delete;
        JoltVehicleComponentMode& operator=(const JoltVehicleComponentMode&) = delete;
        ~JoltVehicleComponentMode() override;

        // EditorBaseComponentMode overrides ...
        void Refresh() override;
        AZStd::string GetComponentModeName() const override;
        AZ::Uuid GetComponentModeType() const override;

    private:
        //! Builds one handle per authored wheel. Called again from Refresh, since adding
        //! or removing a wheel in the property grid changes how many handles there are.
        void RebuildManipulators();
        void DestroyManipulators();

        AZ::Vector3 GetWheelPosition(AZ::u32 wheelIndex) const;
        AZ::Transform GetChassisSpace() const;

        AZStd::vector<AZStd::unique_ptr<AzToolsFramework::TranslationManipulators>> m_wheelManipulators;
    };
} // namespace JoltPhysics
