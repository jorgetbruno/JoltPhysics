#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>

namespace JoltPhysics
{
    //! A vehicle's wheel attachment points, as the viewport edits them.
    //!
    //! Lets JoltVehicleComponentMode place handles on a vehicle without reaching into
    //! its configuration, and keeps the wheel count out of the mode: a car has four
    //! handles, a bike two and a tank eight.
    class JoltVehicleWheelRequests : public AZ::EntityComponentBus
    {
    public:
        //! How many wheels are authored on this vehicle. Zero means the vehicle is
        //! relying on its type's default layout, which is built at simulation time and
        //! so has nothing to drag here.
        virtual AZ::u32 GetWheelCount() const = 0;

        //! The wheel's suspension attachment point, in chassis space.
        virtual AZ::Vector3 GetWheelPosition(AZ::u32 wheelIndex) const = 0;
        virtual void SetWheelPosition(AZ::u32 wheelIndex, const AZ::Vector3& position) = 0;

        //! The space those positions are in: the chassis entity's world transform,
        //! without scale.
        virtual AZ::Transform GetChassisSpace() const = 0;

    protected:
        ~JoltVehicleWheelRequests() = default;
    };

    using JoltVehicleWheelRequestBus = AZ::EBus<JoltVehicleWheelRequests>;
} // namespace JoltPhysics
