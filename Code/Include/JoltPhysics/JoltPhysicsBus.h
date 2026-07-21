#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/utility/pair.h>

namespace JoltPhysics
{
    class JoltPhysicsRequests
    {
    public:
        AZ_RTTI(JoltPhysicsRequests, "{E9F7A5B3-4C2D-4E8F-9A1B-3C5D7E8F9A2B}");
        virtual ~JoltPhysicsRequests() = default;
    };

    class JoltPhysicsBusTraits
        : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
    };

    using JoltPhysicsRequestBus = AZ::EBus<JoltPhysicsRequests, JoltPhysicsBusTraits>;
    using JoltPhysicsInterface = AZ::Interface<JoltPhysicsRequests>;

    //! Runtime control of joints (mirrors the PhysX gem's JointRequestBus surface so
    //! gameplay code can read joint state and drive motors per joint component).
    class JoltJointRequests
        : public AZ::ComponentBus
    {
    public:
        virtual ~JoltJointRequests() = default;

        //! Current joint position: hinge angle (radians) or slider displacement (meters).
        virtual float GetPosition() const = 0;
        //! Current joint velocity: hinge angular velocity (rad/s) or slider speed (m/s).
        virtual float GetVelocity() const = 0;
        //! The joint frame in world space.
        virtual AZ::Transform GetTransform() const = 0;
        //! Drives the joint motor at the given velocity (hinge: rad/s, slider: m/s).
        virtual void SetVelocity(float velocity) = 0;
        //! Sets the maximum force/torque the motor may apply.
        virtual void SetMaximumForce(float force) = 0;
        //! The configured limits (hinge: radians, slider: meters).
        virtual AZStd::pair<float, float> GetLimits() const = 0;
    };

    using JoltJointRequestBus = AZ::EBus<JoltJointRequests>;

    //! Runtime control of vehicles (AzPhysics has no vehicle interfaces in O3DE 26.05,
    //! so the vehicle surface lives on this gem's own bus).
    class JoltVehicleRequests
        : public AZ::ComponentBus
    {
    public:
        virtual ~JoltVehicleRequests() = default;

        //! Applies driver input: throttle [-1..1], steering [-1..1], brake [0..1], handbrake [0..1].
        virtual void SetDriverInput(float forward, float right, float brake, float handbrake) = 0;
        //! Chassis speed along its forward axis (m/s).
        virtual float GetSpeed() const = 0;
        virtual float GetEngineRpm() const = 0;
        virtual int GetCurrentGear() const = 0;
    };

    using JoltVehicleRequestBus = AZ::EBus<JoltVehicleRequests>;

} // namespace JoltPhysics
