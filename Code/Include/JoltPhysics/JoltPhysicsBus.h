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

} // namespace JoltPhysics
