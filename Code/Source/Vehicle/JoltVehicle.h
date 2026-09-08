#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/functional.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>

#include <JoltPhysics/JoltPhysicsBus.h>
#include <Vehicle/JoltVehicleConfiguration.h>

namespace JPH
{
    class WheeledVehicleController;
    class TrackedVehicleController;
    class VehicleCollisionTester;
}

namespace JoltPhysics
{
    //! The highest the ground can sit in chassis space and still be reachable by every
    //! wheel at full compression: min over wheels of (attachment - minSuspension - radius).
    float ComputeHighestReachableGround(const JoltVehicleConfiguration& configuration);

    //! How far the chassis collider hangs below that reach, or 0 when it clears. A positive
    //! value is the distance the collider has to be raised for the wheels to carry the car.
    float ComputeChassisColliderOvershoot(
        const JPH::Shape& shape, const JoltVehicleConfiguration& configuration);

    class JoltScene;

    //! Owns a JPH::VehicleConstraint attached to an existing chassis rigid body, driven
    //! by one of Jolt's three controllers (wheeled, motorcycle or tracked - see
    //! JoltVehicleType). The chassis remains a normal dynamic rigid body in the scene;
    //! this class only drives the wheels/tracks and engine.
    class JoltVehicle
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltVehicle, AZ::SystemAllocator);

        JoltVehicle(const JoltVehicleConfiguration& configuration, JoltScene* scene, JPH::Body* chassisBody);
        ~JoltVehicle();

        bool IsValid() const { return m_constraint != nullptr; }

        JoltVehicleType GetVehicleType() const { return m_vehicleType; }

        //! forward: throttle [-1..1], right: steering [-1..1], brake/handbrake [0..1].
        //! A tracked vehicle has no steered wheels and no separate handbrake: steering is
        //! converted to a left/right track speed ratio (full lock pivots the vehicle by
        //! reversing the inner track) and the handbrake is folded into the brake.
        void SetDriverInput(float forward, float right, float brake, float handbrake);

        //! Individual input channels, for callers that update one axis at a time (a
        //! gamepad trigger arriving on a different event than the stick). Each keeps the
        //! other channels at their last value.
        void SetForwardInput(float forward);
        void SetSteeringInput(float right);
        void SetBrakeInput(float brake);
        void SetHandBrakeInput(float handbrake);

        //! The driver input as last set, so a rebuild can put it back. Jolt bakes the
        //! configuration into the constraint, so changing it means building a new vehicle
        //! - and a car mid-corner should not drop the throttle because of that.
        [[nodiscard]] float GetForwardInput() const { return m_forwardInput; }
        [[nodiscard]] float GetSteeringInput() const { return m_steeringInput; }
        [[nodiscard]] float GetBrakeInput() const { return m_brakeInput; }
        [[nodiscard]] float GetHandBrakeInput() const { return m_handBrakeInput; }

        //! Forces the transmission into the given gear (-1 = reverse, 0 = neutral,
        //! 1.. = forward gears) with the clutch fully engaged. In Automatic mode the
        //! auto-shifter takes over again from the new gear.
        void SetGear(int gear);
        //! Switches between automatic and manual shifting at runtime.
        void SetTransmissionAutomatic(bool automatic);
        bool IsTransmissionAutomatic() const;

        //! Chassis speed along its forward axis (m/s).
        float GetSpeed() const;
        float GetEngineRpm() const;
        int GetCurrentGear() const;

        //! Motorcycle lean angle in radians (0 for the other vehicle types).
        float GetLeanAngle() const;

        //! Motorcycle runtime toggles (no-ops for the other vehicle types): the lean
        //! balance controller, and the limit that stops steering past the lean the bike
        //! can hold.
        void SetLeanControllerEnabled(bool enabled);
        void SetLeanSteeringLimitEnabled(bool enabled);

        //! How many wheels the constraint ended up with. This is the authored count
        //! unless none were authored, in which case it is the type's default layout.
        AZ::u32 GetWheelCount() const;

        //! World transform of a wheel, for driving a visual wheel mesh: it carries the
        //! suspension position, the steer angle and the rolling of the tyre. Returns
        //! false for an out-of-range index, leaving outTransform untouched.
        bool GetWheelTransform(AZ::u32 wheelIndex, AZ::Transform& outTransform) const;

        //! The same wheel, relative to the chassis body rather than the world.
        bool GetWheelLocalTransform(AZ::u32 wheelIndex, AZ::Transform& outTransform) const;

        //! Current suspension extension (m), for driving a visual suspension.
        float GetSuspensionLength(AZ::u32 wheelIndex) const;

        //! Whether the wheel found ground on the last step - a wheel in the air neither
        //! drives nor steers.
        bool IsWheelOnGround(AZ::u32 wheelIndex) const;

        //! Wheel spin (rad/s; positive rolls the vehicle forward).
        float GetWheelAngularVelocity(AZ::u32 wheelIndex) const;
        //! Current steering angle of the wheel (radians).
        float GetWheelSteerAngle(AZ::u32 wheelIndex) const;
        //! Longitudinal slip ratio (0 = full traction, 1 = locked or spinning; wheeled
        //! and motorcycle only - a tracked vehicle reports 0).
        float GetWheelLongitudinalSlip(AZ::u32 wheelIndex) const;
        //! Lateral slip angle in radians (wheeled and motorcycle only).
        float GetWheelLateralSlip(AZ::u32 wheelIndex) const;
        //! Ground contact point / normal in world space; false (output untouched) when
        //! the wheel is in the air.
        bool GetWheelContactPoint(AZ::u32 wheelIndex, AZ::Vector3& outPoint) const;
        bool GetWheelContactNormal(AZ::u32 wheelIndex, AZ::Vector3& outNormal) const;
        //! Whether the suspension is fully compressed and riding its hard stop.
        bool IsWheelSuspensionBottomedOut(AZ::u32 wheelIndex) const;

        //! The two per-wheel callbacks. Both are declared on JoltVehicleRequests, which
        //! is where the contract is written down - including the threading rules, which
        //! matter: these run on a Jolt job thread inside the step, with bodies locked.
        //! Passing an empty function restores Jolt's own default.
        //!
        //! JoltVehicleComponent remembers whatever is set here and re-applies it after a
        //! RecreateVehicle; nothing else does, so a caller holding a JoltVehicle directly
        //! owns that itself.
        using CombineFrictionFunction = JoltVehicleRequests::CombineFrictionFunction;
        using TireMaxImpulseFunction = JoltVehicleRequests::TireMaxImpulseFunction;
        void SetCombineFriction(CombineFrictionFunction combineFriction);
        //! No-op on a tracked vehicle: the tyre model belongs to the wheeled controller.
        void SetTireMaxImpulse(TireMaxImpulseFunction tireMaxImpulse);

        //! Replaces the scene gravity for this vehicle alone (driving on walls, loops);
        //! reset restores normal gravity and the chassis' gravity factor.
        void OverrideGravity(const AZ::Vector3& gravity);
        void ResetGravityOverride();

        //! The native constraint, the escape hatch for anything not wrapped - e.g.
        //! Jolt's pre/post-step and post-collide callbacks (C++ only by nature).
        JPH::VehicleConstraint* GetConstraint() const { return m_constraint; }

    private:
        //! Builds the wheels and controller settings for the configured vehicle type.
        void BuildWheeledSettings(const JoltVehicleConfiguration& configuration, JPH::VehicleConstraintSettings& settings);
        void BuildTrackedSettings(const JoltVehicleConfiguration& configuration, JPH::VehicleConstraintSettings& settings);

        //! Validates and copies the configured anti-roll bars onto the constraint settings.
        void BuildAntiRollBars(const JoltVehicleConfiguration& configuration, JPH::VehicleConstraintSettings& settings) const;

        //! The wheel/ground test for the configured mode, with O3DE's up axis applied.
        JPH::VehicleCollisionTester* CreateCollisionTester(const JoltVehicleConfiguration& configuration) const;

        //! Pushes the cached input channels into the controller and wakes the chassis.
        void ApplyDriverInput();

        JoltScene* m_scene = nullptr;
        JPH::Ref<JPH::VehicleConstraint> m_constraint;
        //! Set for Wheeled and Motorcycle (MotorcycleController derives from the wheeled one).
        JPH::WheeledVehicleController* m_wheeledController = nullptr;
        //! Set for Tracked only.
        JPH::TrackedVehicleController* m_trackedController = nullptr;
        JPH::Body* m_chassisBody = nullptr;
        JoltVehicleType m_vehicleType = JoltVehicleType::Wheeled;

        //! Last driver input per channel, so the individual setters compose.
        float m_forwardInput = 0.0f;
        float m_steeringInput = 0.0f;
        float m_brakeInput = 0.0f;
        float m_handBrakeInput = 0.0f;
    };
} // namespace JoltPhysics
