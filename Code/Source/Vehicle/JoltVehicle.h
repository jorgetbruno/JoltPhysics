#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>

#include <Vehicle/JoltVehicleConfiguration.h>

namespace JPH
{
    class WheeledVehicleController;
    class TrackedVehicleController;
}

namespace JoltPhysics
{
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

        //! Chassis speed along its forward axis (m/s).
        float GetSpeed() const;
        float GetEngineRpm() const;
        int GetCurrentGear() const;

        //! Motorcycle lean angle in radians (0 for the other vehicle types).
        float GetLeanAngle() const;

        JPH::VehicleConstraint* GetConstraint() const { return m_constraint; }

    private:
        //! Builds the wheels and controller settings for the configured vehicle type.
        void BuildWheeledSettings(const JoltVehicleConfiguration& configuration, JPH::VehicleConstraintSettings& settings);
        void BuildTrackedSettings(const JoltVehicleConfiguration& configuration, JPH::VehicleConstraintSettings& settings);

        JoltScene* m_scene = nullptr;
        JPH::Ref<JPH::VehicleConstraint> m_constraint;
        //! Set for Wheeled and Motorcycle (MotorcycleController derives from the wheeled one).
        JPH::WheeledVehicleController* m_wheeledController = nullptr;
        //! Set for Tracked only.
        JPH::TrackedVehicleController* m_trackedController = nullptr;
        JPH::Body* m_chassisBody = nullptr;
        JoltVehicleType m_vehicleType = JoltVehicleType::Wheeled;
    };
} // namespace JoltPhysics
