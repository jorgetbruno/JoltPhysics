#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/std/containers/vector.h>

namespace JoltPhysics
{
    //! Which Jolt vehicle controller drives the chassis.
    enum class JoltVehicleType : AZ::u8
    {
        Wheeled = 0, //!< JPH::WheeledVehicleController - car/truck, steered by wheel angle.
        Motorcycle,  //!< JPH::MotorcycleController - wheeled plus a lean-balance spring.
        Tracked,     //!< JPH::TrackedVehicleController - tank, steered by track speed.
    };

    //! One wheel of a Jolt vehicle (mirrors the useful subset of JPH::WheelSettingsWV).
    //! Steering and brake torques are ignored on a tracked vehicle, which brakes and
    //! steers through its tracks rather than per wheel.
    struct JoltWheelConfiguration
    {
        AZ_CLASS_ALLOCATOR(JoltWheelConfiguration, AZ::SystemAllocator);
        AZ_TYPE_INFO(JoltWheelConfiguration, "{A7B8C9D0-E1F2-4345-A6B7-C8D9E0F1A2B3}");
        static void Reflect(AZ::ReflectContext* context);

        AZ::Vector3 m_position = AZ::Vector3::CreateZero(); //!< Suspension attachment point in chassis space.
        float m_radius = 0.35f;
        float m_width = 0.25f;
        float m_suspensionMinLength = 0.15f; //!< Suspension length when fully raised (m).
        float m_suspensionMaxLength = 0.45f; //!< Suspension length at max droop (m).
        float m_suspensionFrequency = 1.5f;  //!< Spring frequency (Hz).
        float m_suspensionDamping = 0.7f;    //!< Spring damping (0..1+).
        float m_maxSteerAngleDegrees = 0.0f; //!< Steering lock in degrees; 0 = non-steering wheel.
        float m_maxBrakeTorque = 500.0f;
        float m_maxHandBrakeTorque = 1000.0f;
    };

    //! Vehicle settings: wheels, engine, transmission and drive layout. Which fields
    //! apply depends on m_vehicleType - the differential drives named wheels on a
    //! wheeled vehicle or motorcycle, while a tracked vehicle drives two tracks whose
    //! wheels are assigned automatically by side (mirrors the PhysXVehicle gem's
    //! configuration shape, extended for Jolt's other two controllers).
    struct JoltVehicleConfiguration
    {
        AZ_CLASS_ALLOCATOR(JoltVehicleConfiguration, AZ::SystemAllocator);
        AZ_TYPE_INFO(JoltVehicleConfiguration, "{B8C9D0E1-F2A3-4456-B7C8-D9E0F1A2B3C4}");
        static void Reflect(AZ::ReflectContext* context);

        JoltVehicleType m_vehicleType = JoltVehicleType::Wheeled;

        AZStd::vector<JoltWheelConfiguration> m_wheels;
        float m_chassisMass = 1200.0f; //!< Absolute chassis mass in kg (0 = keep the rigid body's mass).
        int m_leftDriveWheel = 2;    //!< Wheel index driven by the differential (-1 = none).
        int m_rightDriveWheel = 3;   //!< Wheel index driven by the differential (-1 = none).
        float m_differentialRatio = 3.42f;
        float m_maxEngineTorque = 500.0f; //!< Nm.
        float m_maxEngineRpm = 6000.0f;
        AZStd::vector<float> m_gearRatios = { 2.66f, 1.78f, 1.3f, 1.0f, 0.74f };
        float m_reverseGearRatio = -2.9f;

        //! Motorcycle: the lean-balance spring that keeps the bike upright.
        float m_maxLeanAngleDegrees = 45.0f;
        float m_leanSpringConstant = 5000.0f;
        float m_leanSpringDamping = 1000.0f;

        //! Tracked: per-track properties. Wheels are assigned to the left or right track
        //! by the sign of their Y position, and the first wheel of each track is its
        //! driven wheel.
        float m_trackInertia = 10.0f;           //!< kg m^2, as seen on the driven wheel.
        float m_trackAngularDamping = 0.5f;
        float m_trackMaxBrakeTorque = 15000.0f; //!< Nm on the driven wheel.
        float m_trackDifferentialRatio = 6.0f;

        //! Property visibility helpers for the editor (the inspector only shows the
        //! settings that apply to the selected vehicle type).
        AZ::Crc32 GetWheeledSettingsVisibility() const;
        AZ::Crc32 GetMotorcycleSettingsVisibility() const;
        AZ::Crc32 GetTrackedSettingsVisibility() const;
    };
} // namespace JoltPhysics
