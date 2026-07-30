#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

namespace JoltPhysics
{
    //! Which Jolt vehicle controller drives the chassis.
    enum class JoltVehicleType : AZ::u8
    {
        Wheeled = 0, //!< JPH::WheeledVehicleController - car/truck, steered by wheel angle.
        Motorcycle,  //!< JPH::MotorcycleController - wheeled plus a lean-balance spring.
        Tracked,     //!< JPH::TrackedVehicleController - tank, steered by track speed.
    };

    //! How each wheel looks for the ground.
    enum class JoltVehicleCollisionTester : AZ::u8
    {
        //! One ray straight down the suspension. Cheapest, and what a wheel of zero
        //! width would do: it can drop into a gap or catch on an edge narrower than
        //! the tyre, because nothing but the centre line is tested.
        Ray = 0,
        //! Sphere cast of the wheel's radius. Rolls over edges and small gaps the ray
        //! falls into, without the cost of the full wheel shape.
        Sphere,
        //! Cylinder cast of the whole wheel. Closest to the real contact patch,
        //! and the most expensive.
        Cylinder,
    };

    //! Couples the suspension of two wheels so the chassis leans less in a corner, the
    //! way a real anti-roll bar does. Without one a tall vehicle on soft springs rolls
    //! onto its side in a turn that it would otherwise take comfortably.
    struct JoltVehicleAntiRollBar
    {
        AZ_CLASS_ALLOCATOR(JoltVehicleAntiRollBar, AZ::SystemAllocator);
        AZ_TYPE_INFO(JoltVehicleAntiRollBar, "{3C1D7E96-4A25-4B0E-8F31-9D6A2B4C7E58}");
        static void Reflect(AZ::ReflectContext* context);

        int m_leftWheel = 0;  //!< Index into the wheel list.
        int m_rightWheel = 1; //!< Index into the wheel list.
        float m_stiffness = 1000.0f; //!< Spring constant (N/m); 0 disables this bar.
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

        //! How the wheels look for the ground. Cylinder by default: a ray only tests the
        //! wheel's centre line, so a ray-tested wheel drops into gaps and catches on
        //! edges a real tyre of that width would roll straight over.
        JoltVehicleCollisionTester m_collisionTester = JoltVehicleCollisionTester::Cylinder;

        //! Anti-roll bars, each naming the two wheels it couples. Empty by default:
        //! whether a vehicle wants one, and how stiff, depends on its geometry.
        AZStd::vector<JoltVehicleAntiRollBar> m_antiRollBars;

        //! How far the chassis may pitch or roll away from world up before the suspension
        //! stops pushing (degrees; 180 disables the limit, which is Jolt's own default).
        //!
        //! 60 here, matching Jolt's samples, because without a limit a vehicle powers
        //! itself right over: the default tank has enough torque to pop a wheelie, and
        //! with nothing to stop it the suspension keeps driving from the vertical until
        //! it lands on its back. The limit costs nothing on a slope a vehicle could
        //! actually climb.
        float m_maxPitchRollAngleDegrees = 60.0f;

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

        //! The owning entity's name, set at creation and not serialized: it exists so the
        //! vehicle's diagnostics can say which vehicle they are about (see
        //! Internal::NameClause). Mirrors m_debugName on the AzPhysics body configurations.
        AZStd::string m_debugName;

        //! Property visibility helpers for the editor (the inspector only shows the
        //! settings that apply to the selected vehicle type).
        AZ::Crc32 GetWheeledSettingsVisibility() const;
        AZ::Crc32 GetMotorcycleSettingsVisibility() const;
        AZ::Crc32 GetTrackedSettingsVisibility() const;
    };
} // namespace JoltPhysics
