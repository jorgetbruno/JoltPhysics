#pragma once

#include <AzCore/Math/Vector2.h>
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

    //! How gears are selected (JPH::ETransmissionMode).
    enum class JoltVehicleTransmissionMode : AZ::u8
    {
        Automatic = 0, //!< Shifts by the RPM thresholds below.
        Manual,        //!< Gears change only through JoltVehicleRequestBus::SetGear.
    };

    //! How the suspension spring fields are interpreted (JPH::ESpringMode).
    enum class JoltSuspensionSpringMode : AZ::u8
    {
        FrequencyAndDamping = 0, //!< Spring strength given as a frequency (Hz) - self-tuning against mass.
        StiffnessAndDamping,     //!< Spring strength given directly as a stiffness (N/m).
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

    //! One differential of a wheeled vehicle or motorcycle (mirrors
    //! JPH::VehicleDifferentialSettings). Several differentials with engine torque
    //! ratios that sum to 1 make an AWD/4WD drivetrain.
    struct JoltVehicleDifferential
    {
        AZ_CLASS_ALLOCATOR(JoltVehicleDifferential, AZ::SystemAllocator);
        AZ_TYPE_INFO(JoltVehicleDifferential, "{5B7E2C90-1D4A-4F6B-8E3C-A9D0F1B2C3E4}");
        static void Reflect(AZ::ReflectContext* context);

        int m_leftWheel = -1;  //!< Index into the wheel list; -1 = no wheel on this side.
        int m_rightWheel = -1; //!< Index into the wheel list; -1 = no wheel on this side.
        float m_differentialRatio = 3.42f; //!< Rotation ratio between gearbox and these wheels.
        float m_leftRightSplit = 0.5f; //!< Torque split across the pair (0 = all left, 1 = all right).
        //! Max/min wheel speed ratio beyond which all torque goes to the slower wheel
        //! (a limited-slip differential). Large values approximate an open differential.
        float m_limitedSlipRatio = 1.4f;
        float m_engineTorqueRatio = 1.0f; //!< Share of engine torque; all differentials should sum to 1.
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
        //! Extra natural length (m) beyond max droop, compressing the spring even at
        //! full extension. Preload stiffens the ride but adds a contact discontinuity.
        float m_suspensionPreloadLength = 0.0f;
        JoltSuspensionSpringMode m_suspensionSpringMode = JoltSuspensionSpringMode::FrequencyAndDamping;
        //! Spring frequency (Hz) - or, in StiffnessAndDamping mode, the stiffness (N/m).
        float m_suspensionFrequency = 1.5f;
        float m_suspensionDamping = 0.7f;    //!< Spring damping (0..1+).
        //! Where tire forces are applied, in chassis space; only used when enabled below.
        AZ::Vector3 m_suspensionForcePoint = AZ::Vector3::CreateZero();
        //! Applies tire forces at the fixed point above instead of at the contact point:
        //! more stable, less accurate against dynamic objects.
        bool m_enableSuspensionForcePoint = false;
        float m_inertia = 0.9f; //!< Wheel moment of inertia (kg m^2); 0.5*m*r^2 for a cylinder.
        float m_angularDamping = 0.2f; //!< Angular damping factor of the wheel's spin.
        float m_maxSteerAngleDegrees = 0.0f; //!< Steering lock in degrees; 0 = non-steering wheel.
        float m_maxBrakeTorque = 500.0f;
        float m_maxHandBrakeTorque = 1000.0f;

        //! Tire friction curves (wheeled/motorcycle only). Each point is (x, friction):
        //! for the longitudinal curve x is the slip ratio, for the lateral curve x is the
        //! slip angle in degrees. Empty keeps Jolt's default curve - the single biggest
        //! handling knob, so most vehicles will want to author these.
        AZStd::vector<AZ::Vector2> m_longitudinalFrictionCurve;
        AZStd::vector<AZ::Vector2> m_lateralFrictionCurve;

        //! Tracked-vehicle tire friction (JPH::WheelSettingsTV uses plain scalars).
        float m_trackedLongitudinalFriction = 4.0f;
        float m_trackedLateralFriction = 2.0f;
    };

    //! Vehicle settings: wheels, engine, transmission and drive layout. Which fields
    //! apply depends on m_vehicleType - the differentials drive named wheels on a
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

        //! The drivetrain. Empty means the legacy single differential below (or, on a
        //! motorcycle, the rear wheel); authoring entries here overrides it and allows
        //! AWD/4WD with per-differential splits and limited slip.
        AZStd::vector<JoltVehicleDifferential> m_differentials;
        //! Limited-slip ratio *between* differentials (AWD center coupling); large
        //! values approximate an open center differential.
        float m_differentialLimitedSlipRatio = 1.4f;

        //! Legacy single-differential fields, kept for data saved before
        //! m_differentials existed and hidden from the inspector. Read only when
        //! m_differentials is empty.
        int m_leftDriveWheel = 2;
        int m_rightDriveWheel = 3;
        float m_differentialRatio = 3.42f;

        // Engine (JPH::VehicleEngineSettings).
        float m_maxEngineTorque = 500.0f; //!< Nm.
        float m_maxEngineRpm = 6000.0f;
        float m_minEngineRpm = 1000.0f; //!< Idle; the engine never drops below this.
        float m_engineInertia = 0.5f; //!< Engine moment of inertia (kg m^2).
        float m_engineAngularDamping = 0.2f;
        //! Torque curve points (x = RPM fraction between min and max, y = fraction of
        //! max torque). Empty keeps Jolt's default curve (80% at idle, 100% at 2/3 RPM).
        AZStd::vector<AZ::Vector2> m_engineTorqueCurve;

        // Transmission (JPH::VehicleTransmissionSettings).
        JoltVehicleTransmissionMode m_transmissionMode = JoltVehicleTransmissionMode::Automatic;
        AZStd::vector<float> m_gearRatios = { 2.66f, 1.78f, 1.3f, 1.0f, 0.74f };
        float m_reverseGearRatio = -2.9f;
        float m_gearSwitchTime = 0.5f; //!< Seconds a gear change takes (automatic mode).
        float m_clutchReleaseTime = 0.3f; //!< Seconds to re-engage the clutch after a shift.
        float m_gearSwitchLatency = 0.5f; //!< Minimum seconds between automatic shifts.
        float m_shiftUpRpm = 4000.0f;
        float m_shiftDownRpm = 2000.0f;
        //! Clutch torque coupling engine to gearbox when fully engaged (k m^2 s^-1).
        float m_clutchStrength = 10.0f;

        //! Motorcycle: the lean-balance spring that keeps the bike upright.
        float m_maxLeanAngleDegrees = 45.0f;
        float m_leanSpringConstant = 5000.0f;
        float m_leanSpringDamping = 1000.0f;
        //! Integration gain on accumulated lean error (helps balance at low speed);
        //! 0 disables the integrator, its decay drains the accumulated error.
        float m_leanSpringIntegrationCoefficient = 0.0f;
        float m_leanSpringIntegrationCoefficientDecay = 4.0f;
        //! How much the target lean angle is smoothed (0 = none, closer to 1 = smoother).
        float m_leanSmoothingFactor = 0.8f;

        //! Tracked: per-track properties. Wheels are assigned to the left or right track
        //! by the sign of their Y position; the driven wheel is the authored index below,
        //! or the first wheel of the track when left at -1.
        float m_trackInertia = 10.0f;           //!< kg m^2, as seen on the driven wheel.
        float m_trackAngularDamping = 0.5f;
        float m_trackMaxBrakeTorque = 15000.0f; //!< Nm on the driven wheel.
        float m_trackDifferentialRatio = 6.0f;
        int m_leftTrackDrivenWheel = -1;  //!< Wheel index driving the left track; -1 = first of its side.
        int m_rightTrackDrivenWheel = -1; //!< Wheel index driving the right track; -1 = first of its side.

        //! Solver iteration overrides for the vehicle constraint (0 = the scene's
        //! defaults). Heavy vehicles resting on light bodies may need more.
        AZ::u32 m_numVelocityStepsOverride = 0;
        AZ::u32 m_numPositionStepsOverride = 0;
        //! Wheel collision-test cadence: run the (expensive) wheel casts every Nth step
        //! while the chassis is active / deactivating. 1 = every step, Jolt's default.
        AZ::u32 m_collisionTestStepsActive = 1;
        AZ::u32 m_collisionTestStepsInactive = 1;

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
