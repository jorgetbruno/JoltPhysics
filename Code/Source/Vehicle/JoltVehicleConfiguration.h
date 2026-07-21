#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/std/containers/vector.h>

namespace JoltPhysics
{
    //! One wheel of a Jolt vehicle (mirrors the useful subset of JPH::WheelSettingsWV).
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

    //! 4-wheel-drive-capable vehicle settings: wheels, engine, transmission and the
    //! driven-wheel differential (mirrors the PhysXVehicle gem's configuration shape).
    struct JoltVehicleConfiguration
    {
        AZ_CLASS_ALLOCATOR(JoltVehicleConfiguration, AZ::SystemAllocator);
        AZ_TYPE_INFO(JoltVehicleConfiguration, "{B8C9D0E1-F2A3-4456-B7C8-D9E0F1A2B3C4}");
        static void Reflect(AZ::ReflectContext* context);

        AZStd::vector<JoltWheelConfiguration> m_wheels;
        int m_leftDriveWheel = 2;    //!< Wheel index driven by the differential (-1 = none).
        int m_rightDriveWheel = 3;   //!< Wheel index driven by the differential (-1 = none).
        float m_differentialRatio = 3.42f;
        float m_maxEngineTorque = 500.0f; //!< Nm.
        float m_maxEngineRpm = 6000.0f;
        AZStd::vector<float> m_gearRatios = { 2.66f, 1.78f, 1.3f, 1.0f, 0.74f };
        float m_reverseGearRatio = -2.9f;
    };
} // namespace JoltPhysics
