#include <Vehicle/JoltVehicleConfiguration.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace JoltPhysics
{
    void JoltWheelConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltWheelConfiguration>()
                ->Version(1)
                ->Field("Position", &JoltWheelConfiguration::m_position)
                ->Field("Radius", &JoltWheelConfiguration::m_radius)
                ->Field("Width", &JoltWheelConfiguration::m_width)
                ->Field("SuspensionMinLength", &JoltWheelConfiguration::m_suspensionMinLength)
                ->Field("SuspensionMaxLength", &JoltWheelConfiguration::m_suspensionMaxLength)
                ->Field("SuspensionFrequency", &JoltWheelConfiguration::m_suspensionFrequency)
                ->Field("SuspensionDamping", &JoltWheelConfiguration::m_suspensionDamping)
                ->Field("MaxSteerAngleDegrees", &JoltWheelConfiguration::m_maxSteerAngleDegrees)
                ->Field("MaxBrakeTorque", &JoltWheelConfiguration::m_maxBrakeTorque)
                ->Field("MaxHandBrakeTorque", &JoltWheelConfiguration::m_maxHandBrakeTorque)
                ;
        }
    }

    void JoltVehicleConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltVehicleConfiguration>()
                ->Version(1)
                ->Field("Wheels", &JoltVehicleConfiguration::m_wheels)
                ->Field("LeftDriveWheel", &JoltVehicleConfiguration::m_leftDriveWheel)
                ->Field("RightDriveWheel", &JoltVehicleConfiguration::m_rightDriveWheel)
                ->Field("DifferentialRatio", &JoltVehicleConfiguration::m_differentialRatio)
                ->Field("MaxEngineTorque", &JoltVehicleConfiguration::m_maxEngineTorque)
                ->Field("MaxEngineRpm", &JoltVehicleConfiguration::m_maxEngineRpm)
                ->Field("GearRatios", &JoltVehicleConfiguration::m_gearRatios)
                ->Field("ReverseGearRatio", &JoltVehicleConfiguration::m_reverseGearRatio)
                ;
        }
    }
} // namespace JoltPhysics
