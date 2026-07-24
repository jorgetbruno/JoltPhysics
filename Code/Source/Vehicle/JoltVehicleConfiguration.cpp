#include <Vehicle/JoltVehicleConfiguration.h>

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>

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

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltWheelConfiguration>("Jolt Wheel", "Settings for a single vehicle wheel.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_position,
                        "Position", "Suspension attachment point in chassis space.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_radius,
                        "Radius", "Wheel radius.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.01f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_width,
                        "Width", "Wheel width.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.01f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_suspensionMinLength,
                        "Suspension min length", "Suspension length when fully raised.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_suspensionMaxLength,
                        "Suspension max length", "Suspension length at maximum droop.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_suspensionFrequency,
                        "Suspension frequency", "Suspension spring frequency.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " Hz")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_suspensionDamping,
                        "Suspension damping", "Suspension spring damping ratio (0..1+).")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_maxSteerAngleDegrees,
                        "Max steer angle", "Steering lock; 0 = a non-steering wheel.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 45.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " deg")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_maxBrakeTorque,
                        "Max brake torque", "Torque applied by the foot brake.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " Nm")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_maxHandBrakeTorque,
                        "Max handbrake torque", "Torque applied by the handbrake.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " Nm")
                    ;
            }
        }
    }

    void JoltVehicleConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltVehicleConfiguration>()
                ->Version(1)
                ->Field("Wheels", &JoltVehicleConfiguration::m_wheels)
                ->Field("ChassisMass", &JoltVehicleConfiguration::m_chassisMass)
                ->Field("LeftDriveWheel", &JoltVehicleConfiguration::m_leftDriveWheel)
                ->Field("RightDriveWheel", &JoltVehicleConfiguration::m_rightDriveWheel)
                ->Field("DifferentialRatio", &JoltVehicleConfiguration::m_differentialRatio)
                ->Field("MaxEngineTorque", &JoltVehicleConfiguration::m_maxEngineTorque)
                ->Field("MaxEngineRpm", &JoltVehicleConfiguration::m_maxEngineRpm)
                ->Field("GearRatios", &JoltVehicleConfiguration::m_gearRatios)
                ->Field("ReverseGearRatio", &JoltVehicleConfiguration::m_reverseGearRatio)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltVehicleConfiguration>("Jolt Vehicle Configuration", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                        ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_wheels,
                        "Wheels", "The vehicle's wheels. Drive-wheel indices below refer to positions in this list.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_chassisMass,
                        "Chassis mass", "Absolute chassis mass; 0 keeps the rigid body's own mass.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " kg")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_leftDriveWheel,
                        "Left drive wheel", "Index into Wheels driven by the differential; -1 = none.")
                        ->Attribute(AZ::Edit::Attributes::Min, -1)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_rightDriveWheel,
                        "Right drive wheel", "Index into Wheels driven by the differential; -1 = none.")
                        ->Attribute(AZ::Edit::Attributes::Min, -1)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_differentialRatio,
                        "Differential ratio", "Final drive ratio between the engine and the driven wheels.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_maxEngineTorque,
                        "Max engine torque", "Peak engine torque.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " Nm")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_maxEngineRpm,
                        "Max engine RPM", "Engine redline.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " rpm")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_gearRatios,
                        "Gear ratios", "Forward gear ratios, from first gear to top.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_reverseGearRatio,
                        "Reverse gear ratio", "Reverse gear ratio (negative).")
                    ;
            }
        }
    }
} // namespace JoltPhysics
