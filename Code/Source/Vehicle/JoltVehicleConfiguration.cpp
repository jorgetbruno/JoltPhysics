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

    void JoltVehicleAntiRollBar::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltVehicleAntiRollBar>()
                ->Version(1)
                ->Field("LeftWheel", &JoltVehicleAntiRollBar::m_leftWheel)
                ->Field("RightWheel", &JoltVehicleAntiRollBar::m_rightWheel)
                ->Field("Stiffness", &JoltVehicleAntiRollBar::m_stiffness)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltVehicleAntiRollBar>(
                    "Jolt Anti-Roll Bar", "Couples two wheels' suspension so the chassis leans less in a corner.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleAntiRollBar::m_leftWheel,
                        "Left wheel", "Index into the wheel list.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleAntiRollBar::m_rightWheel,
                        "Right wheel", "Index into the wheel list.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleAntiRollBar::m_stiffness,
                        "Stiffness", "Spring constant coupling the two wheels; 0 disables this bar.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " N/m")
                    ;
            }
        }
    }

    AZ::Crc32 JoltVehicleConfiguration::GetWheeledSettingsVisibility() const
    {
        // Engine drive layout: a tracked vehicle drives tracks, not named wheels.
        return m_vehicleType == JoltVehicleType::Tracked ? AZ::Edit::PropertyVisibility::Hide
                                                         : AZ::Edit::PropertyVisibility::Show;
    }

    AZ::Crc32 JoltVehicleConfiguration::GetMotorcycleSettingsVisibility() const
    {
        return m_vehicleType == JoltVehicleType::Motorcycle ? AZ::Edit::PropertyVisibility::Show
                                                            : AZ::Edit::PropertyVisibility::Hide;
    }

    AZ::Crc32 JoltVehicleConfiguration::GetTrackedSettingsVisibility() const
    {
        return m_vehicleType == JoltVehicleType::Tracked ? AZ::Edit::PropertyVisibility::Show
                                                         : AZ::Edit::PropertyVisibility::Hide;
    }

    void JoltVehicleConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            JoltVehicleAntiRollBar::Reflect(context);

            serializeContext->Class<JoltVehicleConfiguration>()
                ->Version(3)
                ->Field("VehicleType", &JoltVehicleConfiguration::m_vehicleType)
                ->Field("CollisionTester", &JoltVehicleConfiguration::m_collisionTester)
                ->Field("AntiRollBars", &JoltVehicleConfiguration::m_antiRollBars)
                ->Field("MaxPitchRollAngleDegrees", &JoltVehicleConfiguration::m_maxPitchRollAngleDegrees)
                ->Field("MaxLeanAngleDegrees", &JoltVehicleConfiguration::m_maxLeanAngleDegrees)
                ->Field("LeanSpringConstant", &JoltVehicleConfiguration::m_leanSpringConstant)
                ->Field("LeanSpringDamping", &JoltVehicleConfiguration::m_leanSpringDamping)
                ->Field("TrackInertia", &JoltVehicleConfiguration::m_trackInertia)
                ->Field("TrackAngularDamping", &JoltVehicleConfiguration::m_trackAngularDamping)
                ->Field("TrackMaxBrakeTorque", &JoltVehicleConfiguration::m_trackMaxBrakeTorque)
                ->Field("TrackDifferentialRatio", &JoltVehicleConfiguration::m_trackDifferentialRatio)
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
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &JoltVehicleConfiguration::m_vehicleType,
                        "Vehicle type", "Which Jolt controller drives the chassis. Only the settings that apply "
                        "to the selected type are shown.")
                        ->EnumAttribute(JoltVehicleType::Wheeled, "Wheeled (car)")
                        ->EnumAttribute(JoltVehicleType::Motorcycle, "Motorcycle")
                        ->EnumAttribute(JoltVehicleType::Tracked, "Tracked (tank)")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &JoltVehicleConfiguration::m_collisionTester,
                        "Ground detection", "How each wheel looks for the ground. A ray only tests the wheel's "
                        "centre line, so it drops into gaps a real tyre would roll over.")
                        ->EnumAttribute(JoltVehicleCollisionTester::Ray, "Ray (cheapest)")
                        ->EnumAttribute(JoltVehicleCollisionTester::Sphere, "Sphere cast")
                        ->EnumAttribute(JoltVehicleCollisionTester::Cylinder, "Cylinder cast (most accurate)")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_antiRollBars,
                        "Anti-roll bars", "Each couples two wheels' suspension so the chassis leans less in a "
                        "corner. Usually one per axle.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_maxPitchRollAngleDegrees,
                        "Max pitch/roll angle", "How far the chassis may tip away from world up before the "
                        "suspension stops pushing. 180 never gives up, which lets the vehicle drive on its roof.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 180.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " deg")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_wheels,
                        "Wheels", "The vehicle's wheels. Leave empty for a default layout for the selected type. "
                        "On a tracked vehicle the wheels are split into a left and a right track by the sign of "
                        "their Y position, and the first wheel of each track drives it.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_chassisMass,
                        "Chassis mass", "Absolute chassis mass; 0 keeps the rigid body's own mass.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " kg")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_leftDriveWheel,
                        "Left drive wheel", "Index into Wheels driven by the differential; -1 = none.")
                        ->Attribute(AZ::Edit::Attributes::Min, -1)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltVehicleConfiguration::GetWheeledSettingsVisibility)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_rightDriveWheel,
                        "Right drive wheel", "Index into Wheels driven by the differential; -1 = none.")
                        ->Attribute(AZ::Edit::Attributes::Min, -1)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltVehicleConfiguration::GetWheeledSettingsVisibility)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_differentialRatio,
                        "Differential ratio", "Final drive ratio between the engine and the driven wheels.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltVehicleConfiguration::GetWheeledSettingsVisibility)
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

                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_maxLeanAngleDegrees,
                        "Max lean angle", "How far the motorcycle leans into turns.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 89.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " deg")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltVehicleConfiguration::GetMotorcycleSettingsVisibility)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_leanSpringConstant,
                        "Lean spring constant", "Strength of the spring that keeps the motorcycle upright.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltVehicleConfiguration::GetMotorcycleSettingsVisibility)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_leanSpringDamping,
                        "Lean spring damping", "Damping of the lean spring.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltVehicleConfiguration::GetMotorcycleSettingsVisibility)

                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_trackInertia,
                        "Track inertia", "Moment of inertia of a track and its wheels, as seen on the driven wheel.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " kg m^2")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltVehicleConfiguration::GetTrackedSettingsVisibility)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_trackAngularDamping,
                        "Track angular damping", "Damping applied to a track's rotation.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltVehicleConfiguration::GetTrackedSettingsVisibility)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_trackMaxBrakeTorque,
                        "Track max brake torque", "Braking torque available on a track's driven wheel.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " Nm")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltVehicleConfiguration::GetTrackedSettingsVisibility)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_trackDifferentialRatio,
                        "Track differential ratio", "Ratio between gearbox and track driven-wheel rotation.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltVehicleConfiguration::GetTrackedSettingsVisibility)
                    ;
            }
        }
    }
} // namespace JoltPhysics
