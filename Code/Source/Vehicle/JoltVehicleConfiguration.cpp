#include <Vehicle/JoltVehicleConfiguration.h>

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>

namespace JoltPhysics
{
    void JoltWheelConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->RegisterGenericType<AZStd::vector<AZ::Vector2>>();

            serializeContext->Class<JoltWheelConfiguration>()
                ->Version(1)
                ->Field("Position", &JoltWheelConfiguration::m_position)
                ->Field("Radius", &JoltWheelConfiguration::m_radius)
                ->Field("Width", &JoltWheelConfiguration::m_width)
                ->Field("SuspensionMinLength", &JoltWheelConfiguration::m_suspensionMinLength)
                ->Field("SuspensionMaxLength", &JoltWheelConfiguration::m_suspensionMaxLength)
                ->Field("SuspensionPreloadLength", &JoltWheelConfiguration::m_suspensionPreloadLength)
                ->Field("SuspensionSpringMode", &JoltWheelConfiguration::m_suspensionSpringMode)
                ->Field("SuspensionFrequency", &JoltWheelConfiguration::m_suspensionFrequency)
                ->Field("SuspensionDamping", &JoltWheelConfiguration::m_suspensionDamping)
                ->Field("SuspensionForcePoint", &JoltWheelConfiguration::m_suspensionForcePoint)
                ->Field("EnableSuspensionForcePoint", &JoltWheelConfiguration::m_enableSuspensionForcePoint)
                ->Field("Inertia", &JoltWheelConfiguration::m_inertia)
                ->Field("AngularDamping", &JoltWheelConfiguration::m_angularDamping)
                ->Field("MaxSteerAngleDegrees", &JoltWheelConfiguration::m_maxSteerAngleDegrees)
                ->Field("MaxBrakeTorque", &JoltWheelConfiguration::m_maxBrakeTorque)
                ->Field("MaxHandBrakeTorque", &JoltWheelConfiguration::m_maxHandBrakeTorque)
                ->Field("LongitudinalFrictionCurve", &JoltWheelConfiguration::m_longitudinalFrictionCurve)
                ->Field("LateralFrictionCurve", &JoltWheelConfiguration::m_lateralFrictionCurve)
                ->Field("TrackedLongitudinalFriction", &JoltWheelConfiguration::m_trackedLongitudinalFriction)
                ->Field("TrackedLateralFriction", &JoltWheelConfiguration::m_trackedLateralFriction)
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
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_inertia,
                        "Inertia", "Wheel moment of inertia; 0.5 * mass * radius^2 for a solid cylinder.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.001f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " kg m^2")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_angularDamping,
                        "Angular damping", "Damping of the wheel's spin; usually close to 0.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_suspensionMinLength,
                        "Suspension min length", "Suspension length when fully raised.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_suspensionMaxLength,
                        "Suspension max length", "Suspension length at maximum droop.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_suspensionPreloadLength,
                        "Suspension preload", "Extra natural spring length beyond max droop. Preload stiffens "
                        "the ride but makes touching down bouncier (a contact discontinuity).")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &JoltWheelConfiguration::m_suspensionSpringMode,
                        "Suspension spring mode", "Whether the spring strength below is a frequency (self-tuning "
                        "against mass) or a raw stiffness.")
                        ->EnumAttribute(JoltSuspensionSpringMode::FrequencyAndDamping, "Frequency and damping")
                        ->EnumAttribute(JoltSuspensionSpringMode::StiffnessAndDamping, "Stiffness and damping")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_suspensionFrequency,
                        "Suspension frequency", "Spring frequency (Hz) - or stiffness (N/m) in stiffness mode.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_suspensionDamping,
                        "Suspension damping", "Suspension spring damping ratio (0..1+).")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_enableSuspensionForcePoint,
                        "Use suspension force point", "Apply tire forces at the fixed point below instead of at "
                        "the contact point: more stable, less accurate against dynamic objects.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_suspensionForcePoint,
                        "Suspension force point", "Where tire forces are applied, in chassis space. The wheel "
                        "centre in its neutral pose is a good default.")
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
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_longitudinalFrictionCurve,
                        "Longitudinal friction curve", "Tire friction vs slip ratio, as (slip ratio, friction) "
                        "points. Empty keeps Jolt's default. Wheeled and motorcycle only.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_lateralFrictionCurve,
                        "Lateral friction curve", "Tire friction vs slip angle, as (slip angle in degrees, "
                        "friction) points. Empty keeps Jolt's default. Wheeled and motorcycle only.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_trackedLongitudinalFriction,
                        "Tracked longitudinal friction", "Track friction in the rolling direction (tracked "
                        "vehicles use scalars, not curves).")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWheelConfiguration::m_trackedLateralFriction,
                        "Tracked lateral friction", "Track friction sideways (tracked vehicles only).")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
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

    void JoltVehicleDifferential::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltVehicleDifferential>()
                ->Version(1)
                ->Field("LeftWheel", &JoltVehicleDifferential::m_leftWheel)
                ->Field("RightWheel", &JoltVehicleDifferential::m_rightWheel)
                ->Field("DifferentialRatio", &JoltVehicleDifferential::m_differentialRatio)
                ->Field("LeftRightSplit", &JoltVehicleDifferential::m_leftRightSplit)
                ->Field("LimitedSlipRatio", &JoltVehicleDifferential::m_limitedSlipRatio)
                ->Field("EngineTorqueRatio", &JoltVehicleDifferential::m_engineTorqueRatio)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltVehicleDifferential>(
                    "Jolt Differential", "Drives a pair of wheels; several differentials make an AWD drivetrain.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleDifferential::m_leftWheel,
                        "Left wheel", "Index into the wheel list; -1 = no wheel on this side.")
                        ->Attribute(AZ::Edit::Attributes::Min, -1)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleDifferential::m_rightWheel,
                        "Right wheel", "Index into the wheel list; -1 = no wheel on this side.")
                        ->Attribute(AZ::Edit::Attributes::Min, -1)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleDifferential::m_differentialRatio,
                        "Differential ratio", "Rotation ratio between the gearbox and these wheels (final drive).")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleDifferential::m_leftRightSplit,
                        "Left/right split", "Torque split across the pair: 0 = all left, 0.5 = even, 1 = all right.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 1.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleDifferential::m_limitedSlipRatio,
                        "Limited slip ratio", "Max/min wheel speed ratio beyond which all torque goes to the "
                        "slower wheel. Larger is closer to an open differential.")
                        ->Attribute(AZ::Edit::Attributes::Min, 1.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleDifferential::m_engineTorqueRatio,
                        "Engine torque ratio", "Share of the engine's torque; all differentials should sum to 1.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 1.0f)
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
            JoltVehicleDifferential::Reflect(context);

            // Version 3 layout plus additive fields only: the legacy single-differential
            // fields stay serialized (and hidden), so pre-differentials data keeps
            // loading and keeps its meaning without a converter.
            serializeContext->Class<JoltVehicleConfiguration>()
                ->Version(3)
                ->Field("VehicleType", &JoltVehicleConfiguration::m_vehicleType)
                ->Field("CollisionTester", &JoltVehicleConfiguration::m_collisionTester)
                ->Field("AntiRollBars", &JoltVehicleConfiguration::m_antiRollBars)
                ->Field("MaxPitchRollAngleDegrees", &JoltVehicleConfiguration::m_maxPitchRollAngleDegrees)
                ->Field("MaxLeanAngleDegrees", &JoltVehicleConfiguration::m_maxLeanAngleDegrees)
                ->Field("LeanSpringConstant", &JoltVehicleConfiguration::m_leanSpringConstant)
                ->Field("LeanSpringDamping", &JoltVehicleConfiguration::m_leanSpringDamping)
                ->Field("LeanSpringIntegrationCoefficient", &JoltVehicleConfiguration::m_leanSpringIntegrationCoefficient)
                ->Field("LeanSpringIntegrationCoefficientDecay", &JoltVehicleConfiguration::m_leanSpringIntegrationCoefficientDecay)
                ->Field("LeanSmoothingFactor", &JoltVehicleConfiguration::m_leanSmoothingFactor)
                ->Field("TrackInertia", &JoltVehicleConfiguration::m_trackInertia)
                ->Field("TrackAngularDamping", &JoltVehicleConfiguration::m_trackAngularDamping)
                ->Field("TrackMaxBrakeTorque", &JoltVehicleConfiguration::m_trackMaxBrakeTorque)
                ->Field("TrackDifferentialRatio", &JoltVehicleConfiguration::m_trackDifferentialRatio)
                ->Field("LeftTrackDrivenWheel", &JoltVehicleConfiguration::m_leftTrackDrivenWheel)
                ->Field("RightTrackDrivenWheel", &JoltVehicleConfiguration::m_rightTrackDrivenWheel)
                ->Field("Wheels", &JoltVehicleConfiguration::m_wheels)
                ->Field("ChassisMass", &JoltVehicleConfiguration::m_chassisMass)
                ->Field("Differentials", &JoltVehicleConfiguration::m_differentials)
                ->Field("DifferentialLimitedSlipRatio", &JoltVehicleConfiguration::m_differentialLimitedSlipRatio)
                ->Field("LeftDriveWheel", &JoltVehicleConfiguration::m_leftDriveWheel)
                ->Field("RightDriveWheel", &JoltVehicleConfiguration::m_rightDriveWheel)
                ->Field("DifferentialRatio", &JoltVehicleConfiguration::m_differentialRatio)
                ->Field("MaxEngineTorque", &JoltVehicleConfiguration::m_maxEngineTorque)
                ->Field("MaxEngineRpm", &JoltVehicleConfiguration::m_maxEngineRpm)
                ->Field("MinEngineRpm", &JoltVehicleConfiguration::m_minEngineRpm)
                ->Field("EngineInertia", &JoltVehicleConfiguration::m_engineInertia)
                ->Field("EngineAngularDamping", &JoltVehicleConfiguration::m_engineAngularDamping)
                ->Field("EngineTorqueCurve", &JoltVehicleConfiguration::m_engineTorqueCurve)
                ->Field("TransmissionMode", &JoltVehicleConfiguration::m_transmissionMode)
                ->Field("GearRatios", &JoltVehicleConfiguration::m_gearRatios)
                ->Field("ReverseGearRatio", &JoltVehicleConfiguration::m_reverseGearRatio)
                ->Field("GearSwitchTime", &JoltVehicleConfiguration::m_gearSwitchTime)
                ->Field("ClutchReleaseTime", &JoltVehicleConfiguration::m_clutchReleaseTime)
                ->Field("GearSwitchLatency", &JoltVehicleConfiguration::m_gearSwitchLatency)
                ->Field("ShiftUpRpm", &JoltVehicleConfiguration::m_shiftUpRpm)
                ->Field("ShiftDownRpm", &JoltVehicleConfiguration::m_shiftDownRpm)
                ->Field("ClutchStrength", &JoltVehicleConfiguration::m_clutchStrength)
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
                        "their Y position.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_chassisMass,
                        "Chassis mass", "Absolute chassis mass; 0 keeps the rigid body's own mass.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " kg")

                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_differentials,
                        "Differentials", "The drivetrain: each differential drives a pair of wheels with its own "
                        "final drive, torque split and limited slip. Several with torque ratios summing to 1 make "
                        "an AWD layout. Empty drives the rear axle (wheels 2 and 3; a motorcycle its rear wheel).")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltVehicleConfiguration::GetWheeledSettingsVisibility)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_differentialLimitedSlipRatio,
                        "Center limited slip ratio", "Limited-slip coupling between differentials (an AWD center "
                        "differential). Larger is closer to open.")
                        ->Attribute(AZ::Edit::Attributes::Min, 1.0f)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltVehicleConfiguration::GetWheeledSettingsVisibility)

                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_maxEngineTorque,
                        "Max engine torque", "Peak engine torque.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " Nm")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_maxEngineRpm,
                        "Max engine RPM", "Engine redline.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " rpm")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_minEngineRpm,
                        "Min engine RPM", "Idle; the engine never drops below this.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " rpm")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_engineInertia,
                        "Engine inertia", "Moment of inertia of the engine's rotating mass.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.001f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " kg m^2")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_engineAngularDamping,
                        "Engine angular damping", "Damping of the engine's spin; usually close to 0.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_engineTorqueCurve,
                        "Engine torque curve", "Points of (RPM fraction between min and max, fraction of max "
                        "torque). Empty keeps Jolt's default curve.")

                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &JoltVehicleConfiguration::m_transmissionMode,
                        "Transmission", "Automatic shifts by the RPM thresholds below; Manual only changes gear "
                        "through the vehicle bus (SetGear).")
                        ->EnumAttribute(JoltVehicleTransmissionMode::Automatic, "Automatic")
                        ->EnumAttribute(JoltVehicleTransmissionMode::Manual, "Manual")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_gearRatios,
                        "Gear ratios", "Forward gear ratios, from first gear to top.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_reverseGearRatio,
                        "Reverse gear ratio", "Reverse gear ratio (negative).")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_gearSwitchTime,
                        "Gear switch time", "How long a gear change takes (automatic mode).")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " s")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_clutchReleaseTime,
                        "Clutch release time", "How long the clutch takes to re-engage after a shift.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " s")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_gearSwitchLatency,
                        "Gear switch latency", "Minimum time between automatic shifts.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " s")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_shiftUpRpm,
                        "Shift up RPM", "Automatic mode shifts up above this engine speed.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " rpm")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_shiftDownRpm,
                        "Shift down RPM", "Automatic mode shifts down below this engine speed.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " rpm")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_clutchStrength,
                        "Clutch strength", "Torque coupling between engine and gearbox when fully engaged.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)

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
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_leanSpringIntegrationCoefficient,
                        "Lean integration coefficient", "Gain on accumulated lean error; helps balance at low "
                        "speed. 0 disables the integrator.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltVehicleConfiguration::GetMotorcycleSettingsVisibility)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_leanSpringIntegrationCoefficientDecay,
                        "Lean integration decay", "How quickly the accumulated lean error drains.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltVehicleConfiguration::GetMotorcycleSettingsVisibility)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_leanSmoothingFactor,
                        "Lean smoothing", "How much the target lean angle is smoothed (0 = none, near 1 = heavy).")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 1.0f)
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
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_leftTrackDrivenWheel,
                        "Left track driven wheel", "Wheel index driving the left track; -1 = first wheel of the side.")
                        ->Attribute(AZ::Edit::Attributes::Min, -1)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltVehicleConfiguration::GetTrackedSettingsVisibility)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleConfiguration::m_rightTrackDrivenWheel,
                        "Right track driven wheel", "Wheel index driving the right track; -1 = first wheel of the side.")
                        ->Attribute(AZ::Edit::Attributes::Min, -1)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltVehicleConfiguration::GetTrackedSettingsVisibility)
                    ;
            }
        }
    }
} // namespace JoltPhysics
