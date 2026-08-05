#include <Vehicle/JoltVehicle.h>

#include <Scene/JoltScene.h>
#include <System/CollisionLayerFilters.h>
#include <Utils/Conversions.h>
#include <Vehicle/JoltVehicleConfiguration.h>

#include <AzCore/Math/MathUtils.h>
#include <AzCore/std/algorithm.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Vehicle/MotorcycleController.h>
#include <Jolt/Physics/Vehicle/TrackedVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/VehicleDifferential.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

#include <Utils/JoltDiagnostics.h>

namespace JoltPhysics
{
    namespace
    {
        //! Fills in the base (type-independent) wheel settings from a wheel configuration.
        void ApplyCommonWheelSettings(const JoltWheelConfiguration& wheelConfig, JPH::WheelSettings& wheelSettings)
        {
            wheelSettings.mPosition = Conversions::ToJolt(wheelConfig.m_position);
            wheelSettings.mSuspensionDirection = JPH::Vec3(0.0f, 0.0f, -1.0f);
            wheelSettings.mSteeringAxis = JPH::Vec3::sAxisZ();
            wheelSettings.mWheelUp = JPH::Vec3::sAxisZ();
            wheelSettings.mWheelForward = JPH::Vec3::sAxisX();
            wheelSettings.mRadius = wheelConfig.m_radius;
            wheelSettings.mWidth = wheelConfig.m_width;
            wheelSettings.mSuspensionMinLength = wheelConfig.m_suspensionMinLength;
            wheelSettings.mSuspensionMaxLength = wheelConfig.m_suspensionMaxLength;
            wheelSettings.mSuspensionPreloadLength = wheelConfig.m_suspensionPreloadLength;
            // In stiffness mode the frequency field carries the stiffness directly (the
            // two are one SpringSettings value in Jolt, interpreted by the mode).
            wheelSettings.mSuspensionSpring = JPH::SpringSettings(
                wheelConfig.m_suspensionSpringMode == JoltSuspensionSpringMode::StiffnessAndDamping
                    ? JPH::ESpringMode::StiffnessAndDamping
                    : JPH::ESpringMode::FrequencyAndDamping,
                wheelConfig.m_suspensionFrequency, wheelConfig.m_suspensionDamping);
            wheelSettings.mSuspensionForcePoint = Conversions::ToJolt(wheelConfig.m_suspensionForcePoint);
            wheelSettings.mEnableSuspensionForcePoint = wheelConfig.m_enableSuspensionForcePoint;
        }

        //! Replaces a Jolt curve with authored points; an empty list keeps Jolt's default
        //! curve, so an unedited vehicle behaves exactly as before.
        void ApplyCurve(const AZStd::vector<AZ::Vector2>& points, JPH::LinearCurve& curve)
        {
            if (points.empty())
            {
                return;
            }
            curve.Clear();
            curve.Reserve(static_cast<JPH::uint>(points.size()));
            for (const AZ::Vector2& point : points)
            {
                curve.AddPoint(point.GetX(), point.GetY());
            }
            curve.Sort();
        }

        //! Engine and transmission settings shared by all three controller types.
        void ApplyEngineAndTransmission(
            const JoltVehicleConfiguration& configuration,
            JPH::VehicleEngineSettings& engine,
            JPH::VehicleTransmissionSettings& transmission)
        {
            engine.mMaxTorque = configuration.m_maxEngineTorque;
            engine.mMaxRPM = configuration.m_maxEngineRpm;
            engine.mMinRPM = configuration.m_minEngineRpm;
            engine.mInertia = configuration.m_engineInertia;
            engine.mAngularDamping = configuration.m_engineAngularDamping;
            ApplyCurve(configuration.m_engineTorqueCurve, engine.mNormalizedTorque);

            transmission.mMode = configuration.m_transmissionMode == JoltVehicleTransmissionMode::Manual
                ? JPH::ETransmissionMode::Manual
                : JPH::ETransmissionMode::Auto;
            transmission.mGearRatios.assign(configuration.m_gearRatios.begin(), configuration.m_gearRatios.end());
            transmission.mReverseGearRatios.assign({ configuration.m_reverseGearRatio });
            transmission.mSwitchTime = configuration.m_gearSwitchTime;
            transmission.mClutchReleaseTime = configuration.m_clutchReleaseTime;
            transmission.mSwitchLatency = configuration.m_gearSwitchLatency;
            transmission.mShiftUpRPM = configuration.m_shiftUpRpm;
            transmission.mShiftDownRPM = configuration.m_shiftDownRpm;
            transmission.mClutchStrength = configuration.m_clutchStrength;
        }

        //! The drivetrain to build: the authored differential list, or one synthesized
        //! from the legacy drive-wheel fields when the list is empty (which is also what
        //! data saved before the list existed loads as).
        AZStd::vector<JoltVehicleDifferential> EffectiveDifferentials(const JoltVehicleConfiguration& configuration)
        {
            if (!configuration.m_differentials.empty())
            {
                return configuration.m_differentials;
            }

            JoltVehicleDifferential differential;
            differential.m_leftWheel = configuration.m_leftDriveWheel;
            differential.m_rightWheel = configuration.m_rightDriveWheel;
            differential.m_differentialRatio = configuration.m_differentialRatio;

            // A motorcycle has a single driven (rear) wheel; the legacy defaults still
            // point at a car's rear axle (wheels 2 and 3), which a two-wheeler does not
            // have, so drive its rear wheel instead.
            if (configuration.m_vehicleType == JoltVehicleType::Motorcycle &&
                configuration.m_leftDriveWheel == JoltVehicleConfiguration().m_leftDriveWheel &&
                configuration.m_rightDriveWheel == JoltVehicleConfiguration().m_rightDriveWheel)
            {
                differential.m_leftWheel = -1;
                differential.m_rightWheel = static_cast<int>(configuration.m_wheels.size()) - 1;
            }

            return { differential };
        }

        //! The default wheel layout for a vehicle type, used when none is authored.
        AZStd::vector<JoltWheelConfiguration> MakeDefaultWheels(JoltVehicleType vehicleType)
        {
            AZStd::vector<JoltWheelConfiguration> wheels;

            auto addWheel = [&wheels](float x, float y, float steerDegrees, float radius)
            {
                JoltWheelConfiguration wheel;
                wheel.m_position = AZ::Vector3(x, y, -0.2f);
                wheel.m_maxSteerAngleDegrees = steerDegrees;
                wheel.m_radius = radius;
                wheels.push_back(wheel);
            };

            switch (vehicleType)
            {
            case JoltVehicleType::Motorcycle:
                // Front wheel steers, rear wheel drives; both on the centre line.
                addWheel(0.75f, 0.0f, 30.0f, 0.31f);
                addWheel(-0.75f, 0.0f, 0.0f, 0.31f);
                break;

            case JoltVehicleType::Tracked:
                // Four road wheels per side; +Y is the left track, -Y the right one.
                for (const float side : { 0.7f, -0.7f })
                {
                    for (const float x : { 1.2f, 0.4f, -0.4f, -1.2f })
                    {
                        addWheel(x, side, 0.0f, 0.3f);
                    }
                }
                break;

            case JoltVehicleType::Wheeled:
            default:
                // Front axle steers, rear axle drives.
                addWheel(0.8f, 0.45f, 35.0f, 0.35f);
                addWheel(0.8f, -0.45f, 35.0f, 0.35f);
                addWheel(-0.8f, 0.45f, 0.0f, 0.35f);
                addWheel(-0.8f, -0.45f, 0.0f, 0.35f);
                break;
            }

            return wheels;
        }

        //! The motorcycle lean spring applies a roll impulse of roughly
        //! (constant * angle - damping * rate), so its gains have to be sized to the
        //! chassis roll inertia. Jolt's defaults suit a particular bike; left on a much
        //! lighter one they overcorrect hard enough to throw it into the air, which is
        //! hard to attribute to a spring constant. Warn with a workable value instead.
        void WarnOnImplausibleLeanGains(const JoltVehicleConfiguration& configuration, const JPH::Body& chassisBody)
        {
            const JPH::MotionProperties* motionProperties = chassisBody.GetMotionProperties();
            if (!motionProperties)
            {
                return;
            }

            // The lean impulse is applied about the chassis forward axis, so the inertia
            // that governs the response is the effective one along that axis:
            // an angular impulse L about n produces n . (I^-1 n) * L of spin about n.
            // (The inverse inertia diagonal is expressed in the body's principal frame,
            // which is not the local axis order, so it cannot be indexed directly.)
            const JPH::Vec3 forwardAxis = JPH::Vec3::sAxisX();
            const JPH::Mat44 localInverseInertia = motionProperties->GetLocalSpaceInverseInertia();
            const float inverseRollInertia = forwardAxis.Dot(localInverseInertia.Multiply3x3(forwardAxis));
            if (inverseRollInertia <= 0.0f)
            {
                return;
            }
            const float rollInertia = 1.0f / inverseRollInertia;

            // Real motorcycle lean dynamics sit near 1 Hz; measured against this chassis,
            // a ~1.9 Hz response balances cleanly while ~3.9 Hz throws the bike into the
            // air, so anything above 3 Hz is worth reporting.
            constexpr float maxReasonableFrequencyHz = 3.0f;
            constexpr float suggestedFrequencyHz = 2.0f;
            const float frequencyHz =
                sqrtf(configuration.m_leanSpringConstant / rollInertia) / (2.0f * AZ::Constants::Pi);

            const float suggestedConstant =
                rollInertia * (2.0f * AZ::Constants::Pi * suggestedFrequencyHz) * (2.0f * AZ::Constants::Pi * suggestedFrequencyHz);
            AZ_Warning("JoltPhysics", frequencyHz <= maxReasonableFrequencyHz,
                "Motorcycle%s: lean spring constant %.0f is very stiff for this chassis (roll inertia %.1f kg m^2, "
                "implying a %.1f Hz lean response); the balance correction is likely to throw the bike around. "
                "Try a lean spring constant near %.0f with damping near %.0f.",
                Internal::NameClause(configuration.m_debugName).c_str(),
                configuration.m_leanSpringConstant, rollInertia, frequencyHz, suggestedConstant,
                2.0f * rollInertia * (2.0f * AZ::Constants::Pi * suggestedFrequencyHz));
        }

        //! Jolt asserts on a track ratio of exactly zero (it would mean "no drive at all"),
        //! so steering that lands on zero is nudged to a negligible non-zero value.
        float SanitizeTrackRatio(float ratio)
        {
            constexpr float minimumRatio = 0.01f;
            if (AZStd::abs(ratio) >= minimumRatio)
            {
                return ratio;
            }
            return ratio < 0.0f ? -minimumRatio : minimumRatio;
        }
    }

    JoltVehicle::JoltVehicle(const JoltVehicleConfiguration& configuration, JoltScene* scene, JPH::Body* chassisBody)
        : m_scene(scene)
        , m_chassisBody(chassisBody)
        , m_vehicleType(configuration.m_vehicleType)
    {
        if (!m_scene || !m_scene->GetJoltPhysicsSystem() || !m_chassisBody)
        {
            return;
        }

        if (configuration.m_chassisMass > 0.0f)
        {
            if (JPH::MotionProperties* motionProperties = m_chassisBody->GetMotionProperties())
            {
                motionProperties->ScaleToMass(configuration.m_chassisMass);
            }
        }

        JoltVehicleConfiguration effectiveConfiguration = configuration;
        if (effectiveConfiguration.m_wheels.empty())
        {
            effectiveConfiguration.m_wheels = MakeDefaultWheels(m_vehicleType);
        }

        JPH::VehicleConstraintSettings settings;
        settings.mUp = JPH::Vec3::sAxisZ();
        settings.mForward = JPH::Vec3::sAxisX();
        settings.mMaxPitchRollAngle = AZ::DegToRad(effectiveConfiguration.m_maxPitchRollAngleDegrees);

        if (m_vehicleType == JoltVehicleType::Tracked)
        {
            BuildTrackedSettings(effectiveConfiguration, settings);
        }
        else
        {
            BuildWheeledSettings(effectiveConfiguration, settings);
        }

        if (settings.mWheels.empty() || settings.mController == nullptr)
        {
            return;
        }

        BuildAntiRollBars(effectiveConfiguration, settings);

        if (m_vehicleType == JoltVehicleType::Motorcycle)
        {
            WarnOnImplausibleLeanGains(effectiveConfiguration, *m_chassisBody);
        }

        settings.mNumVelocityStepsOverride = effectiveConfiguration.m_numVelocityStepsOverride;
        settings.mNumPositionStepsOverride = effectiveConfiguration.m_numPositionStepsOverride;

        m_constraint = new JPH::VehicleConstraint(*m_chassisBody, settings);
        m_constraint->SetVehicleCollisionTester(CreateCollisionTester(effectiveConfiguration));
        m_constraint->SetNumStepsBetweenCollisionTestActive(
            AZStd::max<AZ::u32>(effectiveConfiguration.m_collisionTestStepsActive, 1));
        m_constraint->SetNumStepsBetweenCollisionTestInactive(
            AZStd::max<AZ::u32>(effectiveConfiguration.m_collisionTestStepsInactive, 1));

        m_scene->GetJoltPhysicsSystem()->AddConstraint(m_constraint);
        m_scene->GetJoltPhysicsSystem()->AddStepListener(m_constraint);

        if (m_vehicleType == JoltVehicleType::Tracked)
        {
            m_trackedController = static_cast<JPH::TrackedVehicleController*>(m_constraint->GetController());
        }
        else
        {
            // MotorcycleController derives from WheeledVehicleController, so both types
            // share the wheeled driver input and engine/transmission readouts.
            m_wheeledController = static_cast<JPH::WheeledVehicleController*>(m_constraint->GetController());
        }
    }

    void JoltVehicle::BuildWheeledSettings(
        const JoltVehicleConfiguration& configuration, JPH::VehicleConstraintSettings& settings)
    {
        for (const JoltWheelConfiguration& wheelConfig : configuration.m_wheels)
        {
            auto* wheelSettings = new JPH::WheelSettingsWV;
            ApplyCommonWheelSettings(wheelConfig, *wheelSettings);
            wheelSettings->mMaxSteerAngle = AZ::DegToRad(wheelConfig.m_maxSteerAngleDegrees);
            wheelSettings->mMaxBrakeTorque = wheelConfig.m_maxBrakeTorque;
            wheelSettings->mMaxHandBrakeTorque = wheelConfig.m_maxHandBrakeTorque;
            wheelSettings->mInertia = wheelConfig.m_inertia;
            wheelSettings->mAngularDamping = wheelConfig.m_angularDamping;
            ApplyCurve(wheelConfig.m_longitudinalFrictionCurve, wheelSettings->mLongitudinalFriction);
            ApplyCurve(wheelConfig.m_lateralFrictionCurve, wheelSettings->mLateralFriction);
            settings.mWheels.push_back(wheelSettings);
        }

        // A motorcycle is a wheeled vehicle plus a lean-balance spring, so its settings
        // derive from the wheeled ones and the engine/transmission setup below is shared.
        JPH::WheeledVehicleControllerSettings* controllerSettings = nullptr;
        if (configuration.m_vehicleType == JoltVehicleType::Motorcycle)
        {
            auto* motorcycleSettings = new JPH::MotorcycleControllerSettings;
            motorcycleSettings->mMaxLeanAngle = AZ::DegToRad(configuration.m_maxLeanAngleDegrees);
            motorcycleSettings->mLeanSpringConstant = configuration.m_leanSpringConstant;
            motorcycleSettings->mLeanSpringDamping = configuration.m_leanSpringDamping;
            motorcycleSettings->mLeanSpringIntegrationCoefficient = configuration.m_leanSpringIntegrationCoefficient;
            motorcycleSettings->mLeanSpringIntegrationCoefficientDecay = configuration.m_leanSpringIntegrationCoefficientDecay;
            motorcycleSettings->mLeanSmoothingFactor = configuration.m_leanSmoothingFactor;
            controllerSettings = motorcycleSettings;
        }
        else
        {
            controllerSettings = new JPH::WheeledVehicleControllerSettings;
        }

        ApplyEngineAndTransmission(configuration, controllerSettings->mEngine, controllerSettings->mTransmission);
        controllerSettings->mDifferentialLimitedSlipRatio = configuration.m_differentialLimitedSlipRatio;

        const int wheelCount = static_cast<int>(configuration.m_wheels.size());
        for (const JoltVehicleDifferential& differentialConfig : EffectiveDifferentials(configuration))
        {
            if (differentialConfig.m_leftWheel >= wheelCount || differentialConfig.m_rightWheel >= wheelCount)
            {
                AZ_Warning("JoltPhysics", false,
                    "Vehicle%s has a drive wheel index out of range (left %d, right %d, %d wheels); this "
                    "differential is left unconnected.",
                    Internal::NameClause(configuration.m_debugName).c_str(),
                    differentialConfig.m_leftWheel, differentialConfig.m_rightWheel, wheelCount);
                continue;
            }
            if (differentialConfig.m_leftWheel < 0 && differentialConfig.m_rightWheel < 0)
            {
                continue;
            }

            JPH::VehicleDifferentialSettings differential;
            differential.mLeftWheel = differentialConfig.m_leftWheel;
            differential.mRightWheel = differentialConfig.m_rightWheel;
            differential.mDifferentialRatio = differentialConfig.m_differentialRatio;
            differential.mLeftRightSplit = differentialConfig.m_leftRightSplit;
            differential.mLimitedSlipRatio = differentialConfig.m_limitedSlipRatio;
            differential.mEngineTorqueRatio = differentialConfig.m_engineTorqueRatio;
            controllerSettings->mDifferentials.push_back(differential);
        }

        // Jolt's wheeled controller sums the engine torque ratio over the differentials it
        // was given and asserts that the total is 1
        // (`WheeledVehicleController::PreCollide`). Zero differentials sums to zero, so an
        // unconnected vehicle does not merely fail to drive - it reports that assertion on
        // every step, for as long as the level runs, from a stack that says nothing about
        // wheel indices. Give it the wheels the vehicle actually has instead.
        if (controllerSettings->mDifferentials.empty() && wheelCount > 0)
        {
            JPH::VehicleDifferentialSettings fallback;
            fallback.mLeftWheel = wheelCount > 1 ? wheelCount - 2 : -1;
            fallback.mRightWheel = wheelCount - 1;
            fallback.mDifferentialRatio = configuration.m_differentialRatio;
            controllerSettings->mDifferentials.push_back(fallback);

            AZ_Warning("JoltPhysics", false,
                "Vehicle%s had no differential connected to a wheel that exists, so it is driving wheels %d and %d "
                "instead. Set the drive wheel indices to wheels this vehicle has - the defaults name a four-wheeled "
                "car's rear axle, which is wrong for any other layout.",
                Internal::NameClause(configuration.m_debugName).c_str(), fallback.mLeftWheel, fallback.mRightWheel);
        }
        else if (controllerSettings->mDifferentials.empty())
        {
            AZ_Warning("JoltPhysics", false,
                "Vehicle%s has no wheels, so it has nothing to drive.",
                Internal::NameClause(configuration.m_debugName).c_str());
        }

        // The same invariant, from the other side: the ratios have to sum to 1 across the
        // differentials that were *connected*, not the ones that were authored. Dropping an
        // unconnected differential above silently breaks a split that added up in the
        // editor, and two differentials both left at the default 1.0 never added up at all.
        float torqueRatioSum = 0.0f;
        for (const JPH::VehicleDifferentialSettings& differential : controllerSettings->mDifferentials)
        {
            torqueRatioSum += differential.mEngineTorqueRatio;
        }
        if (!controllerSettings->mDifferentials.empty() && AZ::GetAbs(torqueRatioSum - 1.0f) > 1.0e-6f)
        {
            AZ_Warning("JoltPhysics", false,
                "Vehicle%s splits %.2f of the engine's torque across its connected differentials; Jolt requires "
                "exactly 1. Rescaling to keep their proportions. Check the Engine Torque Ratio on each differential.",
                Internal::NameClause(configuration.m_debugName).c_str(), torqueRatioSum);

            const float scale = torqueRatioSum > 1.0e-6f
                ? 1.0f / torqueRatioSum
                : 1.0f / static_cast<float>(controllerSettings->mDifferentials.size());
            for (JPH::VehicleDifferentialSettings& differential : controllerSettings->mDifferentials)
            {
                differential.mEngineTorqueRatio =
                    torqueRatioSum > 1.0e-6f ? differential.mEngineTorqueRatio * scale : scale;
            }
        }

        settings.mController = controllerSettings;
    }

    JPH::VehicleCollisionTester* JoltVehicle::CreateCollisionTester(const JoltVehicleConfiguration& configuration) const
    {
        // The up axis matters: the ray and sphere testers use it to reject near-vertical
        // hits, and their defaults are Y-up, which would treat O3DE's ground as a wall.
        // The cylinder tester takes no up vector - it casts the wheel shape itself and
        // reads the orientation off the constraint.
        const JPH::Vec3 up = Conversions::ToJolt(AZ::Vector3::CreateAxisZ());

        // The widest wheel makes the best sphere: too small and the sphere falls into the
        // gaps the ray already falls into.
        float largestRadius = 0.0f;
        for (const JoltWheelConfiguration& wheel : configuration.m_wheels)
        {
            largestRadius = AZStd::max(largestRadius, wheel.m_radius);
        }

        switch (configuration.m_collisionTester)
        {
        case JoltVehicleCollisionTester::Sphere:
            return new JPH::VehicleCollisionTesterCastSphere(ObjectLayers::Moving, largestRadius, up);
        case JoltVehicleCollisionTester::Cylinder:
            return new JPH::VehicleCollisionTesterCastCylinder(ObjectLayers::Moving);
        case JoltVehicleCollisionTester::Ray:
        default:
            return new JPH::VehicleCollisionTesterRay(ObjectLayers::Moving, up);
        }
    }

    void JoltVehicle::BuildAntiRollBars(
        const JoltVehicleConfiguration& configuration, JPH::VehicleConstraintSettings& settings) const
    {
        const int wheelCount = static_cast<int>(settings.mWheels.size());
        for (const JoltVehicleAntiRollBar& bar : configuration.m_antiRollBars)
        {
            // An out-of-range index would index Jolt's wheel array directly, so it is
            // rejected here rather than left to crash mid-step.
            if (bar.m_leftWheel < 0 || bar.m_leftWheel >= wheelCount ||
                bar.m_rightWheel < 0 || bar.m_rightWheel >= wheelCount)
            {
                AZ_Warning("JoltPhysics", false,
                    "Vehicle%s has an anti-roll bar naming wheels %d and %d, but the vehicle has %d; this bar is "
                    "ignored.",
                    Internal::NameClause(configuration.m_debugName).c_str(), bar.m_leftWheel, bar.m_rightWheel, wheelCount);
                continue;
            }
            if (bar.m_leftWheel == bar.m_rightWheel)
            {
                AZ_Warning("JoltPhysics", false,
                    "Vehicle%s has an anti-roll bar coupling wheel %d to itself, which does nothing; this bar is "
                    "ignored.",
                    Internal::NameClause(configuration.m_debugName).c_str(), bar.m_leftWheel);
                continue;
            }

            JPH::VehicleAntiRollBar antiRollBar;
            antiRollBar.mLeftWheel = bar.m_leftWheel;
            antiRollBar.mRightWheel = bar.m_rightWheel;
            antiRollBar.mStiffness = bar.m_stiffness;
            settings.mAntiRollBars.push_back(antiRollBar);
        }
    }

    void JoltVehicle::BuildTrackedSettings(
        const JoltVehicleConfiguration& configuration, JPH::VehicleConstraintSettings& settings)
    {
        auto* controllerSettings = new JPH::TrackedVehicleControllerSettings;
        ApplyEngineAndTransmission(configuration, controllerSettings->mEngine, controllerSettings->mTransmission);

        // Wheels are split into the two tracks by which side of the chassis they sit on.
        for (size_t wheelIndex = 0; wheelIndex < configuration.m_wheels.size(); ++wheelIndex)
        {
            const JoltWheelConfiguration& wheelConfig = configuration.m_wheels[wheelIndex];

            auto* wheelSettings = new JPH::WheelSettingsTV;
            ApplyCommonWheelSettings(wheelConfig, *wheelSettings);
            wheelSettings->mLongitudinalFriction = wheelConfig.m_trackedLongitudinalFriction;
            wheelSettings->mLateralFriction = wheelConfig.m_trackedLateralFriction;
            settings.mWheels.push_back(wheelSettings);

            const auto trackSide = wheelConfig.m_position.GetY() >= 0.0f ? JPH::ETrackSide::Left : JPH::ETrackSide::Right;
            controllerSettings->mTracks[static_cast<int>(trackSide)].mWheels.push_back(static_cast<JPH::uint>(wheelIndex));
        }

        for (JPH::VehicleTrackSettings& track : controllerSettings->mTracks)
        {
            if (track.mWheels.empty())
            {
                AZ_Warning("JoltPhysics", false,
                    "Tracked vehicle%s has no wheels on one of its sides. Wheels are assigned to the left or right "
                    "track by the sign of their Y position, so both signs need to be represented.",
                    Internal::NameClause(configuration.m_debugName).c_str());
                continue;
            }

            // The authored driven wheel when it belongs to this track; the track's
            // first wheel otherwise.
            const bool isLeftTrack = &track == &controllerSettings->mTracks[static_cast<int>(JPH::ETrackSide::Left)];
            const int authoredDrivenWheel =
                isLeftTrack ? configuration.m_leftTrackDrivenWheel : configuration.m_rightTrackDrivenWheel;
            track.mDrivenWheel = track.mWheels.front();
            if (authoredDrivenWheel >= 0)
            {
                const bool belongsToTrack = AZStd::find(track.mWheels.begin(), track.mWheels.end(),
                    static_cast<JPH::uint>(authoredDrivenWheel)) != track.mWheels.end();
                if (belongsToTrack)
                {
                    track.mDrivenWheel = static_cast<JPH::uint>(authoredDrivenWheel);
                }
                else
                {
                    AZ_Warning("JoltPhysics", false,
                        "Tracked vehicle%s names wheel %d as the %s track's driven wheel, but that wheel is not "
                        "on that track; using the track's first wheel instead.",
                        Internal::NameClause(configuration.m_debugName).c_str(), authoredDrivenWheel,
                        isLeftTrack ? "left" : "right");
                }
            }
            track.mInertia = configuration.m_trackInertia;
            track.mAngularDamping = configuration.m_trackAngularDamping;
            track.mMaxBrakeTorque = configuration.m_trackMaxBrakeTorque;
            track.mDifferentialRatio = configuration.m_trackDifferentialRatio;
        }

        settings.mController = controllerSettings;
    }

    JoltVehicle::~JoltVehicle()
    {
        if (m_constraint && m_scene && m_scene->GetJoltPhysicsSystem())
        {
            m_scene->GetJoltPhysicsSystem()->RemoveStepListener(m_constraint);
            m_scene->GetJoltPhysicsSystem()->RemoveConstraint(m_constraint);
        }
    }

    void JoltVehicle::SetDriverInput(float forward, float right, float brake, float handbrake)
    {
        m_forwardInput = forward;
        m_steeringInput = right;
        m_brakeInput = brake;
        m_handBrakeInput = handbrake;
        ApplyDriverInput();
    }

    void JoltVehicle::SetForwardInput(float forward)
    {
        m_forwardInput = forward;
        ApplyDriverInput();
    }

    void JoltVehicle::SetSteeringInput(float right)
    {
        m_steeringInput = right;
        ApplyDriverInput();
    }

    void JoltVehicle::SetBrakeInput(float brake)
    {
        m_brakeInput = brake;
        ApplyDriverInput();
    }

    void JoltVehicle::SetHandBrakeInput(float handbrake)
    {
        m_handBrakeInput = handbrake;
        ApplyDriverInput();
    }

    void JoltVehicle::ApplyDriverInput()
    {
        if (m_trackedController)
        {
            // Tank steering: both tracks run at full rate going straight, and steering
            // slows (then reverses) the track on the inside of the turn, so full lock
            // pivots the vehicle on the spot. There is no separate handbrake.
            const float steering = AZStd::clamp(m_steeringInput, -1.0f, 1.0f);
            const float leftRatio = steering < 0.0f ? 1.0f + 2.0f * steering : 1.0f;
            const float rightRatio = steering > 0.0f ? 1.0f - 2.0f * steering : 1.0f;
            m_trackedController->SetDriverInput(
                m_forwardInput, SanitizeTrackRatio(leftRatio), SanitizeTrackRatio(rightRatio),
                AZStd::max(m_brakeInput, m_handBrakeInput));
        }
        else if (m_wheeledController)
        {
            m_wheeledController->SetDriverInput(m_forwardInput, m_steeringInput, m_brakeInput, m_handBrakeInput);
        }

        // Wake the chassis when input is applied: a sleeping body makes the constraint
        // inactive, which silently stops the tire forces (Jolt's anti-sleep only resets
        // the sleep timer, it cannot wake an already-sleeping body).
        if (m_chassisBody &&
            (m_forwardInput != 0.0f || m_steeringInput != 0.0f || m_brakeInput != 0.0f || m_handBrakeInput != 0.0f) &&
            !m_chassisBody->IsActive() && m_scene && m_scene->GetBodyInterface())
        {
            m_scene->GetBodyInterface()->ActivateBody(m_chassisBody->GetID());
        }
    }

    void JoltVehicle::SetGear(int gear)
    {
        JPH::VehicleTransmission* transmission = nullptr;
        if (m_trackedController)
        {
            transmission = &m_trackedController->GetTransmission();
        }
        else if (m_wheeledController)
        {
            transmission = &m_wheeledController->GetTransmission();
        }
        if (transmission == nullptr)
        {
            return;
        }

        const int forwardGearCount = static_cast<int>(transmission->mGearRatios.size());
        transmission->Set(AZStd::clamp(gear, -1, forwardGearCount), 1.0f);

        // A commanded gear should take effect now even if the vehicle was resting.
        if (m_chassisBody && !m_chassisBody->IsActive() && m_scene && m_scene->GetBodyInterface())
        {
            m_scene->GetBodyInterface()->ActivateBody(m_chassisBody->GetID());
        }
    }

    void JoltVehicle::SetTransmissionAutomatic(bool automatic)
    {
        const JPH::ETransmissionMode mode =
            automatic ? JPH::ETransmissionMode::Auto : JPH::ETransmissionMode::Manual;
        if (m_trackedController)
        {
            m_trackedController->GetTransmission().mMode = mode;
        }
        else if (m_wheeledController)
        {
            m_wheeledController->GetTransmission().mMode = mode;
        }
    }

    bool JoltVehicle::IsTransmissionAutomatic() const
    {
        if (m_trackedController)
        {
            return m_trackedController->GetTransmission().mMode == JPH::ETransmissionMode::Auto;
        }
        if (m_wheeledController)
        {
            return m_wheeledController->GetTransmission().mMode == JPH::ETransmissionMode::Auto;
        }
        return true;
    }

    void JoltVehicle::SetLeanControllerEnabled(bool enabled)
    {
        if (m_vehicleType == JoltVehicleType::Motorcycle && m_wheeledController)
        {
            static_cast<JPH::MotorcycleController*>(m_wheeledController)->EnableLeanController(enabled);
        }
    }

    void JoltVehicle::SetLeanSteeringLimitEnabled(bool enabled)
    {
        if (m_vehicleType == JoltVehicleType::Motorcycle && m_wheeledController)
        {
            static_cast<JPH::MotorcycleController*>(m_wheeledController)->EnableLeanSteeringLimit(enabled);
        }
    }

    void JoltVehicle::OverrideGravity(const AZ::Vector3& gravity)
    {
        if (m_constraint)
        {
            m_constraint->OverrideGravity(Conversions::ToJolt(gravity));
        }
    }

    void JoltVehicle::ResetGravityOverride()
    {
        if (m_constraint && m_constraint->IsGravityOverridden())
        {
            m_constraint->ResetGravityOverride();
        }
    }

    void JoltVehicle::SetCombineFriction(CombineFrictionFunction combineFriction)
    {
        if (!m_constraint)
        {
            return;
        }
        if (!combineFriction)
        {
            // Restore Jolt's default: multiply by the ground body's friction.
            m_constraint->SetCombineFriction(
                [](JPH::uint, float& ioLongitudinalFriction, float& ioLateralFriction,
                    const JPH::Body& inBody2, const JPH::SubShapeID&)
                {
                    ioLongitudinalFriction *= inBody2.GetFriction();
                    ioLateralFriction *= inBody2.GetFriction();
                });
            return;
        }
        m_constraint->SetCombineFriction(
            [callback = AZStd::move(combineFriction)](JPH::uint inWheelIndex, float& ioLongitudinalFriction,
                float& ioLateralFriction, const JPH::Body& inBody2, const JPH::SubShapeID&)
            {
                callback(static_cast<AZ::u32>(inWheelIndex), ioLongitudinalFriction, ioLateralFriction,
                    AZ::EntityId(inBody2.GetUserData()));
            });
    }

    float JoltVehicle::GetSpeed() const
    {
        if (!m_chassisBody)
        {
            return 0.0f;
        }
        const JPH::Vec3 forward = m_chassisBody->GetRotation() * JPH::Vec3::sAxisX();
        return m_chassisBody->GetLinearVelocity().Dot(forward);
    }

    float JoltVehicle::GetEngineRpm() const
    {
        if (m_trackedController)
        {
            return m_trackedController->GetEngine().GetCurrentRPM();
        }
        return m_wheeledController ? m_wheeledController->GetEngine().GetCurrentRPM() : 0.0f;
    }

    int JoltVehicle::GetCurrentGear() const
    {
        if (m_trackedController)
        {
            return m_trackedController->GetTransmission().GetCurrentGear();
        }
        return m_wheeledController ? m_wheeledController->GetTransmission().GetCurrentGear() : 0;
    }

    AZ::u32 JoltVehicle::GetWheelCount() const
    {
        return m_constraint ? static_cast<AZ::u32>(m_constraint->GetWheels().size()) : 0;
    }

    bool JoltVehicle::GetWheelTransform(AZ::u32 wheelIndex, AZ::Transform& outTransform) const
    {
        if (!m_constraint || wheelIndex >= GetWheelCount())
        {
            return false;
        }

        // Jolt hands back a transform for a cylinder aligned with the wheel's right axis.
        // The gem's wheels spin about Y and stand on Z (see ApplyCommonWheelSettings), so
        // that is the model space asked for here, and the result orients a wheel mesh
        // built the same way - it carries the steer angle and the roll of the tyre, not
        // just the suspension position.
        outTransform = Conversions::FromJolt(
            m_constraint->GetWheelWorldTransform(wheelIndex, JPH::Vec3::sAxisY(), JPH::Vec3::sAxisZ()));
        return true;
    }

    float JoltVehicle::GetSuspensionLength(AZ::u32 wheelIndex) const
    {
        if (!m_constraint || wheelIndex >= GetWheelCount())
        {
            return 0.0f;
        }
        return m_constraint->GetWheel(wheelIndex)->GetSuspensionLength();
    }

    bool JoltVehicle::IsWheelOnGround(AZ::u32 wheelIndex) const
    {
        if (!m_constraint || wheelIndex >= GetWheelCount())
        {
            return false;
        }
        return m_constraint->GetWheel(wheelIndex)->HasContact();
    }

    float JoltVehicle::GetWheelAngularVelocity(AZ::u32 wheelIndex) const
    {
        if (!m_constraint || wheelIndex >= GetWheelCount())
        {
            return 0.0f;
        }
        return m_constraint->GetWheel(wheelIndex)->GetAngularVelocity();
    }

    float JoltVehicle::GetWheelSteerAngle(AZ::u32 wheelIndex) const
    {
        if (!m_constraint || wheelIndex >= GetWheelCount())
        {
            return 0.0f;
        }
        return m_constraint->GetWheel(wheelIndex)->GetSteerAngle();
    }

    float JoltVehicle::GetWheelLongitudinalSlip(AZ::u32 wheelIndex) const
    {
        // Slip is computed by the wheeled tire model (WheelWV); tracked wheels have none.
        if (!m_wheeledController || !m_constraint || wheelIndex >= GetWheelCount())
        {
            return 0.0f;
        }
        return static_cast<const JPH::WheelWV*>(m_constraint->GetWheel(wheelIndex))->mLongitudinalSlip;
    }

    float JoltVehicle::GetWheelLateralSlip(AZ::u32 wheelIndex) const
    {
        if (!m_wheeledController || !m_constraint || wheelIndex >= GetWheelCount())
        {
            return 0.0f;
        }
        return static_cast<const JPH::WheelWV*>(m_constraint->GetWheel(wheelIndex))->mLateralSlip;
    }

    bool JoltVehicle::GetWheelContactPoint(AZ::u32 wheelIndex, AZ::Vector3& outPoint) const
    {
        if (!m_constraint || wheelIndex >= GetWheelCount() || !m_constraint->GetWheel(wheelIndex)->HasContact())
        {
            return false;
        }
        outPoint = Conversions::FromJolt(m_constraint->GetWheel(wheelIndex)->GetContactPosition());
        return true;
    }

    bool JoltVehicle::GetWheelContactNormal(AZ::u32 wheelIndex, AZ::Vector3& outNormal) const
    {
        if (!m_constraint || wheelIndex >= GetWheelCount() || !m_constraint->GetWheel(wheelIndex)->HasContact())
        {
            return false;
        }
        outNormal = Conversions::FromJolt(m_constraint->GetWheel(wheelIndex)->GetContactNormal());
        return true;
    }

    bool JoltVehicle::IsWheelSuspensionBottomedOut(AZ::u32 wheelIndex) const
    {
        if (!m_constraint || wheelIndex >= GetWheelCount())
        {
            return false;
        }
        return m_constraint->GetWheel(wheelIndex)->HasHitHardPoint();
    }

    float JoltVehicle::GetLeanAngle() const
    {
        if (m_vehicleType != JoltVehicleType::Motorcycle || !m_chassisBody)
        {
            return 0.0f;
        }

        // Jolt does not expose the controller's lean angle, so it is measured from the
        // chassis: the roll of its up axis away from the world up.
        const JPH::Vec3 chassisUp = m_chassisBody->GetRotation() * JPH::Vec3::sAxisZ();
        const JPH::Vec3 chassisForward = m_chassisBody->GetRotation() * JPH::Vec3::sAxisX();
        // Project the world up onto the plane perpendicular to the bike's forward axis so
        // pitch (going up a ramp) does not read as lean.
        JPH::Vec3 worldUpInRollPlane = JPH::Vec3::sAxisZ() - chassisForward * JPH::Vec3::sAxisZ().Dot(chassisForward);
        if (worldUpInRollPlane.IsNearZero())
        {
            return 0.0f;
        }
        worldUpInRollPlane = worldUpInRollPlane.Normalized();
        return acosf(AZStd::clamp(chassisUp.Dot(worldUpInRollPlane), -1.0f, 1.0f));
    }

} // namespace JoltPhysics
