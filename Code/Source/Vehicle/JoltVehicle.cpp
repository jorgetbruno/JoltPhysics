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
            wheelSettings.mSuspensionSpring = JPH::SpringSettings(
                JPH::ESpringMode::FrequencyAndDamping, wheelConfig.m_suspensionFrequency, wheelConfig.m_suspensionDamping);
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
                "Motorcycle lean spring constant %.0f is very stiff for this chassis (roll inertia %.1f kg m^2, "
                "implying a %.1f Hz lean response); the balance correction is likely to throw the bike around. "
                "Try a lean spring constant near %.0f with damping near %.0f.",
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

        if (m_vehicleType == JoltVehicleType::Motorcycle)
        {
            WarnOnImplausibleLeanGains(effectiveConfiguration, *m_chassisBody);
        }

        m_constraint = new JPH::VehicleConstraint(*m_chassisBody, settings);
        m_constraint->SetVehicleCollisionTester(
            new JPH::VehicleCollisionTesterRay(ObjectLayers::Moving, Conversions::ToJolt(AZ::Vector3::CreateAxisZ())));

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
            controllerSettings = motorcycleSettings;
        }
        else
        {
            controllerSettings = new JPH::WheeledVehicleControllerSettings;
        }

        controllerSettings->mEngine.mMaxTorque = configuration.m_maxEngineTorque;
        controllerSettings->mEngine.mMaxRPM = configuration.m_maxEngineRpm;
        controllerSettings->mTransmission.mMode = JPH::ETransmissionMode::Auto;
        controllerSettings->mTransmission.mGearRatios.assign(
            configuration.m_gearRatios.begin(), configuration.m_gearRatios.end());
        controllerSettings->mTransmission.mReverseGearRatios.assign({ configuration.m_reverseGearRatio });

        // A motorcycle has a single driven (rear) wheel, so it uses a differential with
        // only one side connected; Jolt reads -1 as "no wheel on this side".
        int leftDriveWheel = configuration.m_leftDriveWheel;
        int rightDriveWheel = configuration.m_rightDriveWheel;
        if (configuration.m_vehicleType == JoltVehicleType::Motorcycle &&
            leftDriveWheel == JoltVehicleConfiguration().m_leftDriveWheel &&
            rightDriveWheel == JoltVehicleConfiguration().m_rightDriveWheel)
        {
            // Defaults still point at a car's rear axle (wheels 2 and 3), which a two
            // wheeler does not have; drive its rear wheel instead.
            leftDriveWheel = -1;
            rightDriveWheel = static_cast<int>(configuration.m_wheels.size()) - 1;
        }

        const int wheelCount = static_cast<int>(configuration.m_wheels.size());
        if (leftDriveWheel >= wheelCount || rightDriveWheel >= wheelCount)
        {
            AZ_Warning("JoltPhysics", false,
                "Vehicle drive wheel index out of range (left %d, right %d, %d wheels); the differential is "
                "left unconnected and the vehicle will not drive.",
                leftDriveWheel, rightDriveWheel, wheelCount);
        }
        else if (leftDriveWheel >= 0 || rightDriveWheel >= 0)
        {
            JPH::VehicleDifferentialSettings differential;
            differential.mLeftWheel = leftDriveWheel;
            differential.mRightWheel = rightDriveWheel;
            differential.mDifferentialRatio = configuration.m_differentialRatio;
            controllerSettings->mDifferentials.push_back(differential);
        }

        settings.mController = controllerSettings;
    }

    void JoltVehicle::BuildTrackedSettings(
        const JoltVehicleConfiguration& configuration, JPH::VehicleConstraintSettings& settings)
    {
        auto* controllerSettings = new JPH::TrackedVehicleControllerSettings;
        controllerSettings->mEngine.mMaxTorque = configuration.m_maxEngineTorque;
        controllerSettings->mEngine.mMaxRPM = configuration.m_maxEngineRpm;
        controllerSettings->mTransmission.mMode = JPH::ETransmissionMode::Auto;
        controllerSettings->mTransmission.mGearRatios.assign(
            configuration.m_gearRatios.begin(), configuration.m_gearRatios.end());
        controllerSettings->mTransmission.mReverseGearRatios.assign({ configuration.m_reverseGearRatio });

        // Wheels are split into the two tracks by which side of the chassis they sit on.
        for (size_t wheelIndex = 0; wheelIndex < configuration.m_wheels.size(); ++wheelIndex)
        {
            const JoltWheelConfiguration& wheelConfig = configuration.m_wheels[wheelIndex];

            auto* wheelSettings = new JPH::WheelSettingsTV;
            ApplyCommonWheelSettings(wheelConfig, *wheelSettings);
            settings.mWheels.push_back(wheelSettings);

            const auto trackSide = wheelConfig.m_position.GetY() >= 0.0f ? JPH::ETrackSide::Left : JPH::ETrackSide::Right;
            controllerSettings->mTracks[static_cast<int>(trackSide)].mWheels.push_back(static_cast<JPH::uint>(wheelIndex));
        }

        for (JPH::VehicleTrackSettings& track : controllerSettings->mTracks)
        {
            if (track.mWheels.empty())
            {
                AZ_Warning("JoltPhysics", false,
                    "Tracked vehicle has no wheels on one of its sides. Wheels are assigned to the left or right "
                    "track by the sign of their Y position, so both signs need to be represented.");
                continue;
            }

            // The first wheel of a track drives it; the rest are along for the ride.
            track.mDrivenWheel = track.mWheels.front();
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
        if (m_trackedController)
        {
            // Tank steering: both tracks run at full rate going straight, and steering
            // slows (then reverses) the track on the inside of the turn, so full lock
            // pivots the vehicle on the spot. There is no separate handbrake.
            const float steering = AZStd::clamp(right, -1.0f, 1.0f);
            const float leftRatio = steering < 0.0f ? 1.0f + 2.0f * steering : 1.0f;
            const float rightRatio = steering > 0.0f ? 1.0f - 2.0f * steering : 1.0f;
            m_trackedController->SetDriverInput(
                forward, SanitizeTrackRatio(leftRatio), SanitizeTrackRatio(rightRatio), AZStd::max(brake, handbrake));
        }
        else if (m_wheeledController)
        {
            m_wheeledController->SetDriverInput(forward, right, brake, handbrake);
        }

        // Wake the chassis when input is applied: a sleeping body makes the constraint
        // inactive, which silently stops the tire forces (Jolt's anti-sleep only resets
        // the sleep timer, it cannot wake an already-sleeping body).
        if (m_chassisBody && (forward != 0.0f || right != 0.0f || brake != 0.0f || handbrake != 0.0f) &&
            !m_chassisBody->IsActive() && m_scene && m_scene->GetBodyInterface())
        {
            m_scene->GetBodyInterface()->ActivateBody(m_chassisBody->GetID());
        }
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
