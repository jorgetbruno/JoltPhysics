#include <Vehicle/JoltVehicle.h>

#include <Scene/JoltScene.h>
#include <System/CollisionLayerFilters.h>
#include <Utils/Conversions.h>
#include <Vehicle/JoltVehicleConfiguration.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/VehicleDifferential.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

namespace JoltPhysics
{
    JoltVehicle::JoltVehicle(const JoltVehicleConfiguration& configuration, JoltScene* scene, JPH::Body* chassisBody)
        : m_scene(scene)
        , m_chassisBody(chassisBody)
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
            // Default 4-wheel layout: front axle steers, rear axle drives.
            struct DefaultWheel { float x, y, steer; };
            for (const DefaultWheel& desc : { DefaultWheel{ 0.8f, 0.45f, 35.0f }, DefaultWheel{ 0.8f, -0.45f, 35.0f },
                                              DefaultWheel{ -0.8f, 0.45f, 0.0f }, DefaultWheel{ -0.8f, -0.45f, 0.0f } })
            {
                JoltWheelConfiguration wheel;
                wheel.m_position = AZ::Vector3(desc.x, desc.y, -0.2f);
                wheel.m_maxSteerAngleDegrees = desc.steer;
                effectiveConfiguration.m_wheels.push_back(wheel);
            }
        }

        JPH::VehicleConstraintSettings settings;
        settings.mUp = JPH::Vec3::sAxisZ();
        settings.mForward = JPH::Vec3::sAxisX();

        for (const JoltWheelConfiguration& wheelConfig : effectiveConfiguration.m_wheels)
        {
            auto* wheelSettings = new JPH::WheelSettingsWV;
            wheelSettings->mPosition = Conversions::ToJolt(wheelConfig.m_position);
            wheelSettings->mSuspensionDirection = JPH::Vec3(0.0f, 0.0f, -1.0f);
            wheelSettings->mSteeringAxis = JPH::Vec3::sAxisZ();
            wheelSettings->mWheelUp = JPH::Vec3::sAxisZ();
            wheelSettings->mWheelForward = JPH::Vec3::sAxisX();
            wheelSettings->mRadius = wheelConfig.m_radius;
            wheelSettings->mWidth = wheelConfig.m_width;
            wheelSettings->mSuspensionMinLength = wheelConfig.m_suspensionMinLength;
            wheelSettings->mSuspensionMaxLength = wheelConfig.m_suspensionMaxLength;
            wheelSettings->mSuspensionSpring = JPH::SpringSettings(
                JPH::ESpringMode::FrequencyAndDamping, wheelConfig.m_suspensionFrequency, wheelConfig.m_suspensionDamping);
            wheelSettings->mMaxSteerAngle = AZ::DegToRad(wheelConfig.m_maxSteerAngleDegrees);
            wheelSettings->mMaxBrakeTorque = wheelConfig.m_maxBrakeTorque;
            wheelSettings->mMaxHandBrakeTorque = wheelConfig.m_maxHandBrakeTorque;
            settings.mWheels.push_back(wheelSettings);
        }

        auto* controllerSettings = new JPH::WheeledVehicleControllerSettings;
        controllerSettings->mEngine.mMaxTorque = effectiveConfiguration.m_maxEngineTorque;
        controllerSettings->mEngine.mMaxRPM = effectiveConfiguration.m_maxEngineRpm;
        controllerSettings->mTransmission.mMode = JPH::ETransmissionMode::Auto;
        controllerSettings->mTransmission.mGearRatios.assign(
            effectiveConfiguration.m_gearRatios.begin(), effectiveConfiguration.m_gearRatios.end());
        controllerSettings->mTransmission.mReverseGearRatios.assign({ effectiveConfiguration.m_reverseGearRatio });

        if (effectiveConfiguration.m_leftDriveWheel >= 0 || effectiveConfiguration.m_rightDriveWheel >= 0)
        {
            JPH::VehicleDifferentialSettings differential;
            differential.mLeftWheel = effectiveConfiguration.m_leftDriveWheel;
            differential.mRightWheel = effectiveConfiguration.m_rightDriveWheel;
            differential.mDifferentialRatio = effectiveConfiguration.m_differentialRatio;
            controllerSettings->mDifferentials.push_back(differential);
        }

        settings.mController = controllerSettings;

        m_constraint = new JPH::VehicleConstraint(*m_chassisBody, settings);
        m_constraint->SetVehicleCollisionTester(new JPH::VehicleCollisionTesterRay(ObjectLayers::Moving, Conversions::ToJolt(AZ::Vector3::CreateAxisZ())));

        m_scene->GetJoltPhysicsSystem()->AddConstraint(m_constraint);
        m_scene->GetJoltPhysicsSystem()->AddStepListener(m_constraint);

        m_controller = static_cast<JPH::WheeledVehicleController*>(m_constraint->GetController());
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
        if (m_controller)
        {
            m_controller->SetDriverInput(forward, right, brake, handbrake);
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
        return m_controller ? m_controller->GetEngine().GetCurrentRPM() : 0.0f;
    }

    int JoltVehicle::GetCurrentGear() const
    {
        return m_controller ? m_controller->GetTransmission().GetCurrentGear() : 0;
    }

} // namespace JoltPhysics
