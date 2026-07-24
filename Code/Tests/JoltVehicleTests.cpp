#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include "JoltTestWarningCatcher.h"

#include <Configuration/JoltSettingsRegistryManager.h>
#include <RigidBody/JoltRigidBody.h>
#include <Scene/JoltScene.h>
#include <System/JoltSystem.h>
#include <Vehicle/JoltVehicle.h>
#include <Vehicle/JoltVehicleConfiguration.h>

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/Utils.h>
#include <AzCore/Console/ILogger.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

namespace JoltPhysics
{
    class JoltVehicleTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "VehicleTestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);
        }

        void TearDown() override
        {
            m_vehicle.reset();
            m_system->RemoveScene(m_sceneHandle);
            m_system->Shutdown();
            m_system.reset();
        }

        AzPhysics::SimulatedBodyHandle CreateStaticBox(
            const AZ::Vector3& position, const AZ::Vector3& dimensions,
            const AZ::Quaternion& orientation = AZ::Quaternion::CreateIdentity())
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            boxShape->m_dimensions = dimensions;

            AzPhysics::StaticRigidBodyConfiguration staticConfig;
            staticConfig.m_position = position;
            staticConfig.m_orientation = orientation;
            staticConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);
            return m_scene->AddSimulatedBody(&staticConfig);
        }

        //! Standard 4-wheel car: front axle steers, rear axle drives.
        static JoltVehicleConfiguration MakeCarConfiguration()
        {
            JoltVehicleConfiguration config;
            struct WheelDesc { float x, y, steer; };
            for (const auto& desc : { WheelDesc{ 0.8f, 0.45f, 35.0f }, WheelDesc{ 0.8f, -0.45f, 35.0f },
                                      WheelDesc{ -0.8f, 0.45f, 0.0f }, WheelDesc{ -0.8f, -0.45f, 0.0f } })
            {
                JoltWheelConfiguration wheel;
                wheel.m_position = AZ::Vector3(desc.x, desc.y, -0.2f);
                wheel.m_maxSteerAngleDegrees = desc.steer;
                config.m_wheels.push_back(wheel);
            }
            config.m_leftDriveWheel = 2;
            config.m_rightDriveWheel = 3;
            return config;
        }

        //! Two-wheeler using the default motorcycle wheel layout. The engine is sized for
        //! a bike rather than a car: the lean controller only balances while every wheel
        //! is on the ground, so a car-sized engine on this mass just wheelies and falls.
        static JoltVehicleConfiguration MakeMotorcycleConfiguration()
        {
            JoltVehicleConfiguration config;
            config.m_vehicleType = JoltVehicleType::Motorcycle;
            config.m_chassisMass = 250.0f;
            config.m_maxEngineTorque = 150.0f;
            // The lean spring acts on the chassis roll inertia (~8.5 kg m^2 here), so its
            // gains have to be sized for the bike: Jolt's defaults are several times too
            // stiff at this mass and the correction impulse throws the bike into the air.
            config.m_leanSpringConstant = 1200.0f;
            config.m_leanSpringDamping = 200.0f;
            return config; // empty wheel list -> the default two-wheel layout
        }

        //! Tank using the default tracked wheel layout (four road wheels per side).
        static JoltVehicleConfiguration MakeTrackedConfiguration()
        {
            JoltVehicleConfiguration config;
            config.m_vehicleType = JoltVehicleType::Tracked;
            config.m_chassisMass = 2000.0f;
            return config; // empty wheel list -> the default eight-wheel layout
        }

        void CreateVehicle(const AZ::Vector3& position)
        {
            CreateVehicle(position, MakeCarConfiguration(), AZ::Vector3(2.0f, 1.0f, 0.5f), 1200.0f);
        }

        void CreateVehicle(
            const AZ::Vector3& position,
            const JoltVehicleConfiguration& vehicleConfiguration,
            const AZ::Vector3& chassisDimensions,
            float chassisMass)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            boxShape->m_dimensions = chassisDimensions;

            AzPhysics::RigidBodyConfiguration chassisConfig;
            chassisConfig.m_position = position;
            chassisConfig.m_mass = chassisMass;
            chassisConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);
            m_chassisHandle = m_scene->AddSimulatedBody(&chassisConfig);
            ASSERT_NE(m_chassisHandle, AzPhysics::InvalidSimulatedBodyHandle);

            JPH::Body* chassisBody = static_cast<JoltScene*>(m_scene)->GetJoltBody(m_chassisHandle);
            ASSERT_NE(chassisBody, nullptr);

            m_vehicle.reset(aznew JoltVehicle(vehicleConfiguration, static_cast<JoltScene*>(m_scene), chassisBody));
            ASSERT_TRUE(m_vehicle->IsValid());
        }

        //! Yaw of the chassis around the world up axis, in degrees.
        float GetChassisYawDegrees()
        {
            const AZ::Vector3 forward =
                GetChassis()->GetOrientation().TransformVector(AZ::Vector3::CreateAxisX());
            return AZ::RadToDeg(atan2f(forward.GetY(), forward.GetX()));
        }

        AzPhysics::RigidBody* GetChassis()
        {
            return azdynamic_cast<AzPhysics::RigidBody*>(m_scene->GetSimulatedBodyFromHandle(m_chassisHandle));
        }

        void DriveSteps(float forward, float right, float brake, int steps)
        {
            const float fixedDeltaTime = 1.0f / 60.0f;
            for (int i = 0; i < steps; ++i)
            {
                m_vehicle->SetDriverInput(forward, right, brake, 0.0f);
                m_scene->StartSimulation(fixedDeltaTime);
                m_scene->FinishSimulation();
            }
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle;
        AzPhysics::Scene* m_scene = nullptr;
        AzPhysics::SimulatedBodyHandle m_chassisHandle = AzPhysics::InvalidSimulatedBodyHandle;
        AZStd::unique_ptr<JoltVehicle> m_vehicle;
    };

    TEST_F(JoltVehicleTests, AcceleratesSteersAndBrakes)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(200.0f, 200.0f, 1.0f));
        CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.9f));

        auto* chassis = GetChassis();
        ASSERT_NE(chassis, nullptr);

        // The wheel collision tests find the ground and the step listener runs.
        int stepCount = 0;
        m_vehicle->GetConstraint()->SetPreStepCallback(
            [&stepCount](JPH::VehicleConstraint&, const JPH::PhysicsStepListenerContext&)
            {
                ++stepCount;
            });

        // Settle the suspension.
        DriveSteps(0.0f, 0.0f, 0.0f, 60);

        EXPECT_GT(stepCount, 0);
        EXPECT_TRUE(m_vehicle->GetConstraint()->GetWheel(0)->HasContact());
        EXPECT_NEAR(chassis->GetPosition().GetZ(), 0.8f, 0.3f);

        // Full throttle: the car accelerates forwards.
        DriveSteps(1.0f, 0.0f, 0.0f, 180);
        EXPECT_GT(m_vehicle->GetSpeed(), 3.0f);
        EXPECT_GT(m_vehicle->GetEngineRpm(), 1000.0f);
        EXPECT_GT(chassis->GetPosition().GetX(), 3.0f);

        // Steering: the car yaws away from straight +x.
        DriveSteps(0.5f, 0.5f, 0.0f, 120);
        const AZ::Vector3 forward = chassis->GetOrientation().TransformVector(AZ::Vector3::CreateAxisX());
        EXPECT_GT(AZStd::abs(forward.GetY()), 0.05f);

        // Brake to a stop.
        DriveSteps(0.0f, 0.0f, 1.0f, 180);
        EXPECT_LT(AZStd::abs(m_vehicle->GetSpeed()), 0.5f);
    }

    TEST_F(JoltVehicleTests, ClimbsRampAndStaysUpright)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(200.0f, 200.0f, 1.0f));
        // 6 degree ramp rising to z~0.84 at x~8.5, flush with a platform at the top:
        // the car climbs onto the platform without launching (steeper/cliffed ramps
        // make the landing timing-sensitive).
        const AZ::Quaternion rampOrientation = AZ::Quaternion::CreateFromAxisAngle(AZ::Vector3::CreateAxisY(), -AZ::DegToRad(6.0f));
        CreateStaticBox(AZ::Vector3(4.5f, 0.0f, 0.27f), AZ::Vector3(8.0f, 4.0f, 0.3f), rampOrientation);
        CreateStaticBox(AZ::Vector3(10.5f, 0.0f, 0.69f), AZ::Vector3(5.0f, 4.0f, 0.3f));

        CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.9f));

        auto* chassis = GetChassis();
        ASSERT_NE(chassis, nullptr);

        DriveSteps(0.0f, 0.0f, 0.0f, 60);

        float maxZ = 0.0f;
        AZStd::string rampDiag;
        const float fixedDeltaTime = 1.0f / 60.0f;
        for (int i = 0; i < 480; ++i)
        {
            m_vehicle->SetDriverInput(0.6f, 0.0f, 0.0f, 0.0f);
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();
            maxZ = AZStd::max(maxZ, chassis->GetPosition().GetZ());
            if (i % 60 == 0)
            {
                rampDiag += AZStd::string::format("ramp %d: x=%.2f z=%.2f upZ=%.2f speed=%.2f\n",
                    i, chassis->GetPosition().GetX(), chassis->GetPosition().GetZ(),
                    chassis->GetOrientation().TransformVector(AZ::Vector3::CreateAxisZ()).GetZ(),
                    m_vehicle->GetSpeed());
            }
        }

        // The car drove up the ramp (peak height at/above the platform top) and stayed upright.
        EXPECT_GT(chassis->GetPosition().GetX(), 6.0f) << rampDiag.c_str();
        EXPECT_GT(maxZ, 0.9f) << rampDiag.c_str();
        const AZ::Vector3 up = chassis->GetOrientation().TransformVector(AZ::Vector3::CreateAxisZ());
        EXPECT_GT(up.GetZ(), 0.9f) << rampDiag.c_str();
    }

    TEST_F(JoltVehicleTests, VehicleConfigurationSerializationRoundTrip)
    {
        AZ::SerializeContext serializeContext;
        JoltWheelConfiguration::Reflect(&serializeContext);
        JoltVehicleConfiguration::Reflect(&serializeContext);

        JoltVehicleConfiguration source = MakeCarConfiguration();
        source.m_maxEngineTorque = 750.0f;
        source.m_maxEngineRpm = 7200.0f;
        source.m_differentialRatio = 3.9f;
        source.m_gearRatios = { 3.1f, 2.0f, 1.4f, 1.0f };
        source.m_reverseGearRatio = -3.2f;

        JoltVehicleConfiguration roundTripped;
        AZStd::vector<char> buffer;
        {
            AZ::IO::ByteContainerStream<AZStd::vector<char>> stream(&buffer);
            ASSERT_TRUE(AZ::Utils::SaveObjectToStream(stream, AZ::DataStream::ST_BINARY, &source, azrtti_typeid<JoltVehicleConfiguration>(), &serializeContext));
        }
        {
            AZ::IO::ByteContainerStream<AZStd::vector<char>> stream(&buffer);
            ASSERT_TRUE(AZ::Utils::LoadObjectFromStreamInPlace(stream, &serializeContext, azrtti_typeid<JoltVehicleConfiguration>(), &roundTripped));
        }

        ASSERT_EQ(roundTripped.m_wheels.size(), source.m_wheels.size());
        for (size_t i = 0; i < source.m_wheels.size(); ++i)
        {
            EXPECT_TRUE(roundTripped.m_wheels[i].m_position.IsClose(source.m_wheels[i].m_position));
            EXPECT_FLOAT_EQ(roundTripped.m_wheels[i].m_radius, source.m_wheels[i].m_radius);
            EXPECT_FLOAT_EQ(roundTripped.m_wheels[i].m_maxSteerAngleDegrees, source.m_wheels[i].m_maxSteerAngleDegrees);
            EXPECT_FLOAT_EQ(roundTripped.m_wheels[i].m_suspensionMaxLength, source.m_wheels[i].m_suspensionMaxLength);
        }
        EXPECT_FLOAT_EQ(roundTripped.m_maxEngineTorque, 750.0f);
        EXPECT_FLOAT_EQ(roundTripped.m_maxEngineRpm, 7200.0f);
        EXPECT_FLOAT_EQ(roundTripped.m_differentialRatio, 3.9f);
        EXPECT_EQ(roundTripped.m_gearRatios, source.m_gearRatios);
        EXPECT_FLOAT_EQ(roundTripped.m_reverseGearRatio, -3.2f);
        EXPECT_EQ(roundTripped.m_leftDriveWheel, source.m_leftDriveWheel);
        EXPECT_EQ(roundTripped.m_rightDriveWheel, source.m_rightDriveWheel);
    }

    TEST_F(JoltVehicleTests, RawJoltBodyVehicleDrives)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(200.0f, 200.0f, 1.0f));

        // Chassis created directly with JPH, bypassing the gem's rigid-body path.
        JPH::BodyCreationSettings bodySettings(
            new JPH::BoxShape(JPH::Vec3(1.0f, 0.5f, 0.25f)),
            JPH::RVec3(0.0f, 0.0f, 0.9f),
            JPH::Quat::sIdentity(),
            JPH::EMotionType::Dynamic,
            ObjectLayers::Moving);
        // Sleep disabled on this chassis: exercises the vehicle with an always-awake body.
        bodySettings.mAllowSleeping = false;
        auto* joltScene = static_cast<JoltScene*>(m_scene);
        JPH::Body* rawBody = joltScene->GetBodyInterface()->CreateBody(bodySettings);
        ASSERT_NE(rawBody, nullptr);
        joltScene->GetBodyInterface()->AddBody(rawBody->GetID(), JPH::EActivation::Activate);

        auto vehicle = AZStd::make_unique<JoltVehicle>(MakeCarConfiguration(), joltScene, rawBody);
        ASSERT_TRUE(vehicle->IsValid());

        const float fixedDeltaTime = 1.0f / 60.0f;
        for (int i = 0; i < 60; ++i)
        {
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();
        }

        // The car rests on its wheels: suspension compressed and loaded.
        EXPECT_NEAR(rawBody->GetPosition().GetZ(), 0.87f, 0.1f);
        EXPECT_GT(static_cast<const JPH::WheelWV*>(vehicle->GetConstraint()->GetWheel(2))->GetSuspensionLambda(), 10.0f);

        for (int i = 0; i < 180; ++i)
        {
            vehicle->SetDriverInput(1.0f, 0.0f, 0.0f, 0.0f);
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();
        }

        EXPECT_GT(vehicle->GetSpeed(), 3.0f);
        EXPECT_GT(rawBody->GetPosition().GetX(), 3.0f);
    }

    TEST_F(JoltVehicleTests, TrackedVehicleDrivesForward)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(200.0f, 200.0f, 1.0f));
        CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.9f), MakeTrackedConfiguration(), AZ::Vector3(3.0f, 1.6f, 0.6f), 2000.0f);

        EXPECT_EQ(m_vehicle->GetVehicleType(), JoltVehicleType::Tracked);
        // The default layout puts four road wheels on each of the two tracks.
        EXPECT_EQ(m_vehicle->GetConstraint()->GetWheels().size(), 8u);

        auto* chassis = GetChassis();
        ASSERT_NE(chassis, nullptr);

        // Settle onto the ground.
        DriveSteps(0.0f, 0.0f, 0.0f, 60);
        EXPECT_TRUE(m_vehicle->GetConstraint()->GetWheel(0)->HasContact());

        const float startX = chassis->GetPosition().GetX();
        DriveSteps(1.0f, 0.0f, 0.0f, 180);

        EXPECT_GT(m_vehicle->GetSpeed(), 1.0f);
        EXPECT_GT(m_vehicle->GetEngineRpm(), 1000.0f);
        EXPECT_GT(chassis->GetPosition().GetX() - startX, 1.0f);
    }

    TEST_F(JoltVehicleTests, TrackedVehiclePivotsOnFullSteeringLock)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(200.0f, 200.0f, 1.0f));
        CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.9f), MakeTrackedConfiguration(), AZ::Vector3(3.0f, 1.6f, 0.6f), 2000.0f);

        DriveSteps(0.0f, 0.0f, 0.0f, 60);

        const float startYaw = GetChassisYawDegrees();
        const AZ::Vector3 startPosition = GetChassis()->GetPosition();

        // Full lock counter-rotates the tracks, so the tank turns on the spot instead of
        // driving along an arc the way a steered vehicle would.
        DriveSteps(0.5f, 1.0f, 0.0f, 180);

        float yawDelta = GetChassisYawDegrees() - startYaw;
        // Normalize across the +/-180 wrap.
        yawDelta = fmodf(yawDelta + 540.0f, 360.0f) - 180.0f;

        EXPECT_GT(AZStd::abs(yawDelta), 20.0f);
        const float travelled = GetChassis()->GetPosition().GetDistance(startPosition);
        EXPECT_LT(travelled, 3.0f); // pivoting, not driving away
    }

    TEST_F(JoltVehicleTests, MotorcycleStaysUprightWhileDriving)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(200.0f, 200.0f, 1.0f));
        CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.8f), MakeMotorcycleConfiguration(), AZ::Vector3(1.8f, 0.4f, 0.5f), 250.0f);

        EXPECT_EQ(m_vehicle->GetVehicleType(), JoltVehicleType::Motorcycle);
        // The default layout is a front and a rear wheel.
        EXPECT_EQ(m_vehicle->GetConstraint()->GetWheels().size(), 2u);

        auto* chassis = GetChassis();
        ASSERT_NE(chassis, nullptr);

        DriveSteps(0.0f, 0.0f, 0.0f, 60);
        // Part throttle keeps the front wheel down; see MakeMotorcycleConfiguration.
        DriveSteps(0.4f, 0.0f, 0.0f, 240);

        // It drives off under its own power...
        EXPECT_GT(chassis->GetPosition().GetX(), 2.0f);
        // ...and the lean-balance spring keeps a two-wheeler that would otherwise topple
        // roughly upright (its own up axis still close to the world up).
        const AZ::Vector3 chassisUp = chassis->GetOrientation().TransformVector(AZ::Vector3::CreateAxisZ());
        EXPECT_GT(chassisUp.GetZ(), 0.9f);
        EXPECT_LT(m_vehicle->GetLeanAngle(), AZ::DegToRad(20.0f));
        // Both wheels stay on the ground: the lean controller stops correcting as soon as
        // a wheel leaves it, so losing contact is how a motorcycle setup goes wrong.
        EXPECT_TRUE(m_vehicle->GetConstraint()->GetWheel(0)->HasContact());
        EXPECT_TRUE(m_vehicle->GetConstraint()->GetWheel(1)->HasContact());
    }

    TEST_F(JoltVehicleTests, MotorcycleWarnsWhenLeanGainsAreTooStiffForTheChassis)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(200.0f, 200.0f, 1.0f));

        // Jolt's default lean gains on a light bike: stiff enough that the balance
        // correction throws it around, which is reported rather than left to be
        // discovered as a motorcycle launching itself into the air.
        JoltVehicleConfiguration config = MakeMotorcycleConfiguration();
        config.m_leanSpringConstant = 5000.0f;
        config.m_leanSpringDamping = 1000.0f;

        JoltWarningCatcher warnings;
        CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.8f), config, AZ::Vector3(1.8f, 0.4f, 0.5f), 250.0f);
        EXPECT_TRUE(warnings.ContainsWarningWith("lean spring constant"));
    }

} // namespace JoltPhysics
