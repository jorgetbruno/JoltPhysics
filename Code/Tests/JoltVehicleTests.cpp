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

#include <AzCore/Component/Entity.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Physics/SystemBus.h>

#include <Clients/Components/JoltBoxColliderComponent.h>
#include <Clients/Components/JoltRigidBodyComponent.h>
#include <Clients/Components/JoltVehicleComponent.h>

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

        // Brake to a stop. Six seconds rather than three: the cylinder ground test
        // grips well enough that the car gains speed through the steering phase above
        // instead of scrubbing it off, so it arrives at the brakes faster than it did
        // when every wheel was a single ray.
        DriveSteps(0.0f, 0.0f, 1.0f, 360);
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

    TEST_F(JoltVehicleTests, ACylinderGroundTestBridgesAGapThatSwallowsARay)
    {
        // A trench 0.2 m across, narrower than the 0.7 m wheel. A ray fired down the
        // wheel's centre line finds nothing; the wheel itself spans the gap easily.
        auto BuildTrenchAndSettle = [this](JoltVehicleCollisionTester tester)
        {
            TearDown();
            SetUp();
            CreateStaticBox(AZ::Vector3(-25.1f, 0.0f, -0.5f), AZ::Vector3(50.0f, 50.0f, 1.0f));
            CreateStaticBox(AZ::Vector3(25.1f, 0.0f, -0.5f), AZ::Vector3(50.0f, 50.0f, 1.0f));

            JoltVehicleConfiguration config = MakeCarConfiguration();
            config.m_collisionTester = tester;
            // Front axle (local x = +0.8) parked directly over the trench at world x = 0.
            CreateVehicle(AZ::Vector3(-0.8f, 0.0f, 0.9f), config, AZ::Vector3(2.0f, 1.0f, 0.5f), 1200.0f);
            DriveSteps(0.0f, 0.0f, 0.0f, 30);
            return m_vehicle->IsWheelOnGround(0);
        };

        EXPECT_FALSE(BuildTrenchAndSettle(JoltVehicleCollisionTester::Ray));
        EXPECT_TRUE(BuildTrenchAndSettle(JoltVehicleCollisionTester::Cylinder));
    }

    TEST_F(JoltVehicleTests, ThePitchRollLimitStopsATankPoweringItselfOntoItsBack)
    {
        // The default tracked drive has enough torque to pop a wheelie. Left unlimited,
        // the suspension keeps pushing past the vertical and the tank lands on its back
        // still under power - which only showed up once the wheels had real grip.
        auto DriveAndReportUpZ = [this](float pitchLimitDegrees)
        {
            TearDown();
            SetUp();
            CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(400.0f, 200.0f, 1.0f));
            JoltVehicleConfiguration config = MakeTrackedConfiguration();
            config.m_maxPitchRollAngleDegrees = pitchLimitDegrees;
            CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.9f), config, AZ::Vector3(3.0f, 1.6f, 0.6f), 2000.0f);
            DriveSteps(0.0f, 0.0f, 0.0f, 60);
            DriveSteps(1.0f, 0.0f, 0.0f, 180);
            return GetChassis()->GetOrientation().TransformVector(AZ::Vector3::CreateAxisZ()).GetZ();
        };

        EXPECT_LT(DriveAndReportUpZ(180.0f), 0.5f);  // unlimited: tipped past horizontal
        EXPECT_GT(DriveAndReportUpZ(60.0f), 0.9f);   // limited: still upright
    }

    TEST_F(JoltVehicleTests, AnAntiRollBarKeepsTheChassisFlatterThroughACorner)
    {
        // Same corner twice. The default car's track is narrow next to how high its
        // centre of mass sits, so without a bar it does not understeer - it rolls onto
        // its side.
        auto CornerAndReportMaxRoll = [this](bool withAntiRollBars)
        {
            TearDown();
            SetUp();
            CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(400.0f, 400.0f, 1.0f));

            JoltVehicleConfiguration config = MakeCarConfiguration();
            // Deliberately soft springs. On its stiff default springs this car does not
            // roll on its suspension at full lock - it tips over bodily, because the
            // lateral force at the tyres out-levers its track width, and no anti-roll
            // bar can stop that. Soft springs isolate the roll a bar is actually for.
            for (JoltWheelConfiguration& wheel : config.m_wheels)
            {
                wheel.m_suspensionFrequency = 0.8f;
            }
            if (withAntiRollBars)
            {
                config.m_antiRollBars.push_back({ 0, 1, 20000.0f }); // front axle
                config.m_antiRollBars.push_back({ 2, 3, 20000.0f }); // rear axle
            }
            CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.9f), config, AZ::Vector3(2.0f, 1.0f, 0.5f), 1200.0f);

            DriveSteps(0.0f, 0.0f, 0.0f, 60);
            DriveSteps(1.0f, 0.0f, 0.0f, 120); // up to speed
            float maxRollDegrees = 0.0f;
            for (int i = 0; i < 120; ++i)
            {
                DriveSteps(0.6f, 1.0f, 0.0f, 1); // full lock
                // Roll alone: how far the chassis right axis has lifted off horizontal,
                // which pitching over a crest would not register as.
                const AZ::Vector3 right = GetChassis()->GetOrientation().TransformVector(AZ::Vector3::CreateAxisY());
                maxRollDegrees = AZStd::max(
                    maxRollDegrees, AZStd::abs(AZ::RadToDeg(asinf(AZStd::clamp(right.GetZ(), -1.0f, 1.0f)))));
            }
            return maxRollDegrees;
        };

        EXPECT_GT(CornerAndReportMaxRoll(false), 30.0f); // rolls over
        EXPECT_LT(CornerAndReportMaxRoll(true), 10.0f);  // stays on its wheels
    }

    TEST_F(JoltVehicleTests, AnAntiRollBarNamingAWheelThatDoesNotExistIsRejected)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(200.0f, 200.0f, 1.0f));

        JoltVehicleConfiguration config = MakeCarConfiguration();
        config.m_antiRollBars.push_back({ 0, 9, 1000.0f }); // only 4 wheels exist
        config.m_antiRollBars.push_back({ 1, 1, 1000.0f }); // a wheel against itself
        config.m_antiRollBars.push_back({ 0, 1, 1000.0f }); // the one good bar

        JoltWarningCatcher warnings;
        CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.9f), config, AZ::Vector3(2.0f, 1.0f, 0.5f), 1200.0f);

        // Bad bars are dropped with a diagnostic rather than indexing Jolt's wheel array
        // out of range, and the good one still makes it through.
        EXPECT_TRUE(warnings.ContainsWarningWith("anti-roll bar"));
        EXPECT_EQ(m_vehicle->GetConstraint()->GetAntiRollBars().size(), 1u);
    }

    TEST_F(JoltVehicleTests, WheelTransformsFollowTheVehicleForVisualWheels)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(400.0f, 200.0f, 1.0f));
        CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.9f));

        EXPECT_EQ(m_vehicle->GetWheelCount(), 4u);

        DriveSteps(0.0f, 0.0f, 0.0f, 60);

        // Each wheel sits where it was authored, out to its own corner of the chassis and
        // resting on the ground rather than at the attachment point.
        AZ::Transform frontLeft = AZ::Transform::CreateIdentity();
        ASSERT_TRUE(m_vehicle->GetWheelTransform(0, frontLeft));
        EXPECT_NEAR(frontLeft.GetTranslation().GetX(), 0.8f, 0.2f);
        EXPECT_NEAR(frontLeft.GetTranslation().GetY(), 0.45f, 0.2f);
        EXPECT_NEAR(frontLeft.GetTranslation().GetZ(), 0.35f, 0.2f); // wheel radius above ground

        AZ::Transform rearRight = AZ::Transform::CreateIdentity();
        ASSERT_TRUE(m_vehicle->GetWheelTransform(3, rearRight));
        EXPECT_NEAR(rearRight.GetTranslation().GetX(), -0.8f, 0.2f);
        EXPECT_NEAR(rearRight.GetTranslation().GetY(), -0.45f, 0.2f);

        // The suspension is carrying the car, so it sits between its two travel limits.
        const float suspensionLength = m_vehicle->GetSuspensionLength(0);
        EXPECT_GT(suspensionLength, 0.15f);
        EXPECT_LT(suspensionLength, 0.45f);
        EXPECT_TRUE(m_vehicle->IsWheelOnGround(0));

        // Driving forward moves the wheels with the chassis and spins them.
        DriveSteps(1.0f, 0.0f, 0.0f, 120);
        AZ::Transform frontLeftMoved = AZ::Transform::CreateIdentity();
        ASSERT_TRUE(m_vehicle->GetWheelTransform(0, frontLeftMoved));
        EXPECT_GT(frontLeftMoved.GetTranslation().GetX(), frontLeft.GetTranslation().GetX() + 3.0f);
        EXPECT_FALSE(frontLeftMoved.GetRotation().IsClose(frontLeft.GetRotation(), 0.01f));
    }

    TEST_F(JoltVehicleTests, AnOutOfRangeWheelIndexReportsNothingRatherThanReadingPastTheArray)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(200.0f, 200.0f, 1.0f));
        CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.9f));

        AZ::Transform unchanged = AZ::Transform::CreateTranslation(AZ::Vector3(7.0f, 7.0f, 7.0f));
        EXPECT_FALSE(m_vehicle->GetWheelTransform(99, unchanged));
        EXPECT_TRUE(unchanged.GetTranslation().IsClose(AZ::Vector3(7.0f, 7.0f, 7.0f)));
        EXPECT_EQ(m_vehicle->GetSuspensionLength(99), 0.0f);
        EXPECT_FALSE(m_vehicle->IsWheelOnGround(99));
    }


    TEST_F(JoltVehicleTests, AuthoredDifferentialsOverrideTheLegacyDriveWheelFields)
    {
        JoltVehicleConfiguration config = MakeCarConfiguration();
        JoltVehicleDifferential front;
        front.m_leftWheel = 0;
        front.m_rightWheel = 1;
        front.m_differentialRatio = 4.1f;
        front.m_leftRightSplit = 0.3f;
        front.m_limitedSlipRatio = 2.0f;
        front.m_engineTorqueRatio = 0.6f;
        JoltVehicleDifferential rear;
        rear.m_leftWheel = 2;
        rear.m_rightWheel = 3;
        rear.m_engineTorqueRatio = 0.4f;
        config.m_differentials = { front, rear };
        config.m_differentialLimitedSlipRatio = 1.8f;

        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(50.0f, 50.0f, 1.0f));
        CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.9f), config, AZ::Vector3(2.0f, 1.0f, 0.5f), 1200.0f);

        const auto* controller =
            static_cast<const JPH::WheeledVehicleController*>(m_vehicle->GetConstraint()->GetController());
        ASSERT_EQ(controller->GetDifferentials().size(), 2u);
        EXPECT_EQ(controller->GetDifferentials()[0].mLeftWheel, 0);
        EXPECT_EQ(controller->GetDifferentials()[0].mRightWheel, 1);
        EXPECT_FLOAT_EQ(controller->GetDifferentials()[0].mDifferentialRatio, 4.1f);
        EXPECT_FLOAT_EQ(controller->GetDifferentials()[0].mLeftRightSplit, 0.3f);
        EXPECT_FLOAT_EQ(controller->GetDifferentials()[0].mLimitedSlipRatio, 2.0f);
        EXPECT_FLOAT_EQ(controller->GetDifferentials()[0].mEngineTorqueRatio, 0.6f);
        EXPECT_EQ(controller->GetDifferentials()[1].mLeftWheel, 2);
        EXPECT_FLOAT_EQ(controller->GetDifferentialLimitedSlipRatio(), 1.8f);
    }

    TEST_F(JoltVehicleTests, EngineTransmissionAndWheelTuningLandInJolt)
    {
        JoltVehicleConfiguration config = MakeCarConfiguration();
        config.m_minEngineRpm = 900.0f;
        config.m_engineInertia = 0.7f;
        config.m_engineAngularDamping = 0.1f;
        config.m_engineTorqueCurve = { AZ::Vector2(0.0f, 0.5f), AZ::Vector2(1.0f, 1.0f) };
        config.m_gearSwitchTime = 0.25f;
        config.m_clutchReleaseTime = 0.15f;
        config.m_gearSwitchLatency = 0.4f;
        config.m_shiftUpRpm = 4500.0f;
        config.m_shiftDownRpm = 1800.0f;
        config.m_clutchStrength = 12.0f;
        for (JoltWheelConfiguration& wheel : config.m_wheels)
        {
            wheel.m_inertia = 1.2f;
            wheel.m_angularDamping = 0.05f;
            wheel.m_suspensionPreloadLength = 0.1f;
            wheel.m_longitudinalFrictionCurve = { AZ::Vector2(0.0f, 0.3f), AZ::Vector2(1.0f, 0.3f) };
            wheel.m_lateralFrictionCurve = { AZ::Vector2(0.0f, 0.4f), AZ::Vector2(20.0f, 0.4f) };
        }

        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(50.0f, 50.0f, 1.0f));
        CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.9f), config, AZ::Vector3(2.0f, 1.0f, 0.5f), 1200.0f);

        const auto* controller =
            static_cast<const JPH::WheeledVehicleController*>(m_vehicle->GetConstraint()->GetController());
        EXPECT_FLOAT_EQ(controller->GetEngine().mMinRPM, 900.0f);
        EXPECT_FLOAT_EQ(controller->GetEngine().mInertia, 0.7f);
        EXPECT_FLOAT_EQ(controller->GetEngine().mAngularDamping, 0.1f);
        EXPECT_NEAR(controller->GetEngine().mNormalizedTorque.GetValue(0.0f), 0.5f, 1e-3f);
        EXPECT_NEAR(controller->GetEngine().mNormalizedTorque.GetValue(1.0f), 1.0f, 1e-3f);
        EXPECT_FLOAT_EQ(controller->GetTransmission().mSwitchTime, 0.25f);
        EXPECT_FLOAT_EQ(controller->GetTransmission().mClutchReleaseTime, 0.15f);
        EXPECT_FLOAT_EQ(controller->GetTransmission().mSwitchLatency, 0.4f);
        EXPECT_FLOAT_EQ(controller->GetTransmission().mShiftUpRPM, 4500.0f);
        EXPECT_FLOAT_EQ(controller->GetTransmission().mShiftDownRPM, 1800.0f);
        EXPECT_FLOAT_EQ(controller->GetTransmission().mClutchStrength, 12.0f);

        const auto* wheelSettings =
            static_cast<const JPH::WheelSettingsWV*>(m_vehicle->GetConstraint()->GetWheel(0)->GetSettings());
        EXPECT_FLOAT_EQ(wheelSettings->mInertia, 1.2f);
        EXPECT_FLOAT_EQ(wheelSettings->mAngularDamping, 0.05f);
        EXPECT_FLOAT_EQ(wheelSettings->mSuspensionPreloadLength, 0.1f);
        EXPECT_NEAR(wheelSettings->mLongitudinalFriction.GetValue(0.5f), 0.3f, 1e-3f);
        EXPECT_NEAR(wheelSettings->mLateralFriction.GetValue(10.0f), 0.4f, 1e-3f);
    }

    TEST_F(JoltVehicleTests, LowGripTireCurvesMakeTheCarAccelerateSlower)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(400.0f, 100.0f, 1.0f));

        float grippyDistance = 0.0f;
        {
            CreateVehicle(AZ::Vector3(0.0f, 20.0f, 0.9f), MakeCarConfiguration(), AZ::Vector3(2.0f, 1.0f, 0.5f), 1200.0f);
            DriveSteps(0.0f, 0.0f, 0.0f, 60);
            const float startX = GetChassis()->GetPosition().GetX();
            DriveSteps(1.0f, 0.0f, 0.0f, 180);
            grippyDistance = GetChassis()->GetPosition().GetX() - startX;
        }

        float slipperyDistance = 0.0f;
        {
            JoltVehicleConfiguration config = MakeCarConfiguration();
            for (JoltWheelConfiguration& wheel : config.m_wheels)
            {
                // Ice: a tenth of the traction at every slip ratio.
                wheel.m_longitudinalFrictionCurve = { AZ::Vector2(0.0f, 0.1f), AZ::Vector2(1.0f, 0.1f) };
            }
            CreateVehicle(AZ::Vector3(0.0f, -20.0f, 0.9f), config, AZ::Vector3(2.0f, 1.0f, 0.5f), 1200.0f);
            DriveSteps(0.0f, 0.0f, 0.0f, 60);
            const float startX = GetChassis()->GetPosition().GetX();
            DriveSteps(1.0f, 0.0f, 0.0f, 180);
            slipperyDistance = GetChassis()->GetPosition().GetX() - startX;
        }

        // The authored curve is the handling knob: on ice the same throttle covers
        // notably less ground because the wheels spin instead of gripping.
        EXPECT_LT(slipperyDistance, grippyDistance * 0.7f)
            << "grippy " << grippyDistance << " m, slippery " << slipperyDistance << " m";
    }

    TEST_F(JoltVehicleTests, AutomaticTransmissionShiftsUpUnderFullThrottle)
    {
        CreateStaticBox(AZ::Vector3(150.0f, 0.0f, -0.5f), AZ::Vector3(500.0f, 50.0f, 1.0f));
        CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.9f));

        DriveSteps(0.0f, 0.0f, 0.0f, 60);
        EXPECT_LE(m_vehicle->GetCurrentGear(), 1);

        int highestGear = 0;
        const float fixedDeltaTime = 1.0f / 60.0f;
        for (int i = 0; i < 600; ++i)
        {
            m_vehicle->SetDriverInput(1.0f, 0.0f, 0.0f, 0.0f);
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();
            highestGear = AZStd::max(highestGear, m_vehicle->GetCurrentGear());
        }
        EXPECT_GE(highestGear, 2) << "speed " << m_vehicle->GetSpeed() << " rpm " << m_vehicle->GetEngineRpm();
    }

    TEST_F(JoltVehicleTests, ManualTransmissionHoldsNeutralUntilAGearIsSet)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(200.0f, 50.0f, 1.0f));
        JoltVehicleConfiguration config = MakeCarConfiguration();
        config.m_transmissionMode = JoltVehicleTransmissionMode::Manual;
        CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.9f), config, AZ::Vector3(2.0f, 1.0f, 0.5f), 1200.0f);

        EXPECT_FALSE(m_vehicle->IsTransmissionAutomatic());

        // Full throttle in neutral revs the engine but moves nothing.
        DriveSteps(0.0f, 0.0f, 0.0f, 60);
        DriveSteps(1.0f, 0.0f, 0.0f, 120);
        EXPECT_LT(AZStd::abs(m_vehicle->GetSpeed()), 0.5f);
        EXPECT_EQ(m_vehicle->GetCurrentGear(), 0);

        // First gear engages and the same throttle drives the car; nothing shifts it out.
        m_vehicle->SetGear(1);
        DriveSteps(1.0f, 0.0f, 0.0f, 240);
        EXPECT_GT(m_vehicle->GetSpeed(), 3.0f);
        EXPECT_EQ(m_vehicle->GetCurrentGear(), 1);
    }

    TEST_F(JoltVehicleTests, TheHandbrakeStopsARollingCar)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(400.0f, 50.0f, 1.0f));
        CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.9f));

        DriveSteps(0.0f, 0.0f, 0.0f, 60);
        DriveSteps(1.0f, 0.0f, 0.0f, 180);
        ASSERT_GT(m_vehicle->GetSpeed(), 3.0f);

        const float fixedDeltaTime = 1.0f / 60.0f;
        for (int i = 0; i < 360; ++i)
        {
            m_vehicle->SetDriverInput(0.0f, 0.0f, 0.0f, 1.0f);
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();
        }
        EXPECT_LT(AZStd::abs(m_vehicle->GetSpeed()), 0.5f);
    }

    TEST_F(JoltVehicleTests, ASphereGroundTestDrivesOnFlatGround)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(200.0f, 50.0f, 1.0f));
        JoltVehicleConfiguration config = MakeCarConfiguration();
        config.m_collisionTester = JoltVehicleCollisionTester::Sphere;
        CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.9f), config, AZ::Vector3(2.0f, 1.0f, 0.5f), 1200.0f);

        DriveSteps(0.0f, 0.0f, 0.0f, 60);
        EXPECT_TRUE(m_vehicle->IsWheelOnGround(0));
        DriveSteps(1.0f, 0.0f, 0.0f, 180);
        EXPECT_GT(m_vehicle->GetSpeed(), 3.0f);
    }

    TEST_F(JoltVehicleTests, IndividualInputSettersCompose)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(400.0f, 50.0f, 1.0f));
        CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.9f));

        DriveSteps(0.0f, 0.0f, 0.0f, 60);

        // Throttle through the single-channel setter alone.
        const float fixedDeltaTime = 1.0f / 60.0f;
        m_vehicle->SetForwardInput(1.0f);
        for (int i = 0; i < 180; ++i)
        {
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();
        }
        EXPECT_GT(m_vehicle->GetSpeed(), 3.0f);

        // The brake channel composes with (and here replaces) the held throttle.
        m_vehicle->SetForwardInput(0.0f);
        m_vehicle->SetBrakeInput(1.0f);
        for (int i = 0; i < 360; ++i)
        {
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();
        }
        EXPECT_LT(AZStd::abs(m_vehicle->GetSpeed()), 0.5f);
    }

    TEST_F(JoltVehicleTests, WheelStateReadoutsReportSpinSteerSlipAndContact)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(400.0f, 50.0f, 1.0f));
        CreateVehicle(AZ::Vector3(0.0f, 0.0f, 0.9f));

        DriveSteps(0.0f, 0.0f, 0.0f, 60);

        // Steering shows up on a steered wheel and not on a fixed one.
        DriveSteps(0.0f, 1.0f, 0.0f, 5);
        EXPECT_GT(AZStd::abs(m_vehicle->GetWheelSteerAngle(0)), 0.1f);
        EXPECT_NEAR(m_vehicle->GetWheelSteerAngle(2), 0.0f, 1e-3f);

        // Hard launch: the driven wheels spin up, and some longitudinal slip is visible
        // while the tire hunts for grip.
        float maxSlip = 0.0f;
        const float fixedDeltaTime = 1.0f / 60.0f;
        for (int i = 0; i < 120; ++i)
        {
            m_vehicle->SetDriverInput(1.0f, 0.0f, 0.0f, 0.0f);
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();
            maxSlip = AZStd::max(maxSlip, m_vehicle->GetWheelLongitudinalSlip(2));
        }
        EXPECT_GT(m_vehicle->GetWheelAngularVelocity(2), 1.0f);
        EXPECT_GT(maxSlip, 0.001f);

        // Contact point on the ground plane, normal pointing up, suspension not riding
        // its hard stop on flat ground.
        AZ::Vector3 contactPoint = AZ::Vector3::CreateZero();
        AZ::Vector3 contactNormal = AZ::Vector3::CreateZero();
        ASSERT_TRUE(m_vehicle->GetWheelContactPoint(0, contactPoint));
        ASSERT_TRUE(m_vehicle->GetWheelContactNormal(0, contactNormal));
        EXPECT_NEAR(contactPoint.GetZ(), 0.0f, 0.1f);
        EXPECT_GT(contactNormal.GetZ(), 0.99f);
        EXPECT_FALSE(m_vehicle->IsWheelSuspensionBottomedOut(0));
    }

    TEST_F(JoltVehicleTests, ExpandedConfigurationSerializationRoundTrip)
    {
        AZ::SerializeContext serializeContext;
        JoltWheelConfiguration::Reflect(&serializeContext);
        JoltVehicleConfiguration::Reflect(&serializeContext);

        JoltVehicleConfiguration source = MakeCarConfiguration();
        JoltVehicleDifferential front;
        front.m_leftWheel = 0;
        front.m_rightWheel = 1;
        front.m_leftRightSplit = 0.4f;
        front.m_limitedSlipRatio = 1.6f;
        front.m_engineTorqueRatio = 0.5f;
        source.m_differentials = { front };
        source.m_differentialLimitedSlipRatio = 1.7f;
        source.m_minEngineRpm = 850.0f;
        source.m_engineInertia = 0.8f;
        source.m_engineAngularDamping = 0.15f;
        source.m_engineTorqueCurve = { AZ::Vector2(0.0f, 0.6f), AZ::Vector2(1.0f, 0.9f) };
        source.m_transmissionMode = JoltVehicleTransmissionMode::Manual;
        source.m_gearSwitchTime = 0.35f;
        source.m_clutchReleaseTime = 0.2f;
        source.m_gearSwitchLatency = 0.6f;
        source.m_shiftUpRpm = 5200.0f;
        source.m_shiftDownRpm = 1500.0f;
        source.m_clutchStrength = 8.0f;
        source.m_antiRollBars = { JoltVehicleAntiRollBar{} };
        source.m_antiRollBars[0].m_leftWheel = 0;
        source.m_antiRollBars[0].m_rightWheel = 1;
        source.m_antiRollBars[0].m_stiffness = 4200.0f;
        source.m_leanSpringIntegrationCoefficient = 25.0f;
        source.m_leanSpringIntegrationCoefficientDecay = 3.0f;
        source.m_leanSmoothingFactor = 0.6f;
        source.m_trackInertia = 12.0f;
        source.m_leftTrackDrivenWheel = 1;
        source.m_rightTrackDrivenWheel = 5;
        source.m_wheels[0].m_longitudinalFrictionCurve = { AZ::Vector2(0.0f, 0.2f), AZ::Vector2(0.5f, 1.1f) };
        source.m_wheels[0].m_lateralFrictionCurve = { AZ::Vector2(3.0f, 1.2f) };
        source.m_wheels[0].m_inertia = 1.1f;
        source.m_wheels[0].m_angularDamping = 0.07f;
        source.m_wheels[0].m_suspensionPreloadLength = 0.05f;
        source.m_wheels[0].m_suspensionSpringMode = JoltSuspensionSpringMode::StiffnessAndDamping;
        source.m_wheels[0].m_suspensionForcePoint = AZ::Vector3(0.8f, 0.45f, -0.3f);
        source.m_wheels[0].m_enableSuspensionForcePoint = true;
        source.m_wheels[0].m_trackedLongitudinalFriction = 3.5f;
        source.m_wheels[0].m_trackedLateralFriction = 1.5f;

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

        ASSERT_EQ(roundTripped.m_differentials.size(), 1u);
        EXPECT_EQ(roundTripped.m_differentials[0].m_leftWheel, 0);
        EXPECT_FLOAT_EQ(roundTripped.m_differentials[0].m_leftRightSplit, 0.4f);
        EXPECT_FLOAT_EQ(roundTripped.m_differentials[0].m_limitedSlipRatio, 1.6f);
        EXPECT_FLOAT_EQ(roundTripped.m_differentials[0].m_engineTorqueRatio, 0.5f);
        EXPECT_FLOAT_EQ(roundTripped.m_differentialLimitedSlipRatio, 1.7f);
        EXPECT_FLOAT_EQ(roundTripped.m_minEngineRpm, 850.0f);
        EXPECT_FLOAT_EQ(roundTripped.m_engineInertia, 0.8f);
        ASSERT_EQ(roundTripped.m_engineTorqueCurve.size(), 2u);
        EXPECT_TRUE(roundTripped.m_engineTorqueCurve[1].IsClose(AZ::Vector2(1.0f, 0.9f)));
        EXPECT_EQ(roundTripped.m_transmissionMode, JoltVehicleTransmissionMode::Manual);
        EXPECT_FLOAT_EQ(roundTripped.m_gearSwitchTime, 0.35f);
        EXPECT_FLOAT_EQ(roundTripped.m_shiftUpRpm, 5200.0f);
        EXPECT_FLOAT_EQ(roundTripped.m_clutchStrength, 8.0f);
        ASSERT_EQ(roundTripped.m_antiRollBars.size(), 1u);
        EXPECT_FLOAT_EQ(roundTripped.m_antiRollBars[0].m_stiffness, 4200.0f);
        EXPECT_FLOAT_EQ(roundTripped.m_leanSpringIntegrationCoefficient, 25.0f);
        EXPECT_FLOAT_EQ(roundTripped.m_leanSmoothingFactor, 0.6f);
        EXPECT_FLOAT_EQ(roundTripped.m_trackInertia, 12.0f);
        EXPECT_EQ(roundTripped.m_leftTrackDrivenWheel, 1);
        EXPECT_EQ(roundTripped.m_rightTrackDrivenWheel, 5);
        ASSERT_EQ(roundTripped.m_wheels[0].m_longitudinalFrictionCurve.size(), 2u);
        EXPECT_TRUE(roundTripped.m_wheels[0].m_longitudinalFrictionCurve[1].IsClose(AZ::Vector2(0.5f, 1.1f)));
        ASSERT_EQ(roundTripped.m_wheels[0].m_lateralFrictionCurve.size(), 1u);
        EXPECT_EQ(roundTripped.m_wheels[0].m_suspensionSpringMode, JoltSuspensionSpringMode::StiffnessAndDamping);
        EXPECT_TRUE(roundTripped.m_wheels[0].m_suspensionForcePoint.IsClose(AZ::Vector3(0.8f, 0.45f, -0.3f)));
        EXPECT_TRUE(roundTripped.m_wheels[0].m_enableSuspensionForcePoint);
        EXPECT_FLOAT_EQ(roundTripped.m_wheels[0].m_inertia, 1.1f);
        EXPECT_FLOAT_EQ(roundTripped.m_wheels[0].m_trackedLongitudinalFriction, 3.5f);
    }

    //! Hands the fixture's scene out as the default world, which is where the vehicle
    //! component looks for its chassis.
    class VehicleTestDefaultWorld
        : public Physics::DefaultWorldBus::Handler
    {
    public:
        explicit VehicleTestDefaultWorld(AzPhysics::SceneHandle sceneHandle)
            : m_sceneHandle(sceneHandle)
        {
            Physics::DefaultWorldBus::Handler::BusConnect();
        }
        ~VehicleTestDefaultWorld() override
        {
            Physics::DefaultWorldBus::Handler::BusDisconnect();
        }
        AzPhysics::SceneHandle GetDefaultSceneHandle() const override
        {
            return m_sceneHandle;
        }

    private:
        AzPhysics::SceneHandle m_sceneHandle;
    };

    TEST_F(JoltVehicleTests, VehicleComponentActivatesDrivesTheBusAndRecreates)
    {
        VehicleTestDefaultWorld defaultWorld(m_sceneHandle);

        // Ground top at z = -0.6 so an entity at the origin has its wheels in suspension
        // range (the entity transform defaults to identity).
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -1.1f), AZ::Vector3(50.0f, 50.0f, 1.0f));

        auto entity = AZStd::make_unique<AZ::Entity>("VehicleComponentEntity");
        entity->CreateComponent<AzFramework::TransformComponent>();
        entity->CreateComponent<JoltBoxColliderComponent>();
        entity->CreateComponent<JoltRigidBodyComponent>();
        auto* vehicleComponent = entity->CreateComponent<JoltVehicleComponent>();
        vehicleComponent->GetConfiguration() = MakeCarConfiguration();
        entity->Init();
        entity->Activate();

        // The vehicle is created on tick, once the chassis body exists.
        AZ::TickBus::Broadcast(&AZ::TickBus::Events::OnTick, 0.016f, AZ::ScriptTimePoint());

        AZ::u32 wheelCount = 0;
        JoltVehicleRequestBus::EventResult(wheelCount, entity->GetId(), &JoltVehicleRequests::GetWheelCount);
        EXPECT_EQ(wheelCount, 4u);

        // A config edit lands through RecreateVehicle: shrink to a single-differential
        // three-wheeler... keep it simple and just drop a wheel.
        vehicleComponent->GetConfiguration().m_wheels.pop_back();
        JoltVehicleRequestBus::Event(entity->GetId(), &JoltVehicleRequests::RecreateVehicle);
        AZ::TickBus::Broadcast(&AZ::TickBus::Events::OnTick, 0.016f, AZ::ScriptTimePoint());

        wheelCount = 0;
        JoltVehicleRequestBus::EventResult(wheelCount, entity->GetId(), &JoltVehicleRequests::GetWheelCount);
        EXPECT_EQ(wheelCount, 3u);

        entity->Deactivate();
    }

} // namespace JoltPhysics
