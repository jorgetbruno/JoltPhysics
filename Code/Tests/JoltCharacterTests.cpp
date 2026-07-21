#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <Character/JoltCharacter.h>
#include <Configuration/JoltSettingsRegistryManager.h>
#include <RigidBody/JoltRigidBody.h>
#include <Scene/JoltScene.h>
#include <System/JoltSystem.h>

#include <AzFramework/Physics/Common/PhysicsSimulatedBodyEvents.h>
#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    class JoltCharacterTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "CharacterTestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);
        }

        void TearDown() override
        {
            m_system->RemoveScene(m_sceneHandle);
            m_system->Shutdown();
            m_system.reset();
        }

        AzPhysics::SimulatedBodyHandle CreateStaticBox(
            const AZ::Vector3& position, const AZ::Vector3& dimensions,
            const AZ::Quaternion& orientation = AZ::Quaternion::CreateIdentity(), bool isTrigger = false)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            colliderConfig->m_isTrigger = isTrigger;
            auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            boxShape->m_dimensions = dimensions;

            AzPhysics::StaticRigidBodyConfiguration staticConfig;
            staticConfig.m_position = position;
            staticConfig.m_orientation = orientation;
            staticConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);
            return m_scene->AddSimulatedBody(&staticConfig);
        }

        AzPhysics::SimulatedBodyHandle CreateDynamicBox(const AZ::Vector3& position, float mass = 1.0f)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();

            AzPhysics::RigidBodyConfiguration boxConfig;
            boxConfig.m_position = position;
            boxConfig.m_mass = mass;
            boxConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);
            return m_scene->AddSimulatedBody(&boxConfig);
        }

        //! Creates a 1.8 m x 0.3 m capsule character; position is the capsule center.
        AzPhysics::SimulatedBodyHandle CreateCharacter(
            const AZ::Vector3& position, float slopeLimitDegrees = 30.0f, float stepHeight = 0.5f)
        {
            Physics::CharacterConfiguration config;
            config.m_position = position;
            config.m_entityId = AZ::EntityId(0xCAFE);
            config.m_debugName = "TestCharacter";
            config.m_maximumSlopeAngle = slopeLimitDegrees;
            config.m_stepHeight = stepHeight;
            config.m_shapeConfig = AZStd::make_shared<Physics::CapsuleShapeConfiguration>(1.8f, 0.3f);
            return m_scene->AddSimulatedBody(&config);
        }

        JoltCharacter* GetCharacter(AzPhysics::SimulatedBodyHandle handle)
        {
            return azdynamic_cast<JoltCharacter*>(m_scene->GetSimulatedBodyFromHandle(handle));
        }

        //! Requests the given velocity for the character each step and simulates.
        void WalkCharacter(JoltCharacter* character, const AZ::Vector3& velocity, float seconds)
        {
            const float fixedDeltaTime = 1.0f / 60.0f;
            const int steps = static_cast<int>(seconds / fixedDeltaTime);
            for (int i = 0; i < steps; ++i)
            {
                character->AddVelocityForTick(velocity);
                m_scene->StartSimulation(fixedDeltaTime);
                m_scene->FinishSimulation();
            }
        }

        void SimulateSeconds(float seconds)
        {
            const float fixedDeltaTime = 1.0f / 60.0f;
            const int steps = static_cast<int>(seconds / fixedDeltaTime);
            for (int i = 0; i < steps; ++i)
            {
                m_scene->StartSimulation(fixedDeltaTime);
                m_scene->FinishSimulation();
            }
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle;
        AzPhysics::Scene* m_scene = nullptr;
    };

    TEST_F(JoltCharacterTests, FallsAndLandsOnGround)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(20.0f, 20.0f, 1.0f));
        auto characterHandle = CreateCharacter(AZ::Vector3(0.0f, 0.0f, 3.0f));
        JoltCharacter* character = GetCharacter(characterHandle);
        ASSERT_NE(character, nullptr);

        // Drive the fall like gameplay code would: accumulate gravity while airborne,
        // clamp to a small stick velocity while grounded.
        AZ::Vector3 velocity = AZ::Vector3::CreateZero();
        const float fixedDeltaTime = 1.0f / 60.0f;
        for (int i = 0; i < 180; ++i)
        {
            if (character->IsOnGround())
            {
                velocity.SetZ(-0.5f);
            }
            else
            {
                velocity += AZ::Vector3(0.0f, 0.0f, -9.81f) * fixedDeltaTime;
            }
            character->AddVelocityForTick(velocity);
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();
        }

        // Capsule center should rest at half height (0.9 m) above the ground (z=0).
        EXPECT_TRUE(character->IsOnGround());
        EXPECT_NEAR(character->GetCenterPosition().GetZ(), 0.9f, 0.05f);
        EXPECT_NEAR(character->GetBasePosition().GetZ(), 0.0f, 0.05f);
        EXPECT_NEAR(character->GetGroundNormal().GetZ(), 1.0f, 0.01f);
    }

    TEST_F(JoltCharacterTests, SlopeLimitBlocksClimbingSteepSlope)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(40.0f, 40.0f, 1.0f));
        // 45 degree ramp rising towards +x.
        const AZ::Quaternion rampOrientation = AZ::Quaternion::CreateFromAxisAngle(AZ::Vector3::CreateAxisY(), -AZ::DegToRad(45.0f));
        CreateStaticBox(AZ::Vector3(5.0f, 0.0f, 2.5f), AZ::Vector3(8.0f, 4.0f, 0.5f), rampOrientation);

        auto characterHandle = CreateCharacter(AZ::Vector3(0.0f, 0.0f, 1.0f), 30.0f /* slope limit */);
        JoltCharacter* character = GetCharacter(characterHandle);
        ASSERT_NE(character, nullptr);

        // Settle, then try to walk up the 45 degree slope (above the 30 degree limit).
        SimulateSeconds(0.5f);
        WalkCharacter(character, AZ::Vector3(3.0f, 0.0f, -1.0f), 2.0f);

        // The character must not reach the top of the ramp (which is ~3 m higher).
        EXPECT_LT(character->GetCenterPosition().GetZ(), 2.5f);
    }

    TEST_F(JoltCharacterTests, StepHeightClimbsStepWithinLimit)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(20.0f, 20.0f, 1.0f));
        // 0.3 m tall, 4 m deep platform (top surface at z=0.3, spans x in [1.5, 5.5]).
        CreateStaticBox(AZ::Vector3(3.5f, 0.0f, 0.15f), AZ::Vector3(4.0f, 4.0f, 0.3f));

        auto characterHandle = CreateCharacter(AZ::Vector3(0.0f, 0.0f, 1.0f), 30.0f, 0.5f /* step height */);
        JoltCharacter* character = GetCharacter(characterHandle);
        ASSERT_NE(character, nullptr);

        SimulateSeconds(0.5f);
        // Walk forward; gravity-ish downward component keeps the character grounded.
        WalkCharacter(character, AZ::Vector3(1.5f, 0.0f, -1.0f), 2.0f);

        // The character should have stepped up and be standing on the platform.
        EXPECT_GT(character->GetCenterPosition().GetX(), 2.5f);
        EXPECT_NEAR(character->GetBasePosition().GetZ(), 0.3f, 0.1f);
    }

    TEST_F(JoltCharacterTests, StepHeightTooLowBlocksStep)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(20.0f, 20.0f, 1.0f));
        // 0.6 m tall step at x=2 (top surface at z=0.6).
        CreateStaticBox(AZ::Vector3(2.0f, 0.0f, 0.3f), AZ::Vector3(1.0f, 4.0f, 0.6f));

        auto characterHandle = CreateCharacter(AZ::Vector3(0.0f, 0.0f, 1.0f), 30.0f, 0.4f /* step height */);
        JoltCharacter* character = GetCharacter(characterHandle);
        ASSERT_NE(character, nullptr);

        SimulateSeconds(0.5f);
        WalkCharacter(character, AZ::Vector3(1.5f, 0.0f, -1.0f), 2.0f);

        // The 0.6 m step exceeds the 0.4 m step height: the character must stay in front.
        EXPECT_LT(character->GetCenterPosition().GetX(), 2.0f);
    }

    TEST_F(JoltCharacterTests, StickToFloorWalkingDownSlope)
    {
        // 20 degree downslope descending towards +x.
        const AZ::Quaternion slopeOrientation = AZ::Quaternion::CreateFromAxisAngle(AZ::Vector3::CreateAxisY(), AZ::DegToRad(20.0f));
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 0.0f), AZ::Vector3(20.0f, 10.0f, 0.5f), slopeOrientation);

        auto characterHandle = CreateCharacter(AZ::Vector3(-4.0f, 0.0f, 2.5f));
        JoltCharacter* character = GetCharacter(characterHandle);
        ASSERT_NE(character, nullptr);

        // Let the character settle on the slope.
        WalkCharacter(character, AZ::Vector3(0.0f, 0.0f, -2.0f), 0.5f);
        ASSERT_TRUE(character->IsOnGround());

        // Walk downhill; without stick-to-floor the character would go airborne
        // on every step. Allow a small number of airborne steps at the start.
        int airborneSteps = 0;
        const float fixedDeltaTime = 1.0f / 60.0f;
        for (int i = 0; i < 60; ++i)
        {
            character->AddVelocityForTick(AZ::Vector3(2.0f, 0.0f, -1.0f));
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();
            if (!character->IsOnGround())
            {
                ++airborneSteps;
            }
        }
        EXPECT_LT(airborneSteps, 10);
    }

    TEST_F(JoltCharacterTests, PushesDynamicBox)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(20.0f, 20.0f, 1.0f));
        auto boxHandle = CreateDynamicBox(AZ::Vector3(2.0f, 0.0f, 0.5f), 1.0f /* mass */);

        auto characterHandle = CreateCharacter(AZ::Vector3(0.0f, 0.0f, 1.0f));
        JoltCharacter* character = GetCharacter(characterHandle);
        ASSERT_NE(character, nullptr);

        SimulateSeconds(0.5f);
        WalkCharacter(character, AZ::Vector3(2.0f, 0.0f, -1.0f), 2.0f);

        auto* box = azdynamic_cast<AzPhysics::RigidBody*>(m_scene->GetSimulatedBodyFromHandle(boxHandle));
        ASSERT_NE(box, nullptr);
        // The light box should have been pushed away from its spawn x=2.
        EXPECT_GT(box->GetPosition().GetX(), 2.3f);
    }

    TEST_F(JoltCharacterTests, SensorFiresTriggerEventsForCharacter)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(20.0f, 20.0f, 1.0f));
        auto triggerHandle = CreateStaticBox(
            AZ::Vector3(2.0f, 0.0f, 1.0f), AZ::Vector3(2.0f, 4.0f, 2.0f),
            AZ::Quaternion::CreateIdentity(), true /* isTrigger */);

        int enterCount = 0;
        int exitCount = 0;
        AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler enterHandler(
            [&enterCount]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, [[maybe_unused]] const AzPhysics::TriggerEvent& triggerEvent)
            {
                ++enterCount;
            });
        AzPhysics::SimulatedBodyEvents::OnTriggerExit::Handler exitHandler(
            [&exitCount]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, [[maybe_unused]] const AzPhysics::TriggerEvent& triggerEvent)
            {
                ++exitCount;
            });

        AzPhysics::SimulatedBody* triggerBody = m_scene->GetSimulatedBodyFromHandle(triggerHandle);
        ASSERT_NE(triggerBody, nullptr);
        triggerBody->RegisterOnTriggerEnterHandler(enterHandler);
        triggerBody->RegisterOnTriggerExitHandler(exitHandler);

        auto characterHandle = CreateCharacter(AZ::Vector3(0.0f, 0.0f, 1.0f));
        JoltCharacter* character = GetCharacter(characterHandle);
        ASSERT_NE(character, nullptr);

        SimulateSeconds(0.5f);
        // Walk through the trigger volume (centered at x=2, half extent 1) and out the other side.
        WalkCharacter(character, AZ::Vector3(2.0f, 0.0f, -1.0f), 3.0f);

        EXPECT_GE(enterCount, 1);
        EXPECT_GE(exitCount, 1);
    }

} // namespace JoltPhysics
