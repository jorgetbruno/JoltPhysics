#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <System/JoltSystem.h>
#include <Scene/JoltScene.h>
#include <Configuration/JoltSettingsRegistryManager.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    class JoltTriggerTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "TriggerTestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);
        }

        void TearDown() override
        {
            m_system->RemoveScene(m_sceneHandle);
            m_system->Shutdown();
            m_system.reset();
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

    TEST_F(JoltTriggerTests, FallingBodyFiresEnterAndExitOnTriggerVolume)
    {
        // Static trigger volume (sensor) spanning z in [-1, 1].
        auto triggerCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        triggerCollider->m_isTrigger = true;
        auto triggerShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        triggerShape->m_dimensions = AZ::Vector3(10.0f, 10.0f, 2.0f);

        AzPhysics::StaticRigidBodyConfiguration triggerConfig;
        triggerConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(triggerCollider, triggerShape);
        auto triggerHandle = m_scene->AddSimulatedBody(&triggerConfig);
        ASSERT_NE(triggerHandle, AzPhysics::InvalidSimulatedBodyHandle);

        // Dynamic box falling through the volume from z=3.
        auto boxCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        AzPhysics::RigidBodyConfiguration boxConfig;
        boxConfig.m_position = AZ::Vector3(0.0f, 0.0f, 3.0f);
        boxConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(boxCollider, boxShape);
        auto boxHandle = m_scene->AddSimulatedBody(&boxConfig);
        ASSERT_NE(boxHandle, AzPhysics::InvalidSimulatedBodyHandle);

        int enterCount = 0;
        int exitCount = 0;
        AzPhysics::SimulatedBodyHandle lastOther = AzPhysics::InvalidSimulatedBodyHandle;

        auto* triggerBody = m_scene->GetSimulatedBodyFromHandle(triggerHandle);
        ASSERT_NE(triggerBody, nullptr);

        AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler enterHandler(
            [&](AzPhysics::SimulatedBodyHandle /*bodyHandle*/, const AzPhysics::TriggerEvent& triggerEvent)
            {
                ++enterCount;
                lastOther = triggerEvent.m_otherBodyHandle;
            });
        AzPhysics::SimulatedBodyEvents::OnTriggerExit::Handler exitHandler(
            [&](AzPhysics::SimulatedBodyHandle /*bodyHandle*/, const AzPhysics::TriggerEvent& /*triggerEvent*/)
            {
                ++exitCount;
            });
        triggerBody->RegisterOnTriggerEnterHandler(enterHandler);
        triggerBody->RegisterOnTriggerExitHandler(exitHandler);

        SimulateSeconds(0.7f); // box bottom reaches the volume top at ~0.55s, still inside
        EXPECT_EQ(enterCount, 1);
        EXPECT_EQ(lastOther, boxHandle);
        EXPECT_EQ(exitCount, 0);

        SimulateSeconds(2.0f); // box passes through and out the bottom
        EXPECT_EQ(enterCount, 1);
        EXPECT_EQ(exitCount, 1);
    }

    TEST_F(JoltTriggerTests, NonTriggerBodyDoesNotFireTriggerEvents)
    {
        // Same setup but the static volume is NOT a trigger: no events, box rests on it.
        auto slabCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto slabShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        slabShape->m_dimensions = AZ::Vector3(10.0f, 10.0f, 2.0f);

        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(slabCollider, slabShape);
        auto slabHandle = m_scene->AddSimulatedBody(&slabConfig);

        auto* slabBody = m_scene->GetSimulatedBodyFromHandle(slabHandle);
        ASSERT_NE(slabBody, nullptr);

        int enterCount = 0;
        AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler enterHandler(
            [&](AzPhysics::SimulatedBodyHandle, const AzPhysics::TriggerEvent&) { ++enterCount; });
        slabBody->RegisterOnTriggerEnterHandler(enterHandler);

        auto boxCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        AzPhysics::RigidBodyConfiguration boxConfig;
        boxConfig.m_position = AZ::Vector3(0.0f, 0.0f, 3.0f);
        boxConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(boxCollider, boxShape);
        auto boxHandle = m_scene->AddSimulatedBody(&boxConfig);

        SimulateSeconds(2.0f);

        EXPECT_EQ(enterCount, 0);
        EXPECT_NEAR(m_scene->GetSimulatedBodyFromHandle(boxHandle)->GetPosition().GetZ(), 1.5f, 0.2f);
    }

} // namespace JoltPhysics
