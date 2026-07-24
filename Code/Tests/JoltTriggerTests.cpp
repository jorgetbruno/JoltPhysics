#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/Debug/TraceMessageBus.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/string/string.h>

#include <System/JoltSystem.h>
#include <Scene/JoltScene.h>
#include <RigidBody/JoltStaticRigidBody.h>
#include <Configuration/JoltSettingsRegistryManager.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    //! Collects gem warnings so a test can assert that a diagnostic actually fired
    //! (and keeps them out of the test output while connected).
    class JoltWarningCatcher : public AZ::Debug::TraceMessageBus::Handler
    {
    public:
        JoltWarningCatcher()
        {
            BusConnect();
        }
        ~JoltWarningCatcher() override
        {
            BusDisconnect();
        }

        bool OnPreWarning(
            const char* window, const char*, int, const char*, const char* message) override
        {
            if (window && AZStd::string_view(window) == "JoltPhysics")
            {
                m_warnings.push_back(message ? message : "");
            }
            return true; // handled; do not print
        }

        bool ContainsWarningWith(AZStd::string_view substring) const
        {
            for (const AZStd::string& warning : m_warnings)
            {
                if (warning.contains(substring))
                {
                    return true;
                }
            }
            return false;
        }

        AZStd::vector<AZStd::string> m_warnings;
    };

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

    TEST_F(JoltTriggerTests, RemovingBodyInsideTriggerFiresExit)
    {
        // A body removed while overlapping a sensor must still produce OnTriggerExit on the
        // sensor -- Jolt's own OnContactRemoved for the overlap only arrives a step later,
        // after the removed body's id->handle mapping is gone.
        auto triggerCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        triggerCollider->m_isTrigger = true;
        auto triggerShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        triggerShape->m_dimensions = AZ::Vector3(10.0f, 10.0f, 2.0f);

        AzPhysics::StaticRigidBodyConfiguration triggerConfig;
        triggerConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(triggerCollider, triggerShape);
        auto triggerHandle = m_scene->AddSimulatedBody(&triggerConfig);
        ASSERT_NE(triggerHandle, AzPhysics::InvalidSimulatedBodyHandle);

        // Dynamic box starting inside the volume (z in [-0.5, 0.5], sensor spans [-1, 1]).
        auto boxCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        AzPhysics::RigidBodyConfiguration boxConfig;
        boxConfig.m_position = AZ::Vector3(0.0f, 0.0f, 0.0f);
        boxConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(boxCollider, boxShape);
        auto boxHandle = m_scene->AddSimulatedBody(&boxConfig);
        ASSERT_NE(boxHandle, AzPhysics::InvalidSimulatedBodyHandle);
        const AzPhysics::SimulatedBodyHandle boxHandleValue = boxHandle;

        int enterCount = 0;
        int exitCount = 0;
        AzPhysics::SimulatedBodyHandle exitOther = AzPhysics::InvalidSimulatedBodyHandle;

        auto* triggerBody = m_scene->GetSimulatedBodyFromHandle(triggerHandle);
        ASSERT_NE(triggerBody, nullptr);

        AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler enterHandler(
            [&](AzPhysics::SimulatedBodyHandle, const AzPhysics::TriggerEvent&) { ++enterCount; });
        AzPhysics::SimulatedBodyEvents::OnTriggerExit::Handler exitHandler(
            [&](AzPhysics::SimulatedBodyHandle, const AzPhysics::TriggerEvent& triggerEvent)
            {
                ++exitCount;
                exitOther = triggerEvent.m_otherBodyHandle;
            });
        triggerBody->RegisterOnTriggerEnterHandler(enterHandler);
        triggerBody->RegisterOnTriggerExitHandler(exitHandler);

        SimulateSeconds(0.2f); // box is inside -> Enter fires, no Exit yet
        ASSERT_GE(enterCount, 1);
        EXPECT_EQ(exitCount, 0);

        m_scene->RemoveSimulatedBody(boxHandle);
        SimulateSeconds(0.1f); // one flush is enough

        EXPECT_GE(exitCount, 1);
        EXPECT_EQ(exitOther, boxHandleValue); // Exit correctly identifies the removed body
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

    TEST_F(JoltTriggerTests, CompoundTriggerFlagIsDecidedByTheFirstCollider)
    {
        // Jolt sensors are per-body, so a compound cannot mix trigger and solid colliders:
        // the first collider's flag decides for the whole body and the rest are ignored
        // (with a warning at creation). This pins that documented behaviour.
        auto makeCollider = [](float offsetX, bool isTrigger)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            colliderConfig->m_position = AZ::Vector3(offsetX, 0.0f, 0.0f);
            colliderConfig->m_isTrigger = isTrigger;
            auto shapeConfig = AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(2.0f, 2.0f, 2.0f));
            return AzPhysics::ShapeColliderPair(colliderConfig, shapeConfig);
        };

        // Collider 0 is solid, collider 1 asks to be a trigger: the body is solid.
        AzPhysics::StaticRigidBodyConfiguration config;
        config.m_debugName = "MixedTriggerCompound";
        config.m_colliderAndShapeData = AzPhysics::ShapeColliderPairList{
            makeCollider(-3.0f, /*isTrigger*/ false),
            makeCollider(3.0f, /*isTrigger*/ true),
        };

        AzPhysics::SimulatedBodyHandle compoundHandle = AzPhysics::InvalidSimulatedBodyHandle;
        {
            // The mismatch is reported rather than silently ignored.
            JoltWarningCatcher warnings;
            compoundHandle = m_scene->AddSimulatedBody(&config);
            EXPECT_TRUE(warnings.ContainsWarningWith("MixedTriggerCompound"));
        }
        ASSERT_NE(compoundHandle, AzPhysics::InvalidSimulatedBodyHandle);

        auto* compoundBody = azdynamic_cast<JoltStaticRigidBody*>(m_scene->GetSimulatedBodyFromHandle(compoundHandle));
        ASSERT_NE(compoundBody, nullptr);
        EXPECT_FALSE(compoundBody->IsSensor());

        // A box dropped onto the would-be trigger half rests on it instead of passing
        // through, confirming the whole body is solid.
        auto boxCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        AzPhysics::RigidBodyConfiguration boxConfig;
        boxConfig.m_position = AZ::Vector3(3.0f, 0.0f, 5.0f);
        boxConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(boxCollider, boxShape);
        auto boxHandle = m_scene->AddSimulatedBody(&boxConfig);

        SimulateSeconds(2.0f);

        // The compound's colliders span z in [-1, 1], so a unit box rests at z = 1.5.
        EXPECT_NEAR(m_scene->GetSimulatedBodyFromHandle(boxHandle)->GetPosition().GetZ(), 1.5f, 0.1f);
    }

} // namespace JoltPhysics
