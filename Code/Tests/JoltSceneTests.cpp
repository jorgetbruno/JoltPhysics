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
    class JoltSceneTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "TestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);
        }

        void TearDown() override
        {
            m_system->RemoveScene(m_sceneHandle);
            m_system->Shutdown();
            m_system.reset();
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle;
        AzPhysics::Scene* m_scene = nullptr;
    };

    TEST_F(JoltSceneTests, SceneIsValid)
    {
        ASSERT_NE(m_scene, nullptr);
        EXPECT_TRUE(m_scene->IsEnabled());
    }

    TEST_F(JoltSceneTests, GravityCanBeSet)
    {
        AZ::Vector3 newGravity(0.0f, 0.0f, -20.0f);
        m_scene->SetGravity(newGravity);

        AZ::Vector3 gravity = m_scene->GetGravity();
        EXPECT_FLOAT_EQ(gravity.GetZ(), -20.0f);
    }

    TEST_F(JoltSceneTests, SceneCanBeDisabled)
    {
        m_scene->SetEnabled(false);
        EXPECT_FALSE(m_scene->IsEnabled());

        m_scene->SetEnabled(true);
        EXPECT_TRUE(m_scene->IsEnabled());
    }

    TEST_F(JoltSceneTests, RigidBodyCanBeAdded)
    {
        AzPhysics::RigidBodyConfiguration rigidBodyConfig;
        rigidBodyConfig.m_debugName = "TestBody";
        rigidBodyConfig.m_position = AZ::Vector3(0.0f, 0.0f, 10.0f);
        rigidBodyConfig.m_mass = 1.0f;

        AzPhysics::SimulatedBodyHandle handle = m_scene->AddSimulatedBody(&rigidBodyConfig);

        EXPECT_NE(handle, AzPhysics::InvalidSimulatedBodyHandle);

        AzPhysics::SimulatedBody* body = m_scene->GetSimulatedBodyFromHandle(handle);
        EXPECT_NE(body, nullptr);

        m_scene->RemoveSimulatedBody(handle);
    }

    TEST_F(JoltSceneTests, StaticBodyColliderOffsetIsApplied)
    {
        AzPhysics::StaticRigidBodyConfiguration staticConfig;
        staticConfig.m_debugName = "Ground";

        auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
        colliderConfig->m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        auto shapeConfig = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        shapeConfig->m_dimensions = AZ::Vector3(10.0f, 10.0f, 1.0f);

        staticConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, shapeConfig);

        AzPhysics::SimulatedBodyHandle bodyHandle = m_scene->AddSimulatedBody(&staticConfig);
        ASSERT_NE(bodyHandle, AzPhysics::InvalidSimulatedBodyHandle);

        // The 10x10x1 slab is offset -0.5 along z, so its top surface is at z=0.
        AzPhysics::RayCastRequest request;
        request.m_start = AZ::Vector3(0.0f, 0.0f, 5.0f);
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 100.0f;

        AzPhysics::SceneQueryHits hits = m_scene->QueryScene(&request);
        ASSERT_EQ(hits.m_hits.size(), 1u);
        EXPECT_NEAR(hits.m_hits[0].m_position.GetZ(), 0.0f, 0.01f);

        m_scene->RemoveSimulatedBody(bodyHandle);
    }

    TEST_F(JoltSceneTests, CapsuleRestsUprightOnSlab)
    {
        // Static slab with top at z=0.
        auto slabCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        slabCollider->m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        auto slabShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        slabShape->m_dimensions = AZ::Vector3(20.0f, 20.0f, 1.0f);
        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(slabCollider, slabShape);
        m_scene->AddSimulatedBody(&slabConfig);

        // Dynamic capsule (default 1.0m tall, 0.25m radius) dropped from z=3.
        auto capsuleCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto capsuleShape = AZStd::make_shared<Physics::CapsuleShapeConfiguration>();
        AzPhysics::RigidBodyConfiguration capsuleConfig;
        capsuleConfig.m_position = AZ::Vector3(-3.0f, 0.0f, 4.0f);
        capsuleConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(capsuleCollider, capsuleShape);
        auto capsuleHandle = m_scene->AddSimulatedBody(&capsuleConfig);

        const float fixedDeltaTime = 1.0f / 60.0f;
        for (int i = 0; i < 600; ++i)
        {
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();
        }

        // A Z-up capsule resting upright on the slab has its center at z=0.5.
        const float capsuleZ = m_scene->GetSimulatedBodyFromHandle(capsuleHandle)->GetPosition().GetZ();
        EXPECT_NEAR(capsuleZ, 0.5f, 0.05f);
    }

} // namespace JoltPhysics
