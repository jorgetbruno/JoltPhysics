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
    class JoltCollisionFilteringTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "FilteringTestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);
        }

        void TearDown() override
        {
            m_system->RemoveScene(m_sceneHandle);
            m_system->Shutdown();
            m_system.reset();
        }

        AzPhysics::SimulatedBodyHandle CreateBox(
            const AZ::Vector3& position,
            AZ::u8 collisionLayer,
            const AZStd::string& collisionGroupName,
            AZ::u64 collisionGroupMask,
            bool isStatic)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            colliderConfig->m_collisionLayer = AzPhysics::CollisionLayer(collisionLayer);
            colliderConfig->m_collisionGroupId =
                m_system->CreateCollisionGroupPreset(
                    collisionGroupName, AzPhysics::CollisionGroup(collisionGroupMask));

            auto shapeConfig = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            shapeConfig->m_dimensions = AZ::Vector3(1.0f, 1.0f, 1.0f);

            if (isStatic)
            {
                AzPhysics::StaticRigidBodyConfiguration staticConfig;
                staticConfig.m_position = position;
                staticConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, shapeConfig);
                return m_scene->AddSimulatedBody(&staticConfig);
            }

            AzPhysics::RigidBodyConfiguration dynamicConfig;
            dynamicConfig.m_position = position;
            dynamicConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, shapeConfig);
            return m_scene->AddSimulatedBody(&dynamicConfig);
        }

        AzPhysics::SimulatedBodyHandle CreateSlab(
            const AZStd::string& collisionGroupName, AZ::u64 collisionGroupMask, AZ::u8 collisionLayer = 0)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            colliderConfig->m_collisionLayer = AzPhysics::CollisionLayer(collisionLayer);
            colliderConfig->m_collisionGroupId =
                m_system->CreateCollisionGroupPreset(
                    collisionGroupName, AzPhysics::CollisionGroup(collisionGroupMask));

            auto shapeConfig = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            shapeConfig->m_dimensions = AZ::Vector3(20.0f, 20.0f, 1.0f);

            AzPhysics::StaticRigidBodyConfiguration staticConfig;
            staticConfig.m_position = AZ::Vector3(0.0f, 0.0f, -0.5f); // top surface at z=0
            staticConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, shapeConfig);
            return m_scene->AddSimulatedBody(&staticConfig);
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

        float GetBodyZ(AzPhysics::SimulatedBodyHandle handle) const
        {
            return m_scene->GetSimulatedBodyFromHandle(handle)->GetPosition().GetZ();
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle;
        AzPhysics::Scene* m_scene = nullptr;
    };

    TEST_F(JoltCollisionFilteringTests, BodiesCollideWhenBothGroupMasksContainEachOthersLayer)
    {
        CreateSlab("SlabGroup", /*mask*/ 0b11); // contains layers 0 and 1
        auto box = CreateBox(AZ::Vector3(0.0f, 0.0f, 2.0f), /*layer*/ 1, "BoxGroup", /*mask*/ 0b11, false);

        SimulateSeconds(1.5f);

        EXPECT_NEAR(GetBodyZ(box), 0.5f, 0.2f);
    }

    TEST_F(JoltCollisionFilteringTests, BodyFallsThroughWhenSlabGroupMaskExcludesBodyLayer)
    {
        CreateSlab("SlabGroup", /*mask*/ 0b01); // contains only layer 0
        auto box = CreateBox(AZ::Vector3(0.0f, 0.0f, 2.0f), /*layer*/ 1, "BoxGroup", /*mask*/ 0b11, false);

        SimulateSeconds(1.5f);

        EXPECT_LT(GetBodyZ(box), -5.0f);
    }

    TEST_F(JoltCollisionFilteringTests, BodyFallsThroughWhenOwnGroupMaskExcludesSlabLayer)
    {
        CreateSlab("SlabGroup", /*mask*/ 0b11);
        auto box = CreateBox(AZ::Vector3(0.0f, 0.0f, 2.0f), /*layer*/ 1, "BoxGroup", /*mask*/ 0b10, false);

        SimulateSeconds(1.5f);

        EXPECT_LT(GetBodyZ(box), -5.0f);
    }

    TEST_F(JoltCollisionFilteringTests, KinematicBodyStaysAndSupportsDynamicBody)
    {
        auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto slabShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        slabShape->m_dimensions = AZ::Vector3(20.0f, 20.0f, 1.0f);

        AzPhysics::RigidBodyConfiguration kinematicConfig;
        kinematicConfig.m_kinematic = true;
        kinematicConfig.m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        kinematicConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, slabShape);
        auto kinematicSlab = m_scene->AddSimulatedBody(&kinematicConfig);

        auto box = CreateBox(AZ::Vector3(0.0f, 0.0f, 2.0f), 0, "BoxGroup", 0b01, false);

        SimulateSeconds(1.5f);

        EXPECT_NEAR(GetBodyZ(kinematicSlab), -0.5f, 0.01f);
        EXPECT_NEAR(GetBodyZ(box), 0.5f, 0.2f);
    }

} // namespace JoltPhysics
