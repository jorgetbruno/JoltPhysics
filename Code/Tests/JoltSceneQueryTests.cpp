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
    class JoltSceneQueryTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "QueryTestScene";
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
            const AZ::Vector3& position, const AZ::Vector3& dimensions, AZ::u8 collisionLayer = 0)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            colliderConfig->m_collisionLayer = AzPhysics::CollisionLayer(collisionLayer);
            auto shapeConfig = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            shapeConfig->m_dimensions = dimensions;

            AzPhysics::StaticRigidBodyConfiguration staticConfig;
            staticConfig.m_position = position;
            staticConfig.m_entityId = AZ::EntityId(12345 + collisionLayer);
            staticConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, shapeConfig);
            return m_scene->AddSimulatedBody(&staticConfig);
        }

        AzPhysics::RayCastRequest CreateRayDown(const AZ::Vector3& start, float distance = 100.0f)
        {
            AzPhysics::RayCastRequest request;
            request.m_start = start;
            request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
            request.m_distance = distance;
            return request;
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle;
        AzPhysics::Scene* m_scene = nullptr;
    };

    TEST_F(JoltSceneQueryTests, RaycastReportsCompleteHit)
    {
        auto slab = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(20.0f, 20.0f, 1.0f));

        auto request = CreateRayDown(AZ::Vector3(0.0f, 0.0f, 5.0f));
        AzPhysics::SceneQueryHits hits = m_scene->QueryScene(&request);

        ASSERT_EQ(hits.m_hits.size(), 1u);
        const AzPhysics::SceneQueryHit& hit = hits.m_hits[0];
        EXPECT_NEAR(hit.m_distance, 5.0f, 0.01f);
        EXPECT_NEAR(hit.m_position.GetZ(), 0.0f, 0.01f);
        EXPECT_NEAR(hit.m_normal.GetZ(), 1.0f, 0.01f);
        EXPECT_EQ(hit.m_bodyHandle, slab);
        EXPECT_EQ(hit.m_entityId, AZ::EntityId(12345));
    }

    TEST_F(JoltSceneQueryTests, RaycastMissesWhenNothingInPath)
    {
        CreateStaticBox(AZ::Vector3(50.0f, 0.0f, 0.0f), AZ::Vector3(1.0f, 1.0f, 1.0f));

        auto request = CreateRayDown(AZ::Vector3(0.0f, 0.0f, 5.0f));
        AzPhysics::SceneQueryHits hits = m_scene->QueryScene(&request);

        EXPECT_TRUE(hits.m_hits.empty());
    }

    TEST_F(JoltSceneQueryTests, RaycastHonorsCollisionGroupFilter)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 0.0f), AZ::Vector3(20.0f, 20.0f, 1.0f), /*layer*/ 0);
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(1.0f, 1.0f, 1.0f), /*layer*/ 1);

        // Query colliding only with layer 1: hits the small box at z=5, not the slab on layer 0.
        auto request = CreateRayDown(AZ::Vector3(0.0f, 0.0f, 10.0f));
        request.m_collisionGroup = AzPhysics::CollisionGroup(0b10);
        AzPhysics::SceneQueryHits hits = m_scene->QueryScene(&request);

        ASSERT_EQ(hits.m_hits.size(), 1u);
        EXPECT_NEAR(hits.m_hits[0].m_position.GetZ(), 5.5f, 0.01f);
    }

    TEST_F(JoltSceneQueryTests, RaycastHonorsStaticOnlyQueryType)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 0.0f), AZ::Vector3(20.0f, 20.0f, 1.0f));

        AzPhysics::RigidBodyConfiguration dynamicConfig;
        dynamicConfig.m_position = AZ::Vector3(0.0f, 0.0f, 5.0f);
        auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        auto boxCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        dynamicConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(boxCollider, boxShape);
        m_scene->AddSimulatedBody(&dynamicConfig);

        auto request = CreateRayDown(AZ::Vector3(0.0f, 0.0f, 10.0f));
        request.m_queryType = AzPhysics::SceneQuery::QueryType::Static;
        request.m_reportMultipleHits = true;
        AzPhysics::SceneQueryHits hits = m_scene->QueryScene(&request);

        // Only the static slab is reported, the dynamic box is filtered out.
        ASSERT_EQ(hits.m_hits.size(), 1u);
        EXPECT_NEAR(hits.m_hits[0].m_position.GetZ(), 0.5f, 0.01f);
    }

    TEST_F(JoltSceneQueryTests, SphereCastDownReportsHit)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(20.0f, 20.0f, 1.0f));

        auto sphereConfig = AZStd::make_shared<Physics::SphereShapeConfiguration>();
        sphereConfig->m_radius = 0.5f;

        AzPhysics::ShapeCastRequest request;
        request.m_start = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 3.0f));
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 10.0f;
        request.m_shapeConfiguration = sphereConfig;

        AzPhysics::SceneQueryHits hits = m_scene->QueryScene(&request);

        // Sphere surface touches the slab top (z=0) when the sphere center is at z=0.5.
        ASSERT_EQ(hits.m_hits.size(), 1u);
        EXPECT_NEAR(hits.m_hits[0].m_distance, 2.5f, 0.05f);
        EXPECT_NEAR(hits.m_hits[0].m_position.GetZ(), 0.0f, 0.05f);
    }

    TEST_F(JoltSceneQueryTests, OverlapSphereReportsBodiesInsideOnly)
    {
        auto inside = CreateStaticBox(AZ::Vector3(1.0f, 0.0f, 0.0f), AZ::Vector3(1.0f, 1.0f, 1.0f));
        CreateStaticBox(AZ::Vector3(50.0f, 0.0f, 0.0f), AZ::Vector3(1.0f, 1.0f, 1.0f));

        auto sphereConfig = AZStd::make_shared<Physics::SphereShapeConfiguration>();
        sphereConfig->m_radius = 3.0f;

        AzPhysics::OverlapRequest request;
        request.m_pose = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 0.0f));
        request.m_shapeConfiguration = sphereConfig;

        AzPhysics::SceneQueryHits hits = m_scene->QueryScene(&request);

        ASSERT_EQ(hits.m_hits.size(), 1u);
        EXPECT_EQ(hits.m_hits[0].m_bodyHandle, inside);
    }

} // namespace JoltPhysics
