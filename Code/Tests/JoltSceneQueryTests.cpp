#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <System/JoltSystem.h>
#include <Scene/JoltScene.h>
#include <Shape/JoltMeshUtils.h>
#include <Configuration/JoltSettingsRegistryManager.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/SimulatedBodies/StaticRigidBody.h>
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

        //! A static body with two box colliders side by side on X, each 1 m wide, centred
        //! at -1.5 and +1.5 in the body's local frame. Two colliders is what makes the
        //! sub-shape mapping observable: with one, any mapping bug still returns collider 0.
        AzPhysics::SimulatedBodyHandle CreateTwoColliderBody(const AZ::Vector3& position)
        {
            AzPhysics::ShapeColliderPairList colliders;
            for (float offsetX : { -1.5f, 1.5f })
            {
                auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
                colliderConfig->m_position = AZ::Vector3(offsetX, 0.0f, 0.0f);
                auto shapeConfig = AZStd::make_shared<Physics::BoxShapeConfiguration>();
                shapeConfig->m_dimensions = AZ::Vector3(1.0f, 1.0f, 1.0f);
                colliders.emplace_back(colliderConfig, shapeConfig);
            }

            AzPhysics::StaticRigidBodyConfiguration staticConfig;
            staticConfig.m_position = position;
            staticConfig.m_entityId = AZ::EntityId(4242);
            staticConfig.m_colliderAndShapeData = colliders;
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

    TEST_F(JoltSceneQueryTests, RaycastHitCarriesTheShapeItHit)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(20.0f, 20.0f, 1.0f));

        auto request = CreateRayDown(AZ::Vector3(0.0f, 0.0f, 5.0f));
        AzPhysics::SceneQueryHits hits = m_scene->QueryScene(&request);

        ASSERT_EQ(hits.m_hits.size(), 1u);
        const AzPhysics::SceneQueryHit& hit = hits.m_hits[0];
        ASSERT_NE(hit.m_shape, nullptr);
        EXPECT_TRUE(
            AzPhysics::SceneQuery::ResultFlags::Shape == (hit.m_resultFlags & AzPhysics::SceneQuery::ResultFlags::Shape));

        // The hit shape is the body's own collider object, not a copy built for the hit.
        auto* body = azdynamic_cast<AzPhysics::StaticRigidBody*>(
            m_scene->GetSimulatedBodyFromHandle(hit.m_bodyHandle));
        ASSERT_NE(body, nullptr);
        ASSERT_EQ(body->GetShapeCount(), 1u);
        EXPECT_EQ(hit.m_shape, body->GetShape(0).get());
    }

    TEST_F(JoltSceneQueryTests, RaycastHitNamesWhichColliderOfACompoundWasHit)
    {
        CreateTwoColliderBody(AZ::Vector3(0.0f, 0.0f, 0.0f));

        auto* body = azdynamic_cast<AzPhysics::StaticRigidBody*>(
            m_scene->GetSimulatedBodyFromHandle(CreateTwoColliderBody(AZ::Vector3(0.0f, 0.0f, 20.0f))));
        ASSERT_NE(body, nullptr);
        ASSERT_EQ(body->GetShapeCount(), 2u);

        // Straight down onto the left collider, then the right one. Distinguishing them is
        // the whole point: a mapping that always answered 0 would fail the second case.
        auto leftRequest = CreateRayDown(AZ::Vector3(-1.5f, 0.0f, 25.0f), 10.0f);
        AzPhysics::SceneQueryHits leftHits = m_scene->QueryScene(&leftRequest);
        ASSERT_EQ(leftHits.m_hits.size(), 1u);
        EXPECT_EQ(leftHits.m_hits[0].m_shape, body->GetShape(0).get());

        auto rightRequest = CreateRayDown(AZ::Vector3(1.5f, 0.0f, 25.0f), 10.0f);
        AzPhysics::SceneQueryHits rightHits = m_scene->QueryScene(&rightRequest);
        ASSERT_EQ(rightHits.m_hits.size(), 1u);
        EXPECT_EQ(rightHits.m_hits[0].m_shape, body->GetShape(1).get());

        EXPECT_NE(leftHits.m_hits[0].m_shape, rightHits.m_hits[0].m_shape);
    }

    TEST_F(JoltSceneQueryTests, RaycastHitOnAHullGroupNamesTheOneColliderTheHullsBelongTo)
    {
        // A single collider can still be a compound: a mesh collider baked as a convex
        // hull group (hull per mesh node, or a decomposition) stores its hulls as compound
        // children. Those children are hulls, not colliders, so a hit on the second one
        // still belongs to collider 0 - reading it as a collider index used to leave the
        // hit with no shape at all.
        auto makeCube = [](float x)
        {
            AZStd::vector<AZ::Vector3> points;
            for (const float dx : { -0.5f, 0.5f })
            {
                for (const float dy : { -0.5f, 0.5f })
                {
                    for (const float dz : { -0.5f, 0.5f })
                    {
                        points.push_back(AZ::Vector3(x + dx, dy, dz));
                    }
                }
            }
            return points;
        };

        auto cookedConfig = AZStd::make_shared<Physics::CookedMeshShapeConfiguration>();
        const AZStd::vector<AZ::u8> blob = JoltMeshUtils::PackConvexHulls({ makeCube(-3.0f), makeCube(3.0f) });
        cookedConfig->SetCookedMeshData(
            blob.data(), blob.size(), Physics::CookedMeshShapeConfiguration::MeshType::Convex);

        AzPhysics::StaticRigidBodyConfiguration staticConfig;
        staticConfig.m_position = AZ::Vector3::CreateZero();
        staticConfig.m_colliderAndShapeData =
            AzPhysics::ShapeColliderPair(AZStd::make_shared<Physics::ColliderConfiguration>(), cookedConfig);

        auto* body = azdynamic_cast<AzPhysics::StaticRigidBody*>(
            m_scene->GetSimulatedBodyFromHandle(m_scene->AddSimulatedBody(&staticConfig)));
        ASSERT_NE(body, nullptr);
        ASSERT_EQ(body->GetShapeCount(), 1u);

        auto firstHullRequest = CreateRayDown(AZ::Vector3(-3.0f, 0.0f, 5.0f));
        AzPhysics::SceneQueryHits firstHullHits = m_scene->QueryScene(&firstHullRequest);
        ASSERT_EQ(firstHullHits.m_hits.size(), 1u);
        EXPECT_EQ(firstHullHits.m_hits[0].m_shape, body->GetShape(0).get());

        auto secondHullRequest = CreateRayDown(AZ::Vector3(3.0f, 0.0f, 5.0f));
        AzPhysics::SceneQueryHits secondHullHits = m_scene->QueryScene(&secondHullRequest);
        ASSERT_EQ(secondHullHits.m_hits.size(), 1u);
        EXPECT_EQ(secondHullHits.m_hits[0].m_shape, body->GetShape(0).get());

        // Release the native mesh cached on the configuration (in production this is
        // balanced by JoltPhysicsSystemComponent::ReleaseNativeMeshObject).
        if (auto* cachedMesh = static_cast<JPH::Shape*>(cookedConfig->GetCachedNativeMesh()))
        {
            cachedMesh->Release();
            cookedConfig->SetCachedNativeMesh(nullptr);
        }
    }

    TEST_F(JoltSceneQueryTests, FilterCallbackReceivesTheHitShape)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(20.0f, 20.0f, 1.0f));

        Physics::Shape* shapeSeenByCallback = nullptr;
        auto request = CreateRayDown(AZ::Vector3(0.0f, 0.0f, 5.0f));
        request.m_filterCallback = [&shapeSeenByCallback](
            const AzPhysics::SimulatedBody* body, const Physics::Shape* shape)
        {
            EXPECT_NE(body, nullptr);
            shapeSeenByCallback = const_cast<Physics::Shape*>(shape);
            return AzPhysics::SceneQuery::QueryHitType::Touch;
        };

        AzPhysics::SceneQueryHits hits = m_scene->QueryScene(&request);

        ASSERT_EQ(hits.m_hits.size(), 1u);
        // Used to be nullptr unconditionally, so a callback could filter by body but never
        // by which collider was hit.
        ASSERT_NE(shapeSeenByCallback, nullptr);
        EXPECT_EQ(shapeSeenByCallback, hits.m_hits[0].m_shape);
    }

    TEST_F(JoltSceneQueryTests, OverlapHitCarriesTheShapeItHit)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 0.0f), AZ::Vector3(2.0f, 2.0f, 2.0f));

        AzPhysics::OverlapRequest request;
        request.m_pose = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 0.0f));
        auto sphere = AZStd::make_shared<Physics::SphereShapeConfiguration>();
        sphere->m_radius = 1.0f;
        request.m_shapeConfiguration = sphere;

        AzPhysics::SceneQueryHits hits = m_scene->QueryScene(&request);

        ASSERT_GE(hits.m_hits.size(), 1u);
        EXPECT_NE(hits.m_hits[0].m_shape, nullptr);
    }

    TEST_F(JoltSceneQueryTests, AsyncRaycastDeliversHitsOnSimulationFinish)
    {
        auto boxHandle = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 0.0f), AZ::Vector3(2.0f, 2.0f, 2.0f));

        int callbackCount = 0;
        AzPhysics::SceneQuery::AsyncRequestId reportedId = 0;
        AzPhysics::SceneQueryHits reportedHits;

        AzPhysics::RayCastRequest request = CreateRayDown(AZ::Vector3(0.0f, 0.0f, 10.0f));
        const bool queued = m_scene->QuerySceneAsync(
            42, &request,
            [&](AzPhysics::SceneQuery::AsyncRequestId requestId, AzPhysics::SceneQueryHits hits)
            {
                ++callbackCount;
                reportedId = requestId;
                reportedHits = AZStd::move(hits);
            });
        EXPECT_TRUE(queued);

        // Non-blocking: nothing has run yet.
        EXPECT_EQ(callbackCount, 0);

        // The request is copied on queueing, so the caller's request going out of scope
        // (or being reused) cannot affect the queued query.
        request.m_start = AZ::Vector3(100.0f, 100.0f, 100.0f);

        m_scene->StartSimulation(1.0f / 60.0f);
        m_scene->FinishSimulation();

        EXPECT_EQ(callbackCount, 1);
        EXPECT_EQ(reportedId, 42);
        ASSERT_EQ(reportedHits.m_hits.size(), 1u);
        EXPECT_EQ(reportedHits.m_hits[0].m_bodyHandle, boxHandle);
        EXPECT_NEAR(reportedHits.m_hits[0].m_position.GetZ(), 1.0f, 0.05f);

        // The queue is drained: a further step does not re-deliver.
        m_scene->StartSimulation(1.0f / 60.0f);
        m_scene->FinishSimulation();
        EXPECT_EQ(callbackCount, 1);
    }

    TEST_F(JoltSceneQueryTests, AsyncBatchDeliversAllResultsInOrder)
    {
        auto nearBox = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 0.0f), AZ::Vector3(2.0f, 2.0f, 2.0f));
        auto farBox = CreateStaticBox(AZ::Vector3(10.0f, 0.0f, 0.0f), AZ::Vector3(2.0f, 2.0f, 2.0f));

        AzPhysics::SceneQueryRequests requests;
        requests.push_back(AZStd::make_shared<AzPhysics::RayCastRequest>(CreateRayDown(AZ::Vector3(0.0f, 0.0f, 10.0f))));
        requests.push_back(AZStd::make_shared<AzPhysics::RayCastRequest>(CreateRayDown(AZ::Vector3(10.0f, 0.0f, 10.0f))));
        // A ray into empty space still produces an (empty) entry, keeping the results
        // aligned with the requests.
        requests.push_back(AZStd::make_shared<AzPhysics::RayCastRequest>(CreateRayDown(AZ::Vector3(50.0f, 0.0f, 10.0f))));

        int callbackCount = 0;
        AzPhysics::SceneQueryHitsList reportedHits;
        const bool queued = m_scene->QuerySceneAsyncBatch(
            7, requests,
            [&](AzPhysics::SceneQuery::AsyncRequestId, AzPhysics::SceneQueryHitsList hits)
            {
                ++callbackCount;
                reportedHits = AZStd::move(hits);
            });
        EXPECT_TRUE(queued);
        EXPECT_EQ(callbackCount, 0);

        m_scene->StartSimulation(1.0f / 60.0f);
        m_scene->FinishSimulation();

        EXPECT_EQ(callbackCount, 1);
        ASSERT_EQ(reportedHits.size(), 3u);
        ASSERT_EQ(reportedHits[0].m_hits.size(), 1u);
        EXPECT_EQ(reportedHits[0].m_hits[0].m_bodyHandle, nearBox);
        ASSERT_EQ(reportedHits[1].m_hits.size(), 1u);
        EXPECT_EQ(reportedHits[1].m_hits[0].m_bodyHandle, farBox);
        EXPECT_TRUE(reportedHits[2].m_hits.empty());
    }

    TEST_F(JoltSceneQueryTests, AsyncQueryRejectsNullRequestAndEmptyBatch)
    {
        auto callback = [](AzPhysics::SceneQuery::AsyncRequestId, AzPhysics::SceneQueryHits) {};
        EXPECT_FALSE(m_scene->QuerySceneAsync(1, nullptr, callback));

        AzPhysics::RayCastRequest request = CreateRayDown(AZ::Vector3::CreateZero());
        EXPECT_FALSE(m_scene->QuerySceneAsync(1, &request, nullptr));

        auto batchCallback = [](AzPhysics::SceneQuery::AsyncRequestId, AzPhysics::SceneQueryHitsList) {};
        EXPECT_FALSE(m_scene->QuerySceneAsyncBatch(1, {}, batchCallback));
    }

} // namespace JoltPhysics
