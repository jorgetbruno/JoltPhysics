#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <System/JoltSystem.h>
#include <Scene/JoltScene.h>
#include <Configuration/JoltSettingsRegistryManager.h>
#include <Shape/JoltCylinderShapeConfiguration.h>
#include <Shape/JoltMeshUtils.h>
#include <Shape/JoltShapeUtils.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Physics/SimulatedBodies/StaticRigidBody.h>

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

    TEST_F(JoltSceneTests, DisableSimulationFreezesRigidBodyAndEnableResumes)
    {
        auto sphereShape = AZStd::make_shared<Physics::SphereShapeConfiguration>(0.5f);
        AzPhysics::RigidBodyConfiguration sphereConfig;
        sphereConfig.m_position = AZ::Vector3(0.0f, 0.0f, 5.0f);
        sphereConfig.m_colliderAndShapeData =
            AzPhysics::ShapeColliderPair(AZStd::make_shared<Physics::ColliderConfiguration>(), sphereShape);
        auto handle = m_scene->AddSimulatedBody(&sphereConfig);
        ASSERT_NE(handle, AzPhysics::InvalidSimulatedBodyHandle);

        AzPhysics::SimulatedBody* body = m_scene->GetSimulatedBodyFromHandle(handle);
        ASSERT_NE(body, nullptr);
        EXPECT_TRUE(body->m_simulating);

        int enabledSignals = 0;
        int disabledSignals = 0;
        AzPhysics::SceneEvents::OnSimulationBodySimulationEnabled::Handler enabledHandler(
            [&enabledSignals](AzPhysics::SceneHandle, AzPhysics::SimulatedBodyHandle)
            {
                ++enabledSignals;
            });
        AzPhysics::SceneEvents::OnSimulationBodySimulationDisabled::Handler disabledHandler(
            [&disabledSignals](AzPhysics::SceneHandle, AzPhysics::SimulatedBodyHandle)
            {
                ++disabledSignals;
            });
        m_scene->RegisterSimulationBodySimulationEnabledHandler(enabledHandler);
        m_scene->RegisterSimulationBodySimulationDisabledHandler(disabledHandler);

        const float fixedDeltaTime = 1.0f / 60.0f;
        auto simulateSeconds = [&](float seconds)
        {
            for (int i = 0; i < static_cast<int>(seconds / fixedDeltaTime); ++i)
            {
                m_scene->StartSimulation(fixedDeltaTime);
                m_scene->FinishSimulation();
            }
        };

        // Disabled: gravity no longer moves the body.
        m_scene->DisableSimulationOfBody(handle);
        EXPECT_FALSE(body->m_simulating);
        EXPECT_EQ(disabledSignals, 1);
        simulateSeconds(0.5f);
        EXPECT_NEAR(body->GetPosition().GetZ(), 5.0f, 0.001f);

        // Disabling again is a no-op (no extra event).
        m_scene->DisableSimulationOfBody(handle);
        EXPECT_EQ(disabledSignals, 1);

        // Re-enabled: the body falls again.
        m_scene->EnableSimulationOfBody(handle);
        EXPECT_TRUE(body->m_simulating);
        EXPECT_EQ(enabledSignals, 1);
        simulateSeconds(0.5f);
        EXPECT_LT(body->GetPosition().GetZ(), 4.5f);
    }

    TEST_F(JoltSceneTests, DisabledStaticBodyIgnoredByQueriesUntilReEnabled)
    {
        auto slabShape = AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(10.0f, 10.0f, 1.0f));
        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        slabConfig.m_colliderAndShapeData =
            AzPhysics::ShapeColliderPair(AZStd::make_shared<Physics::ColliderConfiguration>(), slabShape);
        auto handle = m_scene->AddSimulatedBody(&slabConfig);
        ASSERT_NE(handle, AzPhysics::InvalidSimulatedBodyHandle);

        AzPhysics::RayCastRequest request;
        request.m_start = AZ::Vector3(0.0f, 0.0f, 5.0f);
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 100.0f;

        EXPECT_EQ(m_scene->QueryScene(&request).m_hits.size(), 1u);

        // A disabled body is out of the world: queries no longer see it.
        m_scene->DisableSimulationOfBody(handle);
        EXPECT_TRUE(m_scene->QueryScene(&request).m_hits.empty());

        m_scene->EnableSimulationOfBody(handle);
        EXPECT_EQ(m_scene->QueryScene(&request).m_hits.size(), 1u);
    }

    TEST_F(JoltSceneTests, ShapeAttachedToStaticBodyIsCollidableAndQueryable)
    {
        // Slab spanning x in [-5, 5] with its top at z=0.
        auto slabCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        slabCollider->m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        auto slabShape = AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(10.0f, 10.0f, 1.0f));
        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(slabCollider, slabShape);
        auto slabHandle = m_scene->AddSimulatedBody(&slabConfig);

        auto* slabBody = azdynamic_cast<AzPhysics::StaticRigidBody*>(m_scene->GetSimulatedBodyFromHandle(slabHandle));
        ASSERT_NE(slabBody, nullptr);

        // A wall 8m out along x is beyond the slab: nothing there yet.
        AzPhysics::RayCastRequest request;
        request.m_start = AZ::Vector3(8.0f, 0.0f, 5.0f);
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 100.0f;
        EXPECT_TRUE(m_scene->QueryScene(&request).m_hits.empty());

        Physics::ColliderConfiguration attachedCollider;
        attachedCollider.m_position = AZ::Vector3(8.0f, 0.0f, -0.5f);
        Physics::BoxShapeConfiguration attachedShapeConfig(AZ::Vector3(2.0f, 2.0f, 1.0f));
        AZStd::shared_ptr<Physics::Shape> attachedShape =
            JoltShapeUtils::CreateShape(attachedCollider, attachedShapeConfig);
        ASSERT_NE(attachedShape, nullptr);

        slabBody->AddShape(attachedShape);

        // The attached geometry is picked up by scene queries...
        auto hits = m_scene->QueryScene(&request);
        ASSERT_EQ(hits.m_hits.size(), 1u);
        EXPECT_NEAR(hits.m_hits[0].m_position.GetZ(), 0.0f, 0.01f);

        // ...and collides: a sphere dropped above it comes to rest on top.
        auto sphereShape = AZStd::make_shared<Physics::SphereShapeConfiguration>(0.5f);
        AzPhysics::RigidBodyConfiguration sphereConfig;
        sphereConfig.m_position = AZ::Vector3(8.0f, 0.0f, 4.0f);
        sphereConfig.m_colliderAndShapeData =
            AzPhysics::ShapeColliderPair(AZStd::make_shared<Physics::ColliderConfiguration>(), sphereShape);
        auto sphereHandle = m_scene->AddSimulatedBody(&sphereConfig);

        const float fixedDeltaTime = 1.0f / 60.0f;
        for (int i = 0; i < 300; ++i)
        {
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();
        }

        EXPECT_NEAR(m_scene->GetSimulatedBodyFromHandle(sphereHandle)->GetPosition().GetZ(), 0.5f, 0.05f);
    }

    TEST_F(JoltSceneTests, CylinderColliderIsNativeAndZAligned)
    {
        // Jolt cylinders are Y-aligned; O3DE (like its capsules) is Z-aligned, so the
        // shape is wrapped in a rotation. Verify through the bounds: a 2 m tall,
        // 0.5 m radius cylinder spans 1.0 x 1.0 x 2.0.
        JoltCylinderShapeConfiguration cylinderConfig(2.0f, 0.5f);
        JPH::RefConst<JPH::Shape> shape = JoltShapeUtils::CreateJoltShapeFromConfig(cylinderConfig);
        ASSERT_NE(shape, nullptr);

        const JPH::AABox bounds = shape->GetLocalBounds();
        EXPECT_NEAR(bounds.GetExtent().GetX(), 0.5f, 0.01f);
        EXPECT_NEAR(bounds.GetExtent().GetY(), 0.5f, 0.01f);
        EXPECT_NEAR(bounds.GetExtent().GetZ(), 1.0f, 0.01f);
    }

    TEST_F(JoltSceneTests, CylinderRestsOnItsFlatFaceUnlikeACapsule)
    {
        // Static ground with its top at z=0.
        auto groundCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        groundCollider->m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        auto groundShape = AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(20.0f, 20.0f, 1.0f));
        AzPhysics::StaticRigidBodyConfiguration groundConfig;
        groundConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(groundCollider, groundShape);
        m_scene->AddSimulatedBody(&groundConfig);

        // A 1 m tall, 0.4 m radius cylinder dropped flat.
        auto cylinderShape = AZStd::make_shared<JoltCylinderShapeConfiguration>(1.0f, 0.4f);
        AzPhysics::RigidBodyConfiguration cylinderConfig;
        cylinderConfig.m_position = AZ::Vector3(0.0f, 0.0f, 3.0f);
        cylinderConfig.m_colliderAndShapeData =
            AzPhysics::ShapeColliderPair(AZStd::make_shared<Physics::ColliderConfiguration>(), cylinderShape);
        auto handle = m_scene->AddSimulatedBody(&cylinderConfig);
        ASSERT_NE(handle, AzPhysics::InvalidSimulatedBodyHandle);

        const float fixedDeltaTime = 1.0f / 60.0f;
        for (int i = 0; i < 300; ++i)
        {
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();
        }

        // It settles on its flat end at half its height. A capsule of the same
        // dimensions would rest on its rounded cap, and a sphere fallback (the capsule
        // degenerate path) would sit at its radius instead.
        auto* body = m_scene->GetSimulatedBodyFromHandle(handle);
        EXPECT_NEAR(body->GetPosition().GetZ(), 0.5f, 0.05f);
    }

    TEST_F(JoltSceneTests, DegenerateCylinderIsClampedInsteadOfFailing)
    {
        // Zero height/radius would be rejected by Jolt; the collider is clamped to a
        // sliver so the body still builds (with a warning).
        JoltCylinderShapeConfiguration degenerate(0.0f, 0.0f);
        JPH::RefConst<JPH::Shape> shape = JoltShapeUtils::CreateJoltShapeFromConfig(degenerate);
        EXPECT_NE(shape, nullptr);
    }

    TEST_F(JoltSceneTests, StaticBodyPerBodyRayCastHitsAndMisses)
    {
        auto slabCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        slabCollider->m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        auto slabShape = AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(4.0f, 4.0f, 1.0f));
        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_entityId = AZ::EntityId(4321);
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(slabCollider, slabShape);
        auto handle = m_scene->AddSimulatedBody(&slabConfig);

        auto* body = azdynamic_cast<AzPhysics::StaticRigidBody*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(body, nullptr);

        // The 4x4x1 slab is offset -0.5 in z, so its top surface is at z=0.
        AzPhysics::RayCastRequest request;
        request.m_start = AZ::Vector3(0.0f, 0.0f, 5.0f);
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 20.0f;

        AzPhysics::SceneQueryHit hit = body->RayCast(request);
        EXPECT_EQ(hit.m_bodyHandle, handle);
        EXPECT_EQ(hit.m_entityId, AZ::EntityId(4321));
        EXPECT_NEAR(hit.m_position.GetZ(), 0.0f, 0.01f);
        EXPECT_NEAR(hit.m_distance, 5.0f, 0.01f);
        EXPECT_NEAR(hit.m_normal.GetZ(), 1.0f, 0.01f);

        // A ray beyond the slab misses.
        request.m_start = AZ::Vector3(10.0f, 0.0f, 5.0f);
        EXPECT_EQ(body->RayCast(request).m_bodyHandle, AzPhysics::InvalidSimulatedBodyHandle);
    }

} // namespace JoltPhysics

namespace JoltPhysics
{
    TEST(JoltMeshRoundtrip, PackAndReadBackProducesShape)
    {
        auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
        auto system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));
        JoltSystemConfiguration systemConfig;
        system->Initialize(&systemConfig);

        const AZ::Vector3 vertices[] = {
            AZ::Vector3(0.0f, 0.0f, 0.0f),
            AZ::Vector3(1.0f, 0.0f, 0.0f),
            AZ::Vector3(0.0f, 1.0f, 0.0f),
        };
        const AZ::u32 indices[] = { 0, 1, 2 };

        AZStd::vector<AZ::u8> blob = JoltMeshUtils::PackTriangleMesh(vertices, 3, indices, 3);
        ASSERT_FALSE(blob.empty());
        JPH::RefConst<JPH::Shape> shape = JoltMeshUtils::CreateMeshShapeFromCookedData(blob);
        EXPECT_NE(shape, nullptr);

        system->Shutdown();
    }

    TEST(JoltCapsuleShape, UnderHeightCapsuleDegradesToSphereWithoutError)
    {
        auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
        auto system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));
        JoltSystemConfiguration systemConfig;
        system->Initialize(&systemConfig);

        // Height (0.5) < 2*radius (0.6): an invalid capsule. Must not error; it degrades
        // to a valid sphere shape instead of producing a negative-half-height capsule.
        Physics::CapsuleShapeConfiguration capsule(0.5f, 0.3f);
        JPH::RefConst<JPH::Shape> shape = JoltShapeUtils::CreateJoltShapeFromConfig(capsule);
        ASSERT_NE(shape, nullptr);
        EXPECT_EQ(shape->GetType(), JPH::EShapeType::Convex);

        // A normal capsule (height 1.8 > 2*0.3) still builds fine.
        Physics::CapsuleShapeConfiguration validCapsule(1.8f, 0.3f);
        EXPECT_NE(JoltShapeUtils::CreateJoltShapeFromConfig(validCapsule), nullptr);

        system->Shutdown();
    }

    TEST(JoltConvexRoundtrip, PackAndReadBackProducesShape)
    {
        auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
        auto system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));
        JoltSystemConfiguration systemConfig;
        system->Initialize(&systemConfig);

        // The eight corners of a unit cube (a convex point cloud; no indices needed).
        const AZ::Vector3 vertices[] = {
            AZ::Vector3(-0.5f, -0.5f, -0.5f), AZ::Vector3(0.5f, -0.5f, -0.5f),
            AZ::Vector3(-0.5f, 0.5f, -0.5f),  AZ::Vector3(0.5f, 0.5f, -0.5f),
            AZ::Vector3(-0.5f, -0.5f, 0.5f),  AZ::Vector3(0.5f, -0.5f, 0.5f),
            AZ::Vector3(-0.5f, 0.5f, 0.5f),   AZ::Vector3(0.5f, 0.5f, 0.5f),
        };

        AZStd::vector<AZ::u8> blob = JoltMeshUtils::PackConvexMesh(vertices, 8);
        ASSERT_FALSE(blob.empty());
        JPH::RefConst<JPH::Shape> shape = JoltMeshUtils::CreateConvexShapeFromCookedData(blob);
        ASSERT_NE(shape, nullptr);
        EXPECT_EQ(shape->GetType(), JPH::EShapeType::Convex);

        system->Shutdown();
    }
} // namespace JoltPhysics
