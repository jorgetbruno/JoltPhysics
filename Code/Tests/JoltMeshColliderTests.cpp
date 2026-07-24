#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/Math/Matrix4x4.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <Configuration/JoltSettingsRegistryManager.h>
#include <Scene/JoltScene.h>
#include <Shape/JoltMeshUtils.h>
#include <System/JoltSystem.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Visibility/VisibleGeometryBus.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace JoltPhysics
{
    class JoltMeshColliderTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "MeshColliderTestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);
        }

        void TearDown() override
        {
            m_system->RemoveScene(m_sceneHandle);
            m_system->Shutdown();
            m_system.reset();
        }

        //! A quad of two triangles in the local XY plane at z=0, halfExtent on a side.
        static AzFramework::VisibleGeometry MakeQuadGeometry(float halfExtent, const AZ::Matrix4x4& transform)
        {
            AzFramework::VisibleGeometry geometry;
            geometry.m_transform = transform;
            const float corners[4][2] = {
                { -halfExtent, -halfExtent }, { halfExtent, -halfExtent },
                { halfExtent, halfExtent }, { -halfExtent, halfExtent }
            };
            for (const auto& corner : corners)
            {
                geometry.m_vertices.push_back(corner[0]);
                geometry.m_vertices.push_back(corner[1]);
                geometry.m_vertices.push_back(0.0f);
            }
            geometry.m_indices = { 0, 1, 2, 0, 2, 3 };
            return geometry;
        }

        //! The eight corners of a cube plus a couple of faces (the convex cooker only
        //! needs the point cloud, but the cooker requires a valid index list).
        static AzFramework::VisibleGeometry MakeCubeGeometry(float halfExtent)
        {
            AzFramework::VisibleGeometry geometry;
            geometry.m_transform = AZ::Matrix4x4::CreateIdentity();
            for (int i = 0; i < 8; ++i)
            {
                geometry.m_vertices.push_back((i & 1) ? halfExtent : -halfExtent);
                geometry.m_vertices.push_back((i & 2) ? halfExtent : -halfExtent);
                geometry.m_vertices.push_back((i & 4) ? halfExtent : -halfExtent);
            }
            geometry.m_indices = { 0, 1, 3, 0, 3, 2, 4, 5, 7, 4, 7, 6 };
            return geometry;
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle;
        AzPhysics::Scene* m_scene = nullptr;
    };

    TEST_F(JoltMeshColliderTests, CookTransformsWorldGeometryIntoEntityLocalSpace)
    {
        // Geometry lives at world (1, 2, 3); the entity is at the same spot, so the
        // cooked vertices must come out centered on the entity's local origin.
        AzFramework::VisibleGeometryContainer container;
        container.push_back(MakeQuadGeometry(2.0f, AZ::Matrix4x4::CreateTranslation(AZ::Vector3(1.0f, 2.0f, 3.0f))));

        Physics::CookedMeshShapeConfiguration cookedConfig;
        ASSERT_TRUE(JoltMeshUtils::CookVisibleGeometry(
            container, AZ::Transform::CreateTranslation(AZ::Vector3(1.0f, 2.0f, 3.0f)),
            Physics::CookedMeshShapeConfiguration::MeshType::TriangleMesh, cookedConfig));
        EXPECT_EQ(cookedConfig.GetMeshType(), Physics::CookedMeshShapeConfiguration::MeshType::TriangleMesh);

        JPH::RefConst<JPH::Shape> shape = JoltMeshUtils::CreateMeshShapeFromCookedData(cookedConfig.GetCookedMeshData());
        ASSERT_NE(shape, nullptr);

        const JPH::AABox bounds = shape->GetLocalBounds();
        EXPECT_NEAR(bounds.GetCenter().GetX(), 0.0f, 0.01f);
        EXPECT_NEAR(bounds.GetCenter().GetY(), 0.0f, 0.01f);
        EXPECT_NEAR(bounds.GetCenter().GetZ(), 0.0f, 0.01f);
        EXPECT_NEAR(bounds.GetExtent().GetX(), 2.0f, 0.01f);
        EXPECT_NEAR(bounds.GetExtent().GetY(), 2.0f, 0.01f);
    }

    TEST_F(JoltMeshColliderTests, CookedTriangleMeshSupportsRestingSphere)
    {
        AzFramework::VisibleGeometryContainer container;
        container.push_back(MakeQuadGeometry(10.0f, AZ::Matrix4x4::CreateIdentity()));

        auto cookedConfig = AZStd::make_shared<Physics::CookedMeshShapeConfiguration>();
        ASSERT_TRUE(JoltMeshUtils::CookVisibleGeometry(
            container, AZ::Transform::CreateIdentity(),
            Physics::CookedMeshShapeConfiguration::MeshType::TriangleMesh, *cookedConfig));

        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_colliderAndShapeData =
            AzPhysics::ShapeColliderPair(AZStd::make_shared<Physics::ColliderConfiguration>(), cookedConfig);
        ASSERT_NE(m_scene->AddSimulatedBody(&slabConfig), AzPhysics::InvalidSimulatedBodyHandle);

        auto sphereShape = AZStd::make_shared<Physics::SphereShapeConfiguration>(0.5f);
        AzPhysics::RigidBodyConfiguration sphereConfig;
        sphereConfig.m_position = AZ::Vector3(0.0f, 0.0f, 3.0f);
        sphereConfig.m_colliderAndShapeData =
            AzPhysics::ShapeColliderPair(AZStd::make_shared<Physics::ColliderConfiguration>(), sphereShape);
        auto sphereHandle = m_scene->AddSimulatedBody(&sphereConfig);

        const float fixedDeltaTime = 1.0f / 60.0f;
        for (int i = 0; i < 300; ++i)
        {
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();
        }

        // The sphere rests on the cooked quad instead of falling through it.
        const float sphereZ = m_scene->GetSimulatedBodyFromHandle(sphereHandle)->GetPosition().GetZ();
        EXPECT_NEAR(sphereZ, 0.5f, 0.05f);

        // Release the native mesh cached on the configuration (in production this is
        // balanced by JoltPhysicsSystemComponent::ReleaseNativeMeshObject, which the
        // test environment does not run).
        if (auto* cachedMesh = static_cast<JPH::Shape*>(cookedConfig->GetCachedNativeMesh()))
        {
            cachedMesh->Release();
            cookedConfig->SetCachedNativeMesh(nullptr);
        }
    }

    TEST_F(JoltMeshColliderTests, CookConvexProducesHullShape)
    {
        AzFramework::VisibleGeometryContainer container;
        container.push_back(MakeCubeGeometry(0.5f));

        Physics::CookedMeshShapeConfiguration cookedConfig;
        ASSERT_TRUE(JoltMeshUtils::CookVisibleGeometry(
            container, AZ::Transform::CreateIdentity(),
            Physics::CookedMeshShapeConfiguration::MeshType::Convex, cookedConfig));
        EXPECT_EQ(cookedConfig.GetMeshType(), Physics::CookedMeshShapeConfiguration::MeshType::Convex);

        JPH::RefConst<JPH::Shape> shape = JoltMeshUtils::CreateConvexShapeFromCookedData(cookedConfig.GetCookedMeshData());
        ASSERT_NE(shape, nullptr);
        EXPECT_EQ(shape->GetType(), JPH::EShapeType::Convex);

        const JPH::AABox bounds = shape->GetLocalBounds();
        // The hull wraps the unit cube (a small convex radius may pad the bounds).
        EXPECT_NEAR(bounds.GetExtent().GetX(), 0.5f, 0.1f);
        EXPECT_NEAR(bounds.GetExtent().GetZ(), 0.5f, 0.1f);
    }

    TEST_F(JoltMeshColliderTests, CookRejectsEmptyOrDegenerateGeometry)
    {
        Physics::CookedMeshShapeConfiguration cookedConfig;

        AzFramework::VisibleGeometryContainer empty;
        EXPECT_FALSE(JoltMeshUtils::CookVisibleGeometry(
            empty, AZ::Transform::CreateIdentity(),
            Physics::CookedMeshShapeConfiguration::MeshType::TriangleMesh, cookedConfig));

        // An entry without indices contributes nothing.
        AzFramework::VisibleGeometry noIndices;
        noIndices.m_transform = AZ::Matrix4x4::CreateIdentity();
        noIndices.m_vertices = { 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f };
        AzFramework::VisibleGeometryContainer degenerate{ noIndices };
        EXPECT_FALSE(JoltMeshUtils::CookVisibleGeometry(
            degenerate, AZ::Transform::CreateIdentity(),
            Physics::CookedMeshShapeConfiguration::MeshType::TriangleMesh, cookedConfig));
    }

} // namespace JoltPhysics
