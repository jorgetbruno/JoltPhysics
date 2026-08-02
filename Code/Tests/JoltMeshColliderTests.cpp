#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <JoltTestWarningCatcher.h>
#include <AzCore/Math/Matrix4x4.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <Configuration/JoltSettingsRegistryManager.h>
#include <Scene/JoltScene.h>
#include <JoltPhysics/JoltModuleGlobals.h>
#include <Shape/JoltMeshUtils.h>
#include <Shape/JoltShape.h>
#include <Shape/JoltShapeUtils.h>
#include <System/JoltSystem.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Visibility/VisibleGeometryBus.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace JoltPhysics
{
    namespace
    {
        // First-hit ray collector for checking which triangle a ray lands on.
        struct FirstHitCollector : public JPH::CastRayCollector
        {
            void AddHit(const JPH::RayCastResult& result) override
            {
                if (!m_hit)
                {
                    m_hit = true;
                    m_subShapeId = result.mSubShapeID2;
                }
            }

            bool m_hit = false;
            JPH::SubShapeID m_subShapeId;
        };
    } // namespace

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

        //! The eight corners of a box plus a couple of faces (the convex cooker only
        //! needs the point cloud, but the cooker requires a valid index list).
        static AzFramework::VisibleGeometry MakeBoxGeometry(
            float halfX, float halfY, float halfZ,
            const AZ::Matrix4x4& transform = AZ::Matrix4x4::CreateIdentity())
        {
            AzFramework::VisibleGeometry geometry;
            geometry.m_transform = transform;
            const float halves[3] = { halfX, halfY, halfZ };
            for (int i = 0; i < 8; ++i)
            {
                geometry.m_vertices.push_back((i & 1) ? halves[0] : -halves[0]);
                geometry.m_vertices.push_back((i & 2) ? halves[1] : -halves[1]);
                geometry.m_vertices.push_back((i & 4) ? halves[2] : -halves[2]);
            }
            geometry.m_indices = { 0, 1, 3, 0, 3, 2, 4, 5, 7, 4, 7, 6 };
            return geometry;
        }

        static AzFramework::VisibleGeometry MakeCubeGeometry(float halfExtent)
        {
            return MakeBoxGeometry(halfExtent, halfExtent, halfExtent);
        }

        //! A single triangle: three points can never form a convex hull on their own.
        static AzFramework::VisibleGeometry MakeTriangleGeometry(
            const AZ::Vector3& a, const AZ::Vector3& b, const AZ::Vector3& c)
        {
            AzFramework::VisibleGeometry geometry;
            geometry.m_transform = AZ::Matrix4x4::CreateIdentity();
            for (const AZ::Vector3& vertex : { a, b, c })
            {
                geometry.m_vertices.push_back(vertex.GetX());
                geometry.m_vertices.push_back(vertex.GetY());
                geometry.m_vertices.push_back(vertex.GetZ());
            }
            geometry.m_indices = { 0, 1, 2 };
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

    TEST_F(JoltMeshColliderTests, PerEntryCookingProducesAHullPerGeometryEntry)
    {
        // Two boxes apart from each other, as "render nodes": each gets its own hull.
        AzFramework::VisibleGeometryContainer container;
        container.push_back(MakeBoxGeometry(
            0.5f, 0.5f, 0.5f, AZ::Matrix4x4::CreateTranslation(AZ::Vector3(-3.0f, 0.0f, 0.0f))));
        container.push_back(MakeBoxGeometry(
            1.0f, 0.5f, 2.0f, AZ::Matrix4x4::CreateTranslation(AZ::Vector3(3.0f, 0.0f, 0.0f))));

        Physics::CookedMeshShapeConfiguration cookedConfig;
        ASSERT_TRUE(JoltMeshUtils::CookVisibleGeometry(
            container, AZ::Transform::CreateIdentity(),
            Physics::CookedMeshShapeConfiguration::MeshType::Convex, cookedConfig,
            JoltMeshUtils::ConvexGrouping::PerGeometryEntry));

        JPH::RefConst<JPH::Shape> shape = JoltMeshUtils::CreateConvexShapeFromCookedData(cookedConfig.GetCookedMeshData());
        ASSERT_NE(shape, nullptr);
        ASSERT_EQ(shape->GetSubType(), JPH::EShapeSubType::StaticCompound);
        const auto* compound = static_cast<const JPH::CompoundShape*>(shape.GetPtr());
        EXPECT_EQ(compound->GetNumSubShapes(), 2u);

        // The group's bounds span both boxes. Jolt reports bounds given a *center-of-mass*
        // transform, and compound children are stored shifted by -COM, so translating the
        // COM transform by +COM puts the bounds back in the frame the hulls were baked in.
        const JPH::Mat44 comFrame = JPH::Mat44::sTranslation(shape->GetCenterOfMass());
        const JPH::AABox bounds = shape->GetWorldSpaceBounds(comFrame, JPH::Vec3::sReplicate(1.0f));
        EXPECT_NEAR(bounds.mMin.GetX(), -3.5f, 0.1f);
        EXPECT_NEAR(bounds.mMax.GetX(), 4.0f, 0.1f);
    }

    TEST_F(JoltMeshColliderTests, PerEntryCookingWithOneEntryStaysABareHull)
    {
        AzFramework::VisibleGeometryContainer container;
        container.push_back(MakeCubeGeometry(0.5f));

        Physics::CookedMeshShapeConfiguration cookedConfig;
        ASSERT_TRUE(JoltMeshUtils::CookVisibleGeometry(
            container, AZ::Transform::CreateIdentity(),
            Physics::CookedMeshShapeConfiguration::MeshType::Convex, cookedConfig,
            JoltMeshUtils::ConvexGrouping::PerGeometryEntry));

        // A one-entry group decodes to a plain hull, not a one-child compound.
        JPH::RefConst<JPH::Shape> shape = JoltMeshUtils::CreateConvexShapeFromCookedData(cookedConfig.GetCookedMeshData());
        ASSERT_NE(shape, nullptr);
        EXPECT_EQ(shape->GetType(), JPH::EShapeType::Convex);
    }

    TEST_F(JoltMeshColliderTests, PerEntryCookingMergesDegenerateEntriesIntoASharedHull)
    {
        // A cube plus two lone triangles: the triangles cannot hull individually, but
        // together they span enough volume to form one shared second hull.
        AzFramework::VisibleGeometryContainer container;
        container.push_back(MakeCubeGeometry(0.5f));
        container.push_back(MakeTriangleGeometry(
            AZ::Vector3(5.0f, 0.0f, 0.0f), AZ::Vector3(6.0f, 0.0f, 0.0f), AZ::Vector3(5.0f, 1.0f, 0.0f)));
        container.push_back(MakeTriangleGeometry(
            AZ::Vector3(5.0f, 0.0f, 1.0f), AZ::Vector3(6.0f, 0.0f, 1.0f), AZ::Vector3(5.0f, 1.0f, 1.0f)));

        Physics::CookedMeshShapeConfiguration cookedConfig;
        ASSERT_TRUE(JoltMeshUtils::CookVisibleGeometry(
            container, AZ::Transform::CreateIdentity(),
            Physics::CookedMeshShapeConfiguration::MeshType::Convex, cookedConfig,
            JoltMeshUtils::ConvexGrouping::PerGeometryEntry));

        JPH::RefConst<JPH::Shape> shape = JoltMeshUtils::CreateConvexShapeFromCookedData(cookedConfig.GetCookedMeshData());
        ASSERT_NE(shape, nullptr);
        ASSERT_EQ(shape->GetSubType(), JPH::EShapeSubType::StaticCompound);
        EXPECT_EQ(static_cast<const JPH::CompoundShape*>(shape.GetPtr())->GetNumSubShapes(), 2u);
    }

    TEST_F(JoltMeshColliderTests, PerEntryCookingFallsBackToOneHullWhenLeftoversCannotGroup)
    {
        // A cube plus a single flat triangle: the triangle can never hull, and neither
        // can it alone, so grouping falls back to one hull around everything rather
        // than dropping the triangle.
        AzFramework::VisibleGeometryContainer container;
        container.push_back(MakeCubeGeometry(0.5f));
        container.push_back(MakeTriangleGeometry(
            AZ::Vector3(5.0f, 0.0f, 0.0f), AZ::Vector3(6.0f, 0.0f, 0.0f), AZ::Vector3(5.0f, 1.0f, 0.0f)));

        Physics::CookedMeshShapeConfiguration cookedConfig;
        ASSERT_TRUE(JoltMeshUtils::CookVisibleGeometry(
            container, AZ::Transform::CreateIdentity(),
            Physics::CookedMeshShapeConfiguration::MeshType::Convex, cookedConfig,
            JoltMeshUtils::ConvexGrouping::PerGeometryEntry));

        JPH::RefConst<JPH::Shape> shape = JoltMeshUtils::CreateConvexShapeFromCookedData(cookedConfig.GetCookedMeshData());
        ASSERT_NE(shape, nullptr);
        EXPECT_EQ(shape->GetType(), JPH::EShapeType::Convex); // single merged hull, no compound

        // ...and the triangle's far point made it into that hull (bounds given a COM
        // transform that places the centroid at its baked position, i.e. baked frame).
        const JPH::AABox bounds = shape->GetWorldSpaceBounds(
            JPH::Mat44::sTranslation(shape->GetCenterOfMass()), JPH::Vec3::sReplicate(1.0f));
        EXPECT_GT(bounds.mMax.GetX(), 5.0f);
    }

    TEST_F(JoltMeshColliderTests, CookedHullGroupSupportsRestingSphere)
    {
        // Two boxes forming a continuous floor (top at z=0.5), each its own hull.
        AzFramework::VisibleGeometryContainer container;
        container.push_back(MakeBoxGeometry(
            2.0f, 5.0f, 0.5f, AZ::Matrix4x4::CreateTranslation(AZ::Vector3(-2.0f, 0.0f, 0.0f))));
        container.push_back(MakeBoxGeometry(
            2.0f, 5.0f, 0.5f, AZ::Matrix4x4::CreateTranslation(AZ::Vector3(2.0f, 0.0f, 0.0f))));

        auto cookedConfig = AZStd::make_shared<Physics::CookedMeshShapeConfiguration>();
        ASSERT_TRUE(JoltMeshUtils::CookVisibleGeometry(
            container, AZ::Transform::CreateIdentity(),
            Physics::CookedMeshShapeConfiguration::MeshType::Convex, *cookedConfig,
            JoltMeshUtils::ConvexGrouping::PerGeometryEntry));

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

        // The sphere rests on the hull group instead of falling between the hulls.
        const float sphereZ = m_scene->GetSimulatedBodyFromHandle(sphereHandle)->GetPosition().GetZ();
        EXPECT_NEAR(sphereZ, 1.0f, 0.05f);

        // Release the native mesh cached on the configuration (in production this is
        // balanced by JoltPhysicsSystemComponent::ReleaseNativeMeshObject, which the
        // test environment does not run).
        if (auto* cachedMesh = static_cast<JPH::Shape*>(cookedConfig->GetCachedNativeMesh()))
        {
            cachedMesh->Release();
            cookedConfig->SetCachedNativeMesh(nullptr);
        }
    }

    TEST_F(JoltMeshColliderTests, GetGeometryCoversEveryHullInAGroup)
    {
        AzFramework::VisibleGeometryContainer container;
        container.push_back(MakeBoxGeometry(
            0.5f, 0.5f, 0.5f, AZ::Matrix4x4::CreateTranslation(AZ::Vector3(-3.0f, 0.0f, 0.0f))));
        container.push_back(MakeBoxGeometry(
            0.5f, 0.5f, 0.5f, AZ::Matrix4x4::CreateTranslation(AZ::Vector3(3.0f, 0.0f, 0.0f))));

        auto cookedConfig = AZStd::make_shared<Physics::CookedMeshShapeConfiguration>();
        ASSERT_TRUE(JoltMeshUtils::CookVisibleGeometry(
            container, AZ::Transform::CreateIdentity(),
            Physics::CookedMeshShapeConfiguration::MeshType::Convex, *cookedConfig,
            JoltMeshUtils::ConvexGrouping::PerGeometryEntry));

        // A hull group is a compound, which refuses GetTrianglesStart ("non-leaf"):
        // GetGeometry has to walk its leaves to see both boxes.
        AZStd::shared_ptr<Physics::Shape> shape =
            JoltShapeUtils::CreateShape(Physics::ColliderConfiguration(), *cookedConfig);
        ASSERT_NE(shape, nullptr);

        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        shape->GetGeometry(vertices, indices, nullptr);
        ASSERT_FALSE(vertices.empty());

        float minX = AZStd::numeric_limits<float>::max();
        float maxX = AZStd::numeric_limits<float>::lowest();
        for (const AZ::Vector3& vertex : vertices)
        {
            minX = AZStd::min(minX, vertex.GetX());
            maxX = AZStd::max(maxX, vertex.GetX());
        }
        EXPECT_LT(minX, -3.4f);
        EXPECT_GT(maxX, 3.4f);

        // Release the native mesh cached on the configuration (in production this is
        // balanced by JoltPhysicsSystemComponent::ReleaseNativeMeshObject, which the
        // test environment does not run).
        if (auto* cachedMesh = static_cast<JPH::Shape*>(cookedConfig->GetCachedNativeMesh()))
        {
            cachedMesh->Release();
            cookedConfig->SetCachedNativeMesh(nullptr);
        }
    }

    TEST_F(JoltMeshColliderTests, GetGeometryExtractsTrianglesThroughDecoratedShapes)
    {
        // The gem's capsule is a RotatedTranslatedShape wrapper, which refuses
        // GetTrianglesStart ("non-leaf") just like a compound: GetGeometry used to
        // return nothing for it.
        auto capsuleConfig = AZStd::make_shared<Physics::CapsuleShapeConfiguration>(2.0f, 0.5f); // (height, radius)
        AZStd::shared_ptr<Physics::Shape> shape =
            JoltShapeUtils::CreateShape(Physics::ColliderConfiguration(), *capsuleConfig);
        ASSERT_NE(shape, nullptr);

        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        shape->GetGeometry(vertices, indices, nullptr);
        EXPECT_FALSE(vertices.empty());
        EXPECT_EQ(vertices.size(), indices.size());
    }

    TEST_F(JoltMeshColliderTests, PerFaceMaterialIndicesSurviveTheRoundTrip)
    {
        // Two quads side by side; the left one maps to slot 0, the right one to slot 1.
        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        auto appendQuad = [&vertices, &indices](float centerX)
        {
            const AZ::u32 base = static_cast<AZ::u32>(vertices.size());
            vertices.emplace_back(centerX - 0.5f, -1.0f, 0.0f);
            vertices.emplace_back(centerX + 0.5f, -1.0f, 0.0f);
            vertices.emplace_back(centerX + 0.5f, 1.0f, 0.0f);
            vertices.emplace_back(centerX - 0.5f, 1.0f, 0.0f);
            indices.insert(indices.end(), { base, base + 1, base + 2, base, base + 2, base + 3 });
        };
        appendQuad(-1.0f);
        appendQuad(1.0f);
        const AZ::u8 materials[4] = { 0, 0, 1, 1 };

        const AZStd::vector<AZ::u8> blob = JoltMeshUtils::PackTriangleMesh(
            vertices.data(), static_cast<AZ::u32>(vertices.size()),
            indices.data(), static_cast<AZ::u32>(indices.size()),
            materials, 4);
        ASSERT_FALSE(blob.empty());
        const JPH::RefConst<JPH::Shape> shape = JoltMeshUtils::CreateMeshShapeFromCookedData(blob);
        ASSERT_NE(shape, nullptr);
        ASSERT_EQ(shape->GetSubType(), JPH::EShapeSubType::Mesh);
        const auto* mesh = static_cast<const JPH::MeshShape*>(shape.GetPtr());

        // Rays straight down at each quad report their face's slot.
        auto materialIndexAt = [mesh](float x)
        {
            FirstHitCollector collector;
            mesh->CastRay(
                JPH::RayCast(JPH::Vec3(x, 0.0f, 1.0f), JPH::Vec3(0.0f, 0.0f, -1.0f)),
                JPH::RayCastSettings(), JPH::SubShapeIDCreator(), collector);
            EXPECT_TRUE(collector.m_hit);
            return mesh->GetMaterialIndex(collector.m_subShapeId);
        };
        EXPECT_EQ(materialIndexAt(-1.0f), 0u);
        EXPECT_EQ(materialIndexAt(1.0f), 1u);
    }

    TEST_F(JoltMeshColliderTests, VersionOneBlobCarriesNoMaterialTable)
    {
        const AZ::Vector3 vertices[4] = {
            AZ::Vector3(-1.0f, -1.0f, 0.0f), AZ::Vector3(1.0f, -1.0f, 0.0f),
            AZ::Vector3(1.0f, 1.0f, 0.0f),   AZ::Vector3(-1.0f, 1.0f, 0.0f),
        };
        const AZ::u32 indices[6] = { 0, 1, 2, 0, 2, 3 };

        const AZStd::vector<AZ::u8> blob = JoltMeshUtils::PackTriangleMesh(vertices, 4, indices, 6);
        ASSERT_FALSE(blob.empty());
        const JPH::RefConst<JPH::Shape> shape = JoltMeshUtils::CreateMeshShapeFromCookedData(blob);
        ASSERT_NE(shape, nullptr);
        const auto* mesh = static_cast<const JPH::MeshShape*>(shape.GetPtr());

        FirstHitCollector collector;
        mesh->CastRay(
            JPH::RayCast(JPH::Vec3(0.0f, 0.0f, 1.0f), JPH::Vec3(0.0f, 0.0f, -1.0f)),
            JPH::RayCastSettings(), JPH::SubShapeIDCreator(), collector);
        ASSERT_TRUE(collector.m_hit);
        EXPECT_EQ(mesh->GetMaterialIndex(collector.m_subShapeId), 0u);
    }

    TEST_F(JoltMeshColliderTests, MaterialIndicesAbove31Clamp)
    {
        // Jolt packs material indices as 5 bits per triangle; an oversized index falls
        // to the last representable slot instead of wrapping into a neighbor's bits.
        const AZ::Vector3 vertices[3] = {
            AZ::Vector3(-1.0f, -1.0f, 0.0f), AZ::Vector3(1.0f, -1.0f, 0.0f), AZ::Vector3(0.0f, 1.0f, 0.0f),
        };
        const AZ::u32 indices[3] = { 0, 1, 2 };
        const AZ::u8 materials[1] = { 200 };

        const AZStd::vector<AZ::u8> blob = JoltMeshUtils::PackTriangleMesh(vertices, 3, indices, 3, materials, 1);
        ASSERT_FALSE(blob.empty());
        const JPH::RefConst<JPH::Shape> shape = JoltMeshUtils::CreateMeshShapeFromCookedData(blob);
        ASSERT_NE(shape, nullptr);
        const auto* mesh = static_cast<const JPH::MeshShape*>(shape.GetPtr());

        FirstHitCollector collector;
        mesh->CastRay(
            JPH::RayCast(JPH::Vec3(0.0f, 0.0f, 1.0f), JPH::Vec3(0.0f, 0.0f, -1.0f)),
            JPH::RayCastSettings(), JPH::SubShapeIDCreator(), collector);
        ASSERT_TRUE(collector.m_hit);
        EXPECT_EQ(mesh->GetMaterialIndex(collector.m_subShapeId), 31u);
    }

    // Cooked blobs arrive from disk, so the decoders are the gem's boundary with data it
    // did not produce this run: an interrupted Asset Processor write, a corrupted product,
    // a stale file. Every one of these must reach the error-and-null path rather than
    // reading past the buffer or trying to allocate its way out.
    class JoltBlobDecoderNegativePaths : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            // Decoding allocates through Jolt (JPH::Array), and unlike the other fixtures
            // here this one builds no JoltSystem - so nothing would have installed this
            // module's allocation hooks and the first successful hull read would jump
            // through a null pointer. Exactly the trap JoltModuleGlobals.h exists for.
            //
            // The hooks alone, not the full installer: decoding needs no factory, and
            // creating one here would register types nothing tears down again.
            InstallJoltAllocationHooks();
        }

        static AZStd::vector<AZ::u8> MakeHullGroupBlob()
        {
            AZStd::vector<AZ::Vector3> cube;
            for (const float dx : { -0.5f, 0.5f })
            {
                for (const float dy : { -0.5f, 0.5f })
                {
                    for (const float dz : { -0.5f, 0.5f })
                    {
                        cube.push_back(AZ::Vector3(dx, dy, dz));
                    }
                }
            }
            return JoltMeshUtils::PackConvexHulls({ cube, cube });
        }

        //! Decoding failures are the expected outcome here, so their diagnostics are
        //! captured rather than printed - and their presence is what the tests check.
        JoltWarningCatcher m_diagnostics;
    };

    TEST_F(JoltBlobDecoderNegativePaths, AHullCountLargerThanTheBlobIsRejectedNotReservedFor)
    {
        // The hull count is read straight out of the file and used to reserve. A bit flip
        // in that field asked for up to four billion vector objects - an allocation
        // failure inside reserve, rather than the clean null this returns.
        AZStd::vector<AZ::u8> blob = MakeHullGroupBlob();
        ASSERT_GT(blob.size(), 20u);

        // The hull count sits immediately after the 12-byte header.
        const AZ::u32 absurdCount = 0xFFFFFFF0u;
        memcpy(blob.data() + 12, &absurdCount, sizeof(absurdCount));

        EXPECT_EQ(JoltMeshUtils::CreateConvexShapeFromCookedData(blob), nullptr);
        EXPECT_TRUE(m_diagnostics.ContainsWarningWith("cannot fit"))
            << "the rejection should say why, not just return null";
    }

    TEST_F(JoltBlobDecoderNegativePaths, TruncatedHullGroupBlobsDecodeToNothing)
    {
        const AZStd::vector<AZ::u8> good = MakeHullGroupBlob();
        for (size_t size = 0; size < good.size(); size += AZStd::max<size_t>(good.size() / 16, 1))
        {
            const AZStd::vector<AZ::u8> truncated(good.begin(), good.begin() + size);
            EXPECT_EQ(JoltMeshUtils::CreateConvexShapeFromCookedData(truncated), nullptr)
                << "a blob truncated to " << size << " bytes decoded to a shape";
        }
    }

    TEST_F(JoltBlobDecoderNegativePaths, GarbageBytesDecodeToNothing)
    {
        const AZStd::vector<AZ::u8> garbage(64, 0xAB);
        EXPECT_EQ(JoltMeshUtils::CreateConvexShapeFromCookedData(garbage), nullptr);
        EXPECT_EQ(JoltMeshUtils::CreateMeshShapeFromCookedData(garbage), nullptr);
    }

    TEST_F(JoltBlobDecoderNegativePaths, AValidHeaderOverACorruptedBodyDecodesToNothing)
    {
        AZStd::vector<AZ::u8> corrupted = MakeHullGroupBlob();
        for (size_t i = 12; i < corrupted.size(); ++i)
        {
            corrupted[i] = 0xFF;
        }
        EXPECT_EQ(JoltMeshUtils::CreateConvexShapeFromCookedData(corrupted), nullptr);
    }

} // namespace JoltPhysics
