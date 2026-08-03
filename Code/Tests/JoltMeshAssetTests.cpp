#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <AzCore/Asset/AssetManager.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/IO/ByteContainerStream.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/Utils.h>

#include <Clients/Components/JoltMeshColliderComponent.h>
#include <Configuration/JoltSettingsRegistryManager.h>
#include <JoltPhysics/Pipeline/JoltMeshAsset.h>
#include <Scene/JoltScene.h>
#include <Shape/JoltMeshUtils.h>
#include <Shape/JoltShapeUtils.h>
#include <System/JoltSystem.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/Material/PhysicsMaterial.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace JoltPhysics
{
    //! The .joltmesh asset type and the collider expansion that consumes it: binary
    //! serialization round-trips (the format the Scene Builder writes), one collider
    //! pair per asset shape with materials/overrides/scale, and the end-to-end path
    //! from asset to resting body.
    class JoltMeshAssetTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "MeshAssetTestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);

            AZ::ComponentApplicationBus::BroadcastResult(
                m_serializeContext, &AZ::ComponentApplicationRequests::GetSerializeContext);
            AZ_Assert(m_serializeContext != nullptr, "No application serialize context");
        }

        void TearDown() override
        {
            m_system->RemoveScene(m_sceneHandle);
            m_system->Shutdown();
            m_system.reset();
        }

        //! The eight corners of a unit box as a convex point cloud.
        static AZStd::vector<AZ::Vector3> MakeBoxPoints(const AZ::Vector3& center, float halfExtent)
        {
            AZStd::vector<AZ::Vector3> points;
            for (int i = 0; i < 8; ++i)
            {
                points.emplace_back(
                    center.GetX() + ((i & 1) ? halfExtent : -halfExtent),
                    center.GetY() + ((i & 2) ? halfExtent : -halfExtent),
                    center.GetZ() + ((i & 4) ? halfExtent : -halfExtent));
            }
            return points;
        }

        static AZ::Data::Asset<Physics::MaterialAsset> CreateMaterialAsset(float friction, float restitution)
        {
            const Physics::MaterialAsset::MaterialProperties properties = {
                { "DynamicFriction", friction },
                { "StaticFriction", friction },
                { "Restitution", restitution },
            };
            AZ::Data::Asset<Physics::MaterialAsset> asset =
                AZ::Data::AssetManager::Instance().CreateAsset<Physics::MaterialAsset>(
                    AZ::Data::AssetId(AZ::Uuid::CreateRandom()));
            asset->SetData("JoltMaterial", 1, properties);
            return asset;
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle;
        AzPhysics::Scene* m_scene = nullptr;
        AZ::SerializeContext* m_serializeContext = nullptr;
    };

    TEST_F(JoltMeshAssetTests, MeshAssetDataRoundTripsThroughBinarySerialization)
    {
        // The Scene Builder writes the bare asset-data struct as a binary ObjectStream;
        // the handler reads the same struct back. Pin the format with a full round trip.
        Pipeline::JoltMeshAssetData source;

        auto boxConfig = AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(1.0f, 2.0f, 3.0f));
        source.m_colliderShapes.emplace_back(
            AZStd::make_shared<Pipeline::JoltAssetColliderConfiguration>(), boxConfig);

        const AZStd::vector<AZ::Vector3> points = MakeBoxPoints(AZ::Vector3::CreateZero(), 0.5f);
        const AZStd::vector<AZ::u8> blob = JoltMeshUtils::PackConvexMesh(points.data(), static_cast<AZ::u32>(points.size()));
        auto convexConfig = AZStd::make_shared<Physics::CookedMeshShapeConfiguration>();
        convexConfig->SetCookedMeshData(blob.data(), blob.size(), Physics::CookedMeshShapeConfiguration::MeshType::Convex);
        source.m_colliderShapes.emplace_back(
            AZStd::make_shared<Pipeline::JoltAssetColliderConfiguration>(), convexConfig);

        source.m_materialSlots.SetSlots({ "wood", "metal" });
        source.m_materialIndexPerShape = { 0, 1 };

        AZStd::vector<AZ::u8> bytes;
        AZ::IO::ByteContainerStream<AZStd::vector<AZ::u8>> stream(&bytes);
        ASSERT_TRUE(AZ::Utils::SaveObjectToStream(
            stream, AZ::DataStream::ST_BINARY, &source, m_serializeContext));

        Pipeline::JoltMeshAssetData loaded;
        stream.Seek(0, AZ::IO::GenericStream::ST_SEEK_BEGIN);
        ASSERT_TRUE(AZ::Utils::LoadObjectFromStreamInPlace(stream, loaded, m_serializeContext));

        ASSERT_EQ(loaded.m_colliderShapes.size(), 2u);
        ASSERT_EQ(loaded.m_colliderShapes[0].second->GetShapeType(), Physics::ShapeType::Box);
        const auto* loadedBox = static_cast<const Physics::BoxShapeConfiguration*>(loaded.m_colliderShapes[0].second.get());
        EXPECT_TRUE(loadedBox->m_dimensions.IsClose(AZ::Vector3(1.0f, 2.0f, 3.0f)));

        ASSERT_EQ(loaded.m_colliderShapes[1].second->GetShapeType(), Physics::ShapeType::CookedMesh);
        const auto* loadedConvex =
            static_cast<const Physics::CookedMeshShapeConfiguration*>(loaded.m_colliderShapes[1].second.get());
        EXPECT_EQ(loadedConvex->GetCookedMeshData(), blob);
        EXPECT_EQ(loadedConvex->GetMeshType(), Physics::CookedMeshShapeConfiguration::MeshType::Convex);

        ASSERT_EQ(loaded.m_materialIndexPerShape.size(), 2u);
        EXPECT_EQ(loaded.m_materialIndexPerShape[0], 0);
        EXPECT_EQ(loaded.m_materialIndexPerShape[1], 1);
        EXPECT_EQ(loaded.m_materialSlots.GetSlotName(0), "wood");
        EXPECT_EQ(loaded.m_materialSlots.GetSlotName(1), "metal");
    }

    TEST_F(JoltMeshAssetTests, ExpansionCreatesOnePairPerAssetShape)
    {
        Pipeline::JoltMeshAssetData assetData;

        // Shape 0: a box with no overrides, mapped to material slot 0.
        assetData.m_colliderShapes.emplace_back(
            AZStd::make_shared<Pipeline::JoltAssetColliderConfiguration>(),
            AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(1.0f, 1.0f, 1.0f)));

        // Shape 1: a sphere overriding layer, trigger, tag and offset, mapped to slot 1.
        auto overrides = AZStd::make_shared<Pipeline::JoltAssetColliderConfiguration>();
        overrides->m_collisionLayer = AzPhysics::CollisionLayer(3);
        overrides->m_isTrigger = true;
        overrides->m_tag = AZStd::string("wheel");
        overrides->m_transform = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 1.0f));
        assetData.m_colliderShapes.emplace_back(
            overrides, AZStd::make_shared<Physics::SphereShapeConfiguration>(0.5f));

        const AZ::Data::Asset<Physics::MaterialAsset> materialA = CreateMaterialAsset(0.4f, 0.0f);
        const AZ::Data::Asset<Physics::MaterialAsset> materialB = CreateMaterialAsset(0.6f, 0.9f);
        assetData.m_materialSlots.SetSlots({ "a", "b" });
        assetData.m_materialSlots.SetMaterialAsset(0, materialA);
        assetData.m_materialSlots.SetMaterialAsset(1, materialB);
        assetData.m_materialIndexPerShape = { 0, 1 };

        Physics::ColliderConfiguration colliderConfig;
        colliderConfig.m_position = AZ::Vector3(1.0f, 0.0f, 0.0f);
        colliderConfig.m_materialSlots = assetData.m_materialSlots;

        const AZ::Vector3 overallScale(2.0f, 2.0f, 2.0f);
        const AzPhysics::ShapeColliderPairList pairs =
            ExpandJoltMeshAssetColliderShapes(assetData, colliderConfig, overallScale);

        ASSERT_EQ(pairs.size(), 2u);

        // Shape 0: no overrides; the component offset scales with the shape.
        EXPECT_EQ(pairs[0].second->GetShapeType(), Physics::ShapeType::Box);
        EXPECT_TRUE(pairs[0].second->m_scale.IsClose(overallScale));
        EXPECT_TRUE(pairs[0].first->m_position.IsClose(AZ::Vector3(2.0f, 0.0f, 0.0f)));
        EXPECT_FALSE(pairs[0].first->m_isTrigger);

        // Shape 1: overrides win over the component configuration.
        EXPECT_EQ(pairs[1].second->GetShapeType(), Physics::ShapeType::Sphere);
        EXPECT_TRUE(pairs[1].second->m_scale.IsClose(overallScale));
        EXPECT_TRUE(pairs[1].first->m_isTrigger);
        EXPECT_EQ(pairs[1].first->m_tag, "wheel");
        EXPECT_EQ(pairs[1].first->m_collisionLayer, AzPhysics::CollisionLayer(3));
        // Override transform composes with the (unscaled) component offset, then scales.
        EXPECT_TRUE(pairs[1].first->m_position.IsClose(AZ::Vector3(2.0f, 0.0f, 2.0f)));

        // Each pair carries exactly one slot holding its mapped material.
        ASSERT_EQ(pairs[0].first->m_materialSlots.GetSlotsCount(), 1u);
        EXPECT_EQ(pairs[0].first->m_materialSlots.GetMaterialAsset(0).GetId(), materialA.GetId());
        ASSERT_EQ(pairs[1].first->m_materialSlots.GetSlotsCount(), 1u);
        EXPECT_EQ(pairs[1].first->m_materialSlots.GetMaterialAsset(0).GetId(), materialB.GetId());
    }

    TEST_F(JoltMeshAssetTests, PrimitiveEntryExpandsWithItsFitTransform)
    {
        // The Primitive export mode stores a plain shape config plus the fitted
        // transform on the entry's collider configuration; expansion must compose it.
        Pipeline::JoltMeshAssetData assetData;
        auto colliderOverrides = AZStd::make_shared<Pipeline::JoltAssetColliderConfiguration>();
        colliderOverrides->m_transform = AZ::Transform::CreateTranslation(AZ::Vector3(1.0f, 2.0f, 3.0f));
        assetData.m_colliderShapes.emplace_back(
            colliderOverrides, AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(2.0f, 4.0f, 6.0f)));
        assetData.m_materialIndexPerShape = { 0 };
        assetData.m_materialSlots.SetSlots({ "a" });

        const AzPhysics::ShapeColliderPairList pairs =
            ExpandJoltMeshAssetColliderShapes(assetData, Physics::ColliderConfiguration(), AZ::Vector3::CreateOne());
        ASSERT_EQ(pairs.size(), 1u);
        EXPECT_EQ(pairs[0].second->GetShapeType(), Physics::ShapeType::Box);
        EXPECT_TRUE(pairs[0].first->m_position.IsClose(AZ::Vector3(1.0f, 2.0f, 3.0f)));
    }

    TEST_F(JoltMeshAssetTests, ScaledShapeConfigProducesScaledShape)
    {
        auto boxConfig = AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(1.0f, 2.0f, 3.0f));
        boxConfig->m_scale = AZ::Vector3(2.0f, 2.0f, 2.0f);

        JPH::RefConst<JPH::Shape> shape = JoltShapeUtils::CreateJoltShapeFromConfig(*boxConfig);
        ASSERT_NE(shape, nullptr);
        ASSERT_EQ(shape->GetSubType(), JPH::EShapeSubType::Scaled);
        const auto* scaled = static_cast<const JPH::ScaledShape*>(shape.GetPtr());
        EXPECT_EQ(scaled->GetScale(), JPH::Vec3(2.0f, 2.0f, 2.0f));

        // Unit scale stays the bare shape: no decorator tax on the common case.
        boxConfig->m_scale = AZ::Vector3::CreateOne();
        shape = JoltShapeUtils::CreateJoltShapeFromConfig(*boxConfig);
        ASSERT_NE(shape, nullptr);
        EXPECT_EQ(shape->GetSubType(), JPH::EShapeSubType::Box);
    }

    TEST_F(JoltMeshAssetTests, AssetSourcedTriangleMeshSupportsRestingSphere)
    {
        // A 20x20 quad as the asset's single triangle-mesh shape.
        const AZ::Vector3 quadVertices[4] = {
            AZ::Vector3(-10.0f, -10.0f, 0.0f), AZ::Vector3(10.0f, -10.0f, 0.0f),
            AZ::Vector3(10.0f, 10.0f, 0.0f),   AZ::Vector3(-10.0f, 10.0f, 0.0f),
        };
        const AZ::u32 quadIndices[6] = { 0, 1, 2, 0, 2, 3 };
        const AZStd::vector<AZ::u8> blob =
            JoltMeshUtils::PackTriangleMesh(quadVertices, 4, quadIndices, 6);

        auto triConfig = AZStd::make_shared<Physics::CookedMeshShapeConfiguration>();
        triConfig->SetCookedMeshData(blob.data(), blob.size(), Physics::CookedMeshShapeConfiguration::MeshType::TriangleMesh);

        Pipeline::JoltMeshAssetData assetData;
        assetData.m_colliderShapes.emplace_back(
            AZStd::make_shared<Pipeline::JoltAssetColliderConfiguration>(), triConfig);
        assetData.m_materialIndexPerShape = { Pipeline::JoltMeshAssetData::TriangleMeshMaterialIndex };

        const AzPhysics::ShapeColliderPairList pairs =
            ExpandJoltMeshAssetColliderShapes(assetData, Physics::ColliderConfiguration(), AZ::Vector3::CreateOne());
        ASSERT_EQ(pairs.size(), 1u);

        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_colliderAndShapeData = pairs;
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

        // The sphere rests on the asset-sourced quad instead of falling through it.
        const float sphereZ = m_scene->GetSimulatedBodyFromHandle(sphereHandle)->GetPosition().GetZ();
        EXPECT_NEAR(sphereZ, 0.5f, 0.05f);

        // Release the native mesh cached on the expanded (cloned) configuration, the same
        // balancing the system component does in production via ReleaseNativeMeshObject.
        auto* expandedCookedConfig =
            static_cast<Physics::CookedMeshShapeConfiguration*>(pairs[0].second.get());
        if (auto* cachedMesh = static_cast<JPH::Shape*>(expandedCookedConfig->GetCachedNativeMesh()))
        {
            cachedMesh->Release();
            expandedCookedConfig->SetCachedNativeMesh(nullptr);
        }
    }

} // namespace JoltPhysics
