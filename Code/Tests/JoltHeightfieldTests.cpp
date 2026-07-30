#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <Material/JoltMaterial.h>
#include <RigidBody/JoltStaticRigidBody.h>
#include <Shape/JoltHeightfieldUtils.h>
#include <System/JoltSystem.h>
#include <Scene/JoltScene.h>
#include <Configuration/JoltSettingsRegistryManager.h>

#include <AzCore/Asset/AssetManager.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/HeightfieldProviderBus.h>
#include <AzFramework/Physics/Material/PhysicsMaterialManager.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

#include <Jolt/Jolt.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>

namespace JoltPhysics
{
    // Minimal HeightfieldProviderRequests implementation for tests: a small grid
    // with configurable heights and per-sample material indices.
    class MockHeightfieldProvider : public Physics::HeightfieldProviderRequestsBus::Handler
    {
    public:
        MockHeightfieldProvider(AZ::EntityId providerEntityId, AZ::u32 columns, AZ::u32 rows, const AZ::Vector2& spacing)
            : m_columns(columns)
            , m_rows(rows)
            , m_spacing(spacing)
        {
            m_heights.assign(columns * rows, 0.0f);
            m_materialIndices.assign(columns * rows, 0);
            Physics::HeightfieldProviderRequestsBus::Handler::BusConnect(providerEntityId);
        }

        ~MockHeightfieldProvider() override
        {
            Physics::HeightfieldProviderRequestsBus::Handler::BusDisconnect();
        }

        AZ::Vector2 GetHeightfieldGridSpacing() const override { return m_spacing; }
        void GetHeightfieldGridSize(size_t& numColumns, size_t& numRows) const override
        {
            numColumns = m_columns;
            numRows = m_rows;
        }
        AZ::u64 GetHeightfieldGridColumns() const override { return m_columns; }
        AZ::u64 GetHeightfieldGridRows() const override { return m_rows; }
        void GetHeightfieldHeightBounds(float& minHeightBounds, float& maxHeightBounds) const override
        {
            minHeightBounds = -10.0f;
            maxHeightBounds = 10.0f;
        }
        float GetHeightfieldMinHeight() const override { return -10.0f; }
        float GetHeightfieldMaxHeight() const override { return 10.0f; }
        AZ::Aabb GetHeightfieldAabb() const override { return AZ::Aabb::CreateNull(); }
        AZ::Transform GetHeightfieldTransform() const override { return AZ::Transform::CreateIdentity(); }
        AZStd::vector<AZ::Data::Asset<Physics::MaterialAsset>> GetMaterialList() const override { return m_materials; }
        AZStd::vector<float> GetHeights() const override { return m_heights; }
        AZStd::vector<Physics::HeightMaterialPoint> GetHeightsAndMaterials() const override
        {
            AZStd::vector<Physics::HeightMaterialPoint> points;
            points.reserve(m_heights.size());
            for (size_t i = 0; i < m_heights.size(); ++i)
            {
                points.emplace_back(m_heights[i], Physics::QuadMeshType::SubdivideUpperLeftToBottomRight, m_materialIndices[i]);
            }
            return points;
        }
        void GetHeightfieldIndicesFromRegion(
            const AZ::Aabb& /*region*/,
            size_t& startColumn,
            size_t& startRow,
            size_t& numColumns,
            size_t& numRows) const override
        {
            startColumn = 0;
            startRow = 0;
            numColumns = m_columns;
            numRows = m_rows;
        }
        void UpdateHeightsAndMaterials(
            const Physics::UpdateHeightfieldSampleFunction& updateHeightsMaterialsCallback,
            size_t startColumn,
            size_t startRow,
            size_t numColumns,
            size_t numRows) const override
        {
            for (size_t y = startRow; y < startRow + numRows; ++y)
            {
                for (size_t x = startColumn; x < startColumn + numColumns; ++x)
                {
                    if (updateHeightsMaterialsCallback && y < m_rows && x < m_columns)
                    {
                        updateHeightsMaterialsCallback(x, y,
                            Physics::HeightMaterialPoint(m_heights[y * m_columns + x],
                                Physics::QuadMeshType::SubdivideUpperLeftToBottomRight, m_materialIndices[y * m_columns + x]));
                    }
                }
            }
        }
        void UpdateHeightsAndMaterialsAsync(
            const Physics::UpdateHeightfieldSampleFunction& /*updateHeightsMaterialsCallback*/,
            const Physics::UpdateHeightfieldCompleteFunction& /*updateHeightsCompleteCallback*/,
            size_t /*startColumn*/,
            size_t /*startRow*/,
            size_t /*numColumns*/,
            size_t /*numRows*/) const override
        {
        }

        AZ::u32 m_columns;
        AZ::u32 m_rows;
        AZ::Vector2 m_spacing;
        AZStd::vector<float> m_heights;
        AZStd::vector<AZ::u8> m_materialIndices;
        AZStd::vector<AZ::Data::Asset<Physics::MaterialAsset>> m_materials;
    };

    class JoltHeightfieldTests : public ::testing::Test
    {
    protected:
        static AZ::EntityId GetProviderEntityId()
        {
            return AZ::EntityId(0xC0FFEE);
        }

        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "HeightfieldTestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);
        }

        void TearDown() override
        {
            m_system->RemoveScene(m_sceneHandle);
            m_system->Shutdown();
            m_system.reset();
        }

        AzPhysics::SimulatedBodyHandle CreateHeightfieldBody(MockHeightfieldProvider& provider)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();

            AZStd::vector<AZ::u8> perSampleIndices;
            for (const auto& point : provider.GetHeightsAndMaterials())
            {
                perSampleIndices.push_back(point.m_materialIndex);
            }

            JPH::PhysicsMaterialList nativeMaterials;
            for (size_t i = 0; i < AZStd::max<size_t>(provider.m_materials.size(), 1); ++i)
            {
                nativeMaterials.push_back(JPH::PhysicsMaterial::sDefault);
            }

            m_lastHeightfieldShape = JoltHeightfieldUtils::CreateHeightFieldShape(
                provider.m_columns, provider.m_rows, provider.m_spacing, provider.m_heights, perSampleIndices, nativeMaterials);

            auto shapeConfig = AZStd::make_shared<Physics::HeightfieldShapeConfiguration>();
            shapeConfig->SetCachedNativeHeightfield(const_cast<JPH::Shape*>(m_lastHeightfieldShape.GetPtr()));

            AzPhysics::StaticRigidBodyConfiguration staticConfig;
            staticConfig.m_entityId = GetProviderEntityId();
            staticConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, shapeConfig);
            return m_scene->AddSimulatedBody(&staticConfig);
        }

        AZ::Data::Asset<Physics::MaterialAsset> CreateMaterialAssetForTest(float friction, float restitution)
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

        AzPhysics::SimulatedBodyHandle CreateDynamicSphere(const AZ::Vector3& position, float radius = 0.5f)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto sphereShape = AZStd::make_shared<Physics::SphereShapeConfiguration>();
            sphereShape->m_radius = radius;

            AzPhysics::RigidBodyConfiguration sphereConfig;
            sphereConfig.m_position = position;
            sphereConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, sphereShape);
            return m_scene->AddSimulatedBody(&sphereConfig);
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

        float RaycastDownZ(const AZ::Vector3& start)
        {

            AzPhysics::RayCastRequest request;
            request.m_start = start;
            request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
            request.m_distance = 100.0f;

            AzPhysics::SceneQueryHits hits = m_scene->QueryScene(&request);

            return hits.m_hits.empty() ? -1000.0f : hits.m_hits[0].m_position.GetZ();
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle;
        AzPhysics::Scene* m_scene = nullptr;
        JPH::RefConst<JPH::Shape> m_lastHeightfieldShape;
    };

    TEST_F(JoltHeightfieldTests, RaycastHitsSlopeAtExpectedHeight)
    {
        MockHeightfieldProvider provider(GetProviderEntityId(), 4, 4, AZ::Vector2(1.0f, 1.0f));
        // Slope rising along +x: height = x * 0.5
        for (AZ::u32 y = 0; y < 4; ++y)
        {
            for (AZ::u32 x = 0; x < 4; ++x)
            {
                provider.m_heights[y * 4 + x] = x * 0.5f;
            }
        }

        auto bodyHandle = CreateHeightfieldBody(provider);
        ASSERT_NE(m_lastHeightfieldShape, nullptr);
        ASSERT_NE(bodyHandle, AzPhysics::InvalidSimulatedBodyHandle);
        EXPECT_NE(m_scene->GetSimulatedBodyFromHandle(bodyHandle), nullptr);

        EXPECT_NEAR(RaycastDownZ(AZ::Vector3(1.5f, -1.5f, 10.0f)), 0.75f, 0.1f);
        EXPECT_NEAR(RaycastDownZ(AZ::Vector3(2.5f, -2.5f, 10.0f)), 1.25f, 0.1f);

    }

    TEST_F(JoltHeightfieldTests, SphereRollsDownSlope)
    {
        MockHeightfieldProvider provider(GetProviderEntityId(), 16, 16, AZ::Vector2(1.0f, 1.0f));
        // Slope descending along +x: height = (15 - x) * 0.3
        for (AZ::u32 y = 0; y < 16; ++y)
        {
            for (AZ::u32 x = 0; x < 16; ++x)
            {
                provider.m_heights[y * 16 + x] = (15 - x) * 0.3f;
            }
        }

        CreateHeightfieldBody(provider);
        auto sphere = CreateDynamicSphere(AZ::Vector3(12.0f, -8.0f, 6.0f), 0.5f);

        const float startX = 12.0f;
        SimulateSeconds(3.0f);

        // The slope descends along +x, so the sphere rolls downhill towards +x.
        const float endX = m_scene->GetSimulatedBodyFromHandle(sphere)->GetPosition().GetX();
        EXPECT_GT(endX, startX + 1.0f);
    }

    TEST_F(JoltHeightfieldTests, RuntimeHeightUpdateChangesCollision)
    {
        MockHeightfieldProvider provider(GetProviderEntityId(), 8, 8, AZ::Vector2(1.0f, 1.0f));
        auto bodyHandle = CreateHeightfieldBody(provider);

        // Everything flat at z=0.
        EXPECT_NEAR(RaycastDownZ(AZ::Vector3(4.0f, -4.0f, 10.0f)), 0.0f, 0.1f);

        // Raise the middle of the grid to z=5 through the shape's SetHeights API,
        // like the collider component does on provider notifications.
        auto* staticBody = static_cast<JoltStaticRigidBody*>(m_scene->GetSimulatedBodyFromHandle(bodyHandle));
        ASSERT_NE(staticBody, nullptr);

        auto* heightFieldShape = static_cast<JPH::HeightFieldShape*>(
            const_cast<JPH::Shape*>(m_lastHeightfieldShape.GetPtr()));
        ASSERT_NE(heightFieldShape, nullptr);

        const AZ::u32 sampleCount = heightFieldShape->GetSampleCount();
        AZStd::vector<float> newHeights(sampleCount * sampleCount, 0.0f);
        for (AZ::u32 y = 0; y < sampleCount; ++y)
        {
            for (AZ::u32 x = 0; x < sampleCount; ++x)
            {
                if (x >= 2 && x <= 5 && y >= 2 && y <= 5)
                {
                    newHeights[y * sampleCount + x] = 5.0f;
                }
            }
        }
        heightFieldShape->SetHeights(
            0, 0, sampleCount, sampleCount, newHeights.data(),
            static_cast<intptr_t>(sampleCount), *m_system->GetJoltAllocator());

        EXPECT_NEAR(RaycastDownZ(AZ::Vector3(3.5f, -3.5f, 10.0f)), 5.0f, 0.2f);
        EXPECT_NEAR(RaycastDownZ(AZ::Vector3(0.5f, -0.5f, 10.0f)), 0.0f, 0.2f);
    }

    TEST_F(JoltHeightfieldTests, PerTriangleMaterialsApply)
    {
        MockHeightfieldProvider provider(GetProviderEntityId(), 8, 8, AZ::Vector2(1.0f, 1.0f));

        // Left half (x < 4): restitution 0. Right half (x >= 4): restitution 0.9.
        provider.m_materials = {
            CreateMaterialAssetForTest(0.5f, 0.0f),
            CreateMaterialAssetForTest(0.5f, 0.9f),
        };
        for (AZ::u32 y = 0; y < 8; ++y)
        {
            for (AZ::u32 x = 0; x < 8; ++x)
            {
                provider.m_materialIndices[y * 8 + x] = x < 4 ? 0 : 1;
            }
        }

        CreateHeightfieldBody(provider);
        auto deadSphere = CreateDynamicSphere(AZ::Vector3(2.0f, -4.0f, 3.0f));
        auto bouncySphere = CreateDynamicSphere(AZ::Vector3(6.0f, -4.0f, 3.0f));

        const float fixedDeltaTime = 1.0f / 60.0f;
        float deadMaxAfterBounce = 0.0f;
        float bouncyMaxAfterBounce = 0.0f;
        bool deadTouched = false;
        bool bouncyTouched = false;

        for (int i = 0; i < 240; ++i)
        {
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();

            const float deadZ = m_scene->GetSimulatedBodyFromHandle(deadSphere)->GetPosition().GetZ();
            const float bouncyZ = m_scene->GetSimulatedBodyFromHandle(bouncySphere)->GetPosition().GetZ();

            if (deadZ < 0.55f) { deadTouched = true; }
            else if (deadTouched) { deadMaxAfterBounce = AZStd::max(deadMaxAfterBounce, deadZ); }

            if (bouncyZ < 0.55f) { bouncyTouched = true; }
            else if (bouncyTouched) { bouncyMaxAfterBounce = AZStd::max(bouncyMaxAfterBounce, bouncyZ); }
        }

        EXPECT_TRUE(deadTouched);
        EXPECT_TRUE(bouncyTouched);
        EXPECT_LT(deadMaxAfterBounce, 0.7f);
        EXPECT_GT(bouncyMaxAfterBounce, 1.0f);
    }

} // namespace JoltPhysics
