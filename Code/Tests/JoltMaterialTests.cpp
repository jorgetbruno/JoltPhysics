#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <AzCore/Asset/AssetManager.h>
#include <AzCore/Interface/Interface.h>

#include <Material/JoltMaterial.h>
#include <System/JoltSystem.h>
#include <Scene/JoltScene.h>
#include <Configuration/JoltSettingsRegistryManager.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/Material/PhysicsMaterialManager.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    class JoltMaterialTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "MaterialTestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);
        }

        void TearDown() override
        {
            m_system->RemoveScene(m_sceneHandle);
            m_system->Shutdown();
            m_system.reset();
        }

        AZ::Data::Asset<Physics::MaterialAsset> CreateMaterialAsset(float friction, float restitution)
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
    };

    TEST_F(JoltMaterialTests, DefaultMaterialHasExpectedValues)
    {
        auto* materialManager = AZ::Interface<Physics::MaterialManager>::Get();
        ASSERT_NE(materialManager, nullptr);

        auto defaultMaterial = materialManager->GetDefaultMaterial();
        ASSERT_NE(defaultMaterial, nullptr);
        EXPECT_TRUE(defaultMaterial->GetId().IsValid());
        EXPECT_EQ(defaultMaterial->GetProperty("DynamicFriction").GetValue<float>(), JoltMaterial::DefaultFriction);
        EXPECT_EQ(defaultMaterial->GetProperty("StaticFriction").GetValue<float>(), JoltMaterial::DefaultFriction);
        EXPECT_EQ(defaultMaterial->GetProperty("Restitution").GetValue<float>(), JoltMaterial::DefaultRestitution);
        EXPECT_EQ(defaultMaterial->GetProperty("Density").GetValue<float>(), JoltMaterial::DefaultDensity);
    }

    TEST_F(JoltMaterialTests, MaterialFromAssetProperties)
    {
        auto* materialManager = AZ::Interface<Physics::MaterialManager>::Get();
        ASSERT_NE(materialManager, nullptr);

        auto asset = CreateMaterialAsset(0.9f, 0.8f);
        auto material = materialManager->FindOrCreateMaterial(
            Physics::MaterialId::CreateFromAssetId(asset.GetId()), asset);
        ASSERT_NE(material, nullptr);

        auto* joltMaterial = azrtti_cast<JoltMaterial*>(material.get());
        ASSERT_NE(joltMaterial, nullptr);
        EXPECT_FLOAT_EQ(joltMaterial->GetDynamicFriction(), 0.9f);
        EXPECT_FLOAT_EQ(joltMaterial->GetRestitution(), 0.8f);

        // Same id resolves to the same material instance.
        auto sameMaterial = materialManager->GetMaterial(Physics::MaterialId::CreateFromAssetId(asset.GetId()));
        EXPECT_EQ(sameMaterial.get(), material.get());
    }

    TEST_F(JoltMaterialTests, BouncySphereBouncesHigherThanDeadSphere)
    {
        // Static slab, default material.
        auto slabCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto slabShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        slabShape->m_dimensions = AZ::Vector3(20.0f, 20.0f, 1.0f);
        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(slabCollider, slabShape);
        m_scene->AddSimulatedBody(&slabConfig);

        auto createSphere = [this](float x, const AZ::Data::Asset<Physics::MaterialAsset>& materialAsset)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            if (materialAsset.GetId().IsValid())
            {
                colliderConfig->m_materialSlots.SetMaterialAsset(0, materialAsset);
            }
            auto sphereShape = AZStd::make_shared<Physics::SphereShapeConfiguration>();
            sphereShape->m_radius = 0.5f;

            AzPhysics::RigidBodyConfiguration sphereConfig;
            sphereConfig.m_position = AZ::Vector3(x, 0.0f, 3.0f);
            sphereConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, sphereShape);
            return m_scene->AddSimulatedBody(&sphereConfig);
        };

        auto deadSphere = createSphere(-2.0f, {});
        auto bouncySphere = createSphere(2.0f, CreateMaterialAsset(0.5f, 0.9f));

        // Simulate, recording the maximum height of each sphere after its first contact.
        const float fixedDeltaTime = 1.0f / 60.0f;
        float deadMaxAfterBounce = 0.0f;
        float bouncyMaxAfterBounce = 0.0f;
        bool deadTouched = false;
        bool bouncyTouched = false;

        for (int i = 0; i < 240; ++i) // 4 seconds
        {
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();

            const float deadZ = m_scene->GetSimulatedBodyFromHandle(deadSphere)->GetPosition().GetZ();
            const float bouncyZ = m_scene->GetSimulatedBodyFromHandle(bouncySphere)->GetPosition().GetZ();

            if (deadZ < 0.55f)
            {
                deadTouched = true;
            }
            else if (deadTouched)
            {
                deadMaxAfterBounce = AZStd::max(deadMaxAfterBounce, deadZ);
            }

            if (bouncyZ < 0.55f)
            {
                bouncyTouched = true;
            }
            else if (bouncyTouched)
            {
                bouncyMaxAfterBounce = AZStd::max(bouncyMaxAfterBounce, bouncyZ);
            }
        }

        EXPECT_TRUE(deadTouched);
        EXPECT_TRUE(bouncyTouched);
        EXPECT_LT(deadMaxAfterBounce, 0.7f);
        EXPECT_GT(bouncyMaxAfterBounce, 1.2f);
    }

} // namespace JoltPhysics
