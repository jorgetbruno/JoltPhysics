#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <AzCore/Asset/AssetManager.h>
#include <AzCore/Interface/Interface.h>

#include <Material/JoltMaterial.h>
#include <Material/JoltMaterialManager.h>
#include <RigidBody/JoltStaticRigidBody.h>
#include <Shape/JoltMeshUtils.h>
#include <Shape/JoltShapeUtils.h>
#include <System/JoltSystem.h>
#include <Scene/JoltScene.h>
#include <Configuration/JoltSettingsRegistryManager.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/Material/PhysicsMaterialManager.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
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

    TEST_F(JoltMaterialTests, CompoundSubShapeMaterialsApplyPerCollider)
    {
        // Static compound slab: dead half (restitution 0) at x=-2, bouncy half (restitution 0.9) at x=+2.
        auto makeSlabCollider = [this](float offsetX, float restitution)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            colliderConfig->m_position = AZ::Vector3(offsetX, 0.0f, -0.5f);
            if (restitution >= 0.0f)
            {
                colliderConfig->m_materialSlots.SetMaterialAsset(0, CreateMaterialAsset(0.5f, restitution));
            }
            auto shapeConfig = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            shapeConfig->m_dimensions = AZ::Vector3(4.0f, 4.0f, 1.0f);
            return AzPhysics::ShapeColliderPair(colliderConfig, shapeConfig);
        };

        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPairList{
            makeSlabCollider(-2.0f, 0.0f),
            makeSlabCollider(2.0f, 0.9f),
        };
        auto slabHandle = m_scene->AddSimulatedBody(&slabConfig);

        auto* slabBody = static_cast<JoltStaticRigidBody*>(m_scene->GetSimulatedBodyFromHandle(slabHandle));
        ASSERT_NE(slabBody, nullptr);
        ASSERT_EQ(slabBody->GetColliderCount(), 2u);
        EXPECT_FLOAT_EQ(JoltMaterialManager::GetFrictionRestitution(slabBody->GetColliderMaterial(0).get()).second, 0.0f);
        EXPECT_FLOAT_EQ(JoltMaterialManager::GetFrictionRestitution(slabBody->GetColliderMaterial(1).get()).second, 0.9f);

        auto dropSphere = [this](float x)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto sphereShape = AZStd::make_shared<Physics::SphereShapeConfiguration>();
            sphereShape->m_radius = 0.5f;
            AzPhysics::RigidBodyConfiguration sphereConfig;
            sphereConfig.m_position = AZ::Vector3(x, 0.0f, 3.0f);
            sphereConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, sphereShape);
            return m_scene->AddSimulatedBody(&sphereConfig);
        };

        auto deadSphere = dropSphere(-2.0f);
        auto bouncySphere = dropSphere(2.0f);

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

    TEST_F(JoltMaterialTests, HullGroupSubShapesUseTheSingleCollidersMaterial)
    {
        // ONE collider whose baked shape is a hull GROUP (two boxes apart on X): the
        // compound's children are not per-collider children, so both must resolve the
        // one collider's material (child index 1 used to run off the collider list and
        // get default material instead).
        auto boxPoints = [](float centerX)
        {
            AZStd::vector<AZ::Vector3> points;
            for (int i = 0; i < 8; ++i)
            {
                points.emplace_back(
                    centerX + ((i & 1) ? 1.5f : -1.5f),
                    (i & 2) ? 2.0f : -2.0f,
                    -0.5f + ((i & 4) ? 0.5f : -0.5f));
            }
            return points;
        };
        const AZStd::vector<AZ::u8> blob = JoltMeshUtils::PackConvexHulls({ boxPoints(-4.0f), boxPoints(4.0f) });
        ASSERT_FALSE(blob.empty());

        auto slabShape = AZStd::make_shared<Physics::CookedMeshShapeConfiguration>();
        slabShape->SetCookedMeshData(blob.data(), blob.size(), Physics::CookedMeshShapeConfiguration::MeshType::Convex);

        auto slabCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        slabCollider->m_materialSlots.SetMaterialAsset(0, CreateMaterialAsset(0.5f, 0.9f));
        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(slabCollider, slabShape);
        m_scene->AddSimulatedBody(&slabConfig);

        auto dropSphere = [this](float x)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto sphereShape = AZStd::make_shared<Physics::SphereShapeConfiguration>();
            sphereShape->m_radius = 0.5f;
            AzPhysics::RigidBodyConfiguration sphereConfig;
            sphereConfig.m_position = AZ::Vector3(x, 0.0f, 3.0f);
            sphereConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, sphereShape);
            return m_scene->AddSimulatedBody(&sphereConfig);
        };

        // One sphere per hull; both sit on the bouncy material, so both bounce high.
        auto leftSphere = dropSphere(-4.0f);
        auto rightSphere = dropSphere(4.0f);

        const float fixedDeltaTime = 1.0f / 60.0f;
        float leftMaxAfterBounce = 0.0f;
        float rightMaxAfterBounce = 0.0f;
        bool leftTouched = false;
        bool rightTouched = false;

        for (int i = 0; i < 240; ++i)
        {
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();

            const float leftZ = m_scene->GetSimulatedBodyFromHandle(leftSphere)->GetPosition().GetZ();
            const float rightZ = m_scene->GetSimulatedBodyFromHandle(rightSphere)->GetPosition().GetZ();

            if (leftZ < 0.55f)
            {
                leftTouched = true;
            }
            else if (leftTouched)
            {
                leftMaxAfterBounce = AZStd::max(leftMaxAfterBounce, leftZ);
            }

            if (rightZ < 0.55f)
            {
                rightTouched = true;
            }
            else if (rightTouched)
            {
                rightMaxAfterBounce = AZStd::max(rightMaxAfterBounce, rightZ);
            }
        }

        EXPECT_TRUE(leftTouched);
        EXPECT_TRUE(rightTouched);
        EXPECT_GT(leftMaxAfterBounce, 1.2f);
        EXPECT_GT(rightMaxAfterBounce, 1.2f);

        // Balance the AddRef that creating the cooked mesh shape took on the configuration.
        if (auto* cachedMesh = static_cast<JPH::Shape*>(slabShape->GetCachedNativeMesh()))
        {
            cachedMesh->Release();
            slabShape->SetCachedNativeMesh(nullptr);
        }
    }

    TEST_F(JoltMaterialTests, StandaloneShapeMaterialRoundTrip)
    {
        auto* materialManager = AZ::Interface<Physics::MaterialManager>::Get();
        ASSERT_NE(materialManager, nullptr);

        // The shape resolves its material from the collider's first material slot.
        auto asset = CreateMaterialAsset(0.7f, 0.8f);
        Physics::ColliderConfiguration colliderConfig;
        colliderConfig.m_materialSlots.SetMaterialAsset(0, asset);
        Physics::SphereShapeConfiguration shapeConfig(0.5f);

        AZStd::shared_ptr<Physics::Shape> shape = JoltShapeUtils::CreateShape(colliderConfig, shapeConfig);
        ASSERT_NE(shape, nullptr);
        ASSERT_NE(shape->GetMaterial(), nullptr);
        EXPECT_EQ(shape->GetMaterialId(), Physics::MaterialId::CreateFromAssetId(asset.GetId()));
        EXPECT_FLOAT_EQ(shape->GetMaterial()->GetProperty("Restitution").GetValue<float>(), 0.8f);

        // SetMaterial replaces it.
        auto defaultMaterial = materialManager->GetDefaultMaterial();
        shape->SetMaterial(defaultMaterial);
        EXPECT_EQ(shape->GetMaterial().get(), defaultMaterial.get());
        EXPECT_EQ(shape->GetMaterialId(), defaultMaterial->GetId());
    }

    TEST_F(JoltMaterialTests, PrebuiltShapeBodyUsesShapeMaterial)
    {
        auto* materialManager = AZ::Interface<Physics::MaterialManager>::Get();
        ASSERT_NE(materialManager, nullptr);

        // Static slab built from a prebuilt Physics::Shape carrying a bouncy material.
        Physics::ColliderConfiguration slabCollider;
        Physics::BoxShapeConfiguration slabShape(AZ::Vector3(20.0f, 20.0f, 1.0f));
        AZStd::shared_ptr<Physics::Shape> shape = JoltShapeUtils::CreateShape(slabCollider, slabShape);
        ASSERT_NE(shape, nullptr);

        auto bouncyAsset = CreateMaterialAsset(0.5f, 0.9f);
        auto bouncyMaterial = materialManager->FindOrCreateMaterial(
            Physics::MaterialId::CreateFromAssetId(bouncyAsset.GetId()), bouncyAsset);
        shape->SetMaterial(bouncyMaterial);

        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        slabConfig.m_colliderAndShapeData = shape;
        auto slabHandle = m_scene->AddSimulatedBody(&slabConfig);

        auto* slabBody = azdynamic_cast<JoltStaticRigidBody*>(m_scene->GetSimulatedBodyFromHandle(slabHandle));
        ASSERT_NE(slabBody, nullptr);
        ASSERT_EQ(slabBody->GetColliderCount(), 1u);
        EXPECT_EQ(slabBody->GetColliderMaterial(0).get(), bouncyMaterial.get());

        // A default (dead) sphere dropped onto the bouncy slab rebounds (restitution
        // combines as max of the two materials).
        auto sphereCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto sphereShape = AZStd::make_shared<Physics::SphereShapeConfiguration>(0.5f);
        AzPhysics::RigidBodyConfiguration sphereConfig;
        sphereConfig.m_position = AZ::Vector3(0.0f, 0.0f, 3.0f);
        sphereConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(sphereCollider, sphereShape);
        auto sphereHandle = m_scene->AddSimulatedBody(&sphereConfig);

        const float fixedDeltaTime = 1.0f / 60.0f;
        float maxAfterBounce = 0.0f;
        bool touched = false;
        for (int i = 0; i < 240; ++i)
        {
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();

            const float z = m_scene->GetSimulatedBodyFromHandle(sphereHandle)->GetPosition().GetZ();
            if (z < 0.55f)
            {
                touched = true;
            }
            else if (touched)
            {
                maxAfterBounce = AZStd::max(maxAfterBounce, z);
            }
        }

        EXPECT_TRUE(touched);
        EXPECT_GT(maxAfterBounce, 1.2f);
    }

    TEST_F(JoltMaterialTests, MaterialPropertyChangeAppliesToExistingBody)
    {
        auto* materialManager = AZ::Interface<Physics::MaterialManager>::Get();
        ASSERT_NE(materialManager, nullptr);

        // Slab with a dead material (restitution 0).
        auto slabAsset = CreateMaterialAsset(0.5f, 0.0f);
        auto slabCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        slabCollider->m_materialSlots.SetMaterialAsset(0, slabAsset);
        auto slabShape = AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(20.0f, 20.0f, 1.0f));
        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(slabCollider, slabShape);
        m_scene->AddSimulatedBody(&slabConfig);

        auto sphereCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto sphereShape = AZStd::make_shared<Physics::SphereShapeConfiguration>(0.5f);
        AzPhysics::RigidBodyConfiguration sphereConfig;
        sphereConfig.m_position = AZ::Vector3(0.0f, 0.0f, 3.0f);
        sphereConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(sphereCollider, sphereShape);
        auto sphereHandle = m_scene->AddSimulatedBody(&sphereConfig);

        const float fixedDeltaTime = 1.0f / 60.0f;
        auto measureBounce = [&]()
        {
            float maxAfterBounce = 0.0f;
            bool touched = false;
            for (int i = 0; i < 240; ++i)
            {
                m_scene->StartSimulation(fixedDeltaTime);
                m_scene->FinishSimulation();

                const float z = m_scene->GetSimulatedBodyFromHandle(sphereHandle)->GetPosition().GetZ();
                if (z < 0.55f)
                {
                    touched = true;
                }
                else if (touched)
                {
                    maxAfterBounce = AZStd::max(maxAfterBounce, z);
                }
            }
            EXPECT_TRUE(touched);
            return maxAfterBounce;
        };

        // Dead first drop.
        EXPECT_LT(measureBounce(), 0.7f);

        // Make the slab's material bouncy at runtime - no body is recreated - and
        // drop the same sphere again: existing bodies pick up the new value.
        auto slabMaterial = materialManager->GetMaterial(Physics::MaterialId::CreateFromAssetId(slabAsset.GetId()));
        ASSERT_NE(slabMaterial, nullptr);
        slabMaterial->SetProperty("Restitution", 0.9f);

        auto* sphereBody = azdynamic_cast<AzPhysics::RigidBody*>(m_scene->GetSimulatedBodyFromHandle(sphereHandle));
        ASSERT_NE(sphereBody, nullptr);
        sphereBody->SetTransform(AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 3.0f)));
        sphereBody->SetLinearVelocity(AZ::Vector3::CreateZero());
        sphereBody->SetAngularVelocity(AZ::Vector3::CreateZero());
        sphereBody->ForceAwake();

        EXPECT_GT(measureBounce(), 1.2f);
    }

    TEST_F(JoltMaterialTests, PerFaceTrimeshMaterialsApply)
    {
        // One trimesh whose left half maps to a dead slot and its right half to a
        // bouncy one: the per-face table baked into the blob (not the collider's
        // single material) decides how each contact point responds.
        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        auto appendQuad = [&vertices, &indices](float centerX, float halfExtent)
        {
            const AZ::u32 base = static_cast<AZ::u32>(vertices.size());
            vertices.emplace_back(centerX - halfExtent, -halfExtent, 0.0f);
            vertices.emplace_back(centerX + halfExtent, -halfExtent, 0.0f);
            vertices.emplace_back(centerX + halfExtent, halfExtent, 0.0f);
            vertices.emplace_back(centerX - halfExtent, halfExtent, 0.0f);
            indices.insert(indices.end(), { base, base + 1, base + 2, base, base + 2, base + 3 });
        };
        appendQuad(-4.0f, 1.5f);
        appendQuad(4.0f, 1.5f);
        const AZ::u8 perFaceMaterials[4] = { 0, 0, 1, 1 };

        const AZStd::vector<AZ::u8> blob = JoltMeshUtils::PackTriangleMesh(
            vertices.data(), static_cast<AZ::u32>(vertices.size()),
            indices.data(), static_cast<AZ::u32>(indices.size()),
            perFaceMaterials, 4);
        ASSERT_FALSE(blob.empty());

        auto slabShape = AZStd::make_shared<Physics::CookedMeshShapeConfiguration>();
        slabShape->SetCookedMeshData(blob.data(), blob.size(), Physics::CookedMeshShapeConfiguration::MeshType::TriangleMesh);

        auto slabCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        slabCollider->m_materialSlots.SetSlots({ "dead", "bouncy" });
        slabCollider->m_materialSlots.SetMaterialAsset(0, CreateMaterialAsset(0.5f, 0.0f));
        slabCollider->m_materialSlots.SetMaterialAsset(1, CreateMaterialAsset(0.5f, 0.9f));
        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(slabCollider, slabShape);
        m_scene->AddSimulatedBody(&slabConfig);

        auto dropSphere = [this](float x)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto sphereShape = AZStd::make_shared<Physics::SphereShapeConfiguration>();
            sphereShape->m_radius = 0.5f;
            AzPhysics::RigidBodyConfiguration sphereConfig;
            sphereConfig.m_position = AZ::Vector3(x, 0.0f, 3.0f);
            sphereConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, sphereShape);
            return m_scene->AddSimulatedBody(&sphereConfig);
        };

        auto deadSphere = dropSphere(-4.0f);
        auto bouncySphere = dropSphere(4.0f);

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

        // Balance the AddRef that creating the cooked mesh shape took on the configuration.
        if (auto* cachedMesh = static_cast<JPH::Shape*>(slabShape->GetCachedNativeMesh()))
        {
            cachedMesh->Release();
            slabShape->SetCachedNativeMesh(nullptr);
        }
    }

    TEST_F(JoltMaterialTests, PerFaceMaterialsApplyThroughCompounds)
    {
        // A compound body whose first child is a trimesh with a per-face table (every
        // face on slot 1, the bouncy material): the compound's sub-shape mapping has to
        // reach the mesh's table through the nesting, not just the collider's material.
        const AZ::Vector3 quadVertices[4] = {
            AZ::Vector3(-10.0f, -10.0f, 0.0f), AZ::Vector3(10.0f, -10.0f, 0.0f),
            AZ::Vector3(10.0f, 10.0f, 0.0f),   AZ::Vector3(-10.0f, 10.0f, 0.0f),
        };
        const AZ::u32 quadIndices[6] = { 0, 1, 2, 0, 2, 3 };
        const AZ::u8 perFaceMaterials[2] = { 1, 1 };
        const AZStd::vector<AZ::u8> blob = JoltMeshUtils::PackTriangleMesh(
            quadVertices, 4, quadIndices, 6, perFaceMaterials, 2);

        auto triShape = AZStd::make_shared<Physics::CookedMeshShapeConfiguration>();
        triShape->SetCookedMeshData(blob.data(), blob.size(), Physics::CookedMeshShapeConfiguration::MeshType::TriangleMesh);
        auto triCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        triCollider->m_materialSlots.SetSlots({ "dead", "bouncy" });
        triCollider->m_materialSlots.SetMaterialAsset(0, CreateMaterialAsset(0.5f, 0.0f));
        triCollider->m_materialSlots.SetMaterialAsset(1, CreateMaterialAsset(0.5f, 0.9f));

        // Second child: a plain dead box far from the trimesh.
        auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        boxShape->m_dimensions = AZ::Vector3(4.0f, 4.0f, 1.0f);
        auto boxCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        boxCollider->m_position = AZ::Vector3(50.0f, 0.0f, -0.5f);

        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPairList{
            AzPhysics::ShapeColliderPair(triCollider, triShape),
            AzPhysics::ShapeColliderPair(boxCollider, boxShape),
        };
        m_scene->AddSimulatedBody(&slabConfig);

        auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto sphereShape = AZStd::make_shared<Physics::SphereShapeConfiguration>();
        sphereShape->m_radius = 0.5f;
        AzPhysics::RigidBodyConfiguration sphereConfig;
        sphereConfig.m_position = AZ::Vector3(0.0f, 0.0f, 3.0f);
        sphereConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, sphereShape);
        auto sphereHandle = m_scene->AddSimulatedBody(&sphereConfig);

        const float fixedDeltaTime = 1.0f / 60.0f;
        float maxAfterBounce = 0.0f;
        bool touched = false;
        for (int i = 0; i < 240; ++i)
        {
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();

            const float sphereZ = m_scene->GetSimulatedBodyFromHandle(sphereHandle)->GetPosition().GetZ();
            if (sphereZ < 0.55f)
            {
                touched = true;
            }
            else if (touched)
            {
                maxAfterBounce = AZStd::max(maxAfterBounce, sphereZ);
            }
        }

        EXPECT_TRUE(touched);
        EXPECT_GT(maxAfterBounce, 1.2f);

        // Balance the AddRef that creating the cooked mesh shape took on the configuration.
        if (auto* cachedMesh = static_cast<JPH::Shape*>(triShape->GetCachedNativeMesh()))
        {
            cachedMesh->Release();
            triShape->SetCachedNativeMesh(nullptr);
        }
    }

    TEST_F(JoltMaterialTests, PerColliderMaterialsSurviveACentreOfMassOffset)
    {
        // Setting a centre-of-mass offset wraps the whole body shape in a
        // RotatedTranslatedShape. The material lookup tested that *wrapped* shape for a
        // compound subtype, so an offset body read as a plain decorator, missed the
        // compound branch and gave every contact collider 0's material - silently undoing
        // per-collider and per-face materials for anyone who nudged a prop's balance
        // point. Scene queries kept naming the right collider, so the two disagreed.
        //
        // The falling body is the compound: collider 0 is dead and rides high where it
        // never touches, collider 1 is bouncy and is the one that lands. Reading collider
        // 0 by mistake is the difference between a bounce and a thud.
        auto floorCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        floorCollider->m_materialSlots.SetSlots({ "floor" });
        floorCollider->m_materialSlots.SetMaterialAsset(0, CreateMaterialAsset(0.5f, 0.0f));
        auto floorShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        floorShape->m_dimensions = AZ::Vector3(40.0f, 40.0f, 1.0f);

        AzPhysics::StaticRigidBodyConfiguration floorConfig;
        floorConfig.m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        floorConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(floorCollider, floorShape);
        m_scene->AddSimulatedBody(&floorConfig);

        auto deadCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        deadCollider->m_position = AZ::Vector3(0.0f, 0.0f, 3.0f); // never reaches the floor
        deadCollider->m_materialSlots.SetSlots({ "dead" });
        deadCollider->m_materialSlots.SetMaterialAsset(0, CreateMaterialAsset(0.5f, 0.0f));
        auto deadShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        deadShape->m_dimensions = AZ::Vector3(0.5f, 0.5f, 0.5f);

        auto bouncyCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        bouncyCollider->m_materialSlots.SetSlots({ "bouncy" });
        bouncyCollider->m_materialSlots.SetMaterialAsset(0, CreateMaterialAsset(0.5f, 0.95f));
        auto bouncyShape = AZStd::make_shared<Physics::SphereShapeConfiguration>();
        bouncyShape->m_radius = 0.5f;

        AzPhysics::RigidBodyConfiguration fallerConfig;
        fallerConfig.m_position = AZ::Vector3(0.0f, 0.0f, 4.0f);
        // The offset is what used to cost this body its per-collider materials.
        fallerConfig.m_centerOfMassOffset = AZ::Vector3(0.0f, 0.0f, 0.25f);
        fallerConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPairList{
            AzPhysics::ShapeColliderPair(bouncyCollider, bouncyShape),
            AzPhysics::ShapeColliderPair(deadCollider, deadShape),
        };
        auto fallerHandle = m_scene->AddSimulatedBody(&fallerConfig);

        const float fixedDeltaTime = 1.0f / 60.0f;
        float lowestZ = 1000.0f;
        float maxAfterBounce = 0.0f;
        bool touched = false;
        for (int i = 0; i < 300; ++i)
        {
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();

            const float z = m_scene->GetSimulatedBodyFromHandle(fallerHandle)->GetPosition().GetZ();
            lowestZ = AZStd::min(lowestZ, z);
            // The sphere's radius is 0.5, so anything near the floor counts as arrival -
            // a good bounce turns around before it ever settles.
            if (z < 1.0f)
            {
                touched = true;
            }
            else if (touched)
            {
                maxAfterBounce = AZStd::max(maxAfterBounce, z);
            }
        }

        ASSERT_TRUE(touched) << "the body never reached the floor; lowest z was " << lowestZ;
        EXPECT_GT(maxAfterBounce, 1.5f)
            << "the touching collider's bouncy material did not reach the contact - it bounced to "
            << maxAfterBounce;
    }

} // namespace JoltPhysics
