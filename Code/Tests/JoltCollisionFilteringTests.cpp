#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <Clients/Components/JoltRigidBodyComponent.h>
#include <Clients/Components/JoltBoxColliderComponent.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Physics/CollisionBus.h>
#include <JoltTestWarningCatcher.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <Character/JoltCharacter.h>
#include <System/JoltSystem.h>
#include <Scene/JoltScene.h>
#include <Configuration/JoltSettingsRegistryManager.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/Character.h>
#include <AzFramework/Physics/Ragdoll.h>
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

        //! Capsule character on the given collision layer and group. Driven downwards
        //! each tick, because a character moves only when asked to.
        AzPhysics::SimulatedBodyHandle CreateCharacterOnLayer(
            AZ::u8 collisionLayer, const AZStd::string& collisionGroupName, AZ::u64 collisionGroupMask,
            bool rigidBodyBacked)
        {
            JoltCharacterConfiguration config;
            config.m_position = AZ::Vector3(0.0f, 0.0f, 2.0f);
            config.m_debugName = "FilteringCharacter";
            config.m_collisionLayer = AzPhysics::CollisionLayer(collisionLayer);
            config.m_collisionGroupId =
                m_system->CreateCollisionGroupPreset(collisionGroupName, AzPhysics::CollisionGroup(collisionGroupMask));
            config.m_shapeConfig = AZStd::make_shared<Physics::CapsuleShapeConfiguration>(1.8f, 0.3f);
            config.m_rigidBodyCharacter = rigidBodyBacked;
            return m_scene->AddSimulatedBody(&config);
        }

        //! Walks a character straight down for the given time, one request per tick.
        void DriveCharacterDown(AzPhysics::SimulatedBodyHandle handle, float seconds)
        {
            auto* character = azdynamic_cast<Physics::Character*>(m_scene->GetSimulatedBodyFromHandle(handle));
            ASSERT_NE(character, nullptr);

            const float fixedDeltaTime = 1.0f / 60.0f;
            const int steps = static_cast<int>(seconds / fixedDeltaTime);
            for (int i = 0; i < steps; ++i)
            {
                character->AddVelocityForTick(AZ::Vector3(0.0f, 0.0f, -4.0f));
                m_scene->StartSimulation(fixedDeltaTime);
                m_scene->FinishSimulation();
            }
        }

        //! Two-node ragdoll whose colliders use the given collision layer and group.
        AzPhysics::SimulatedBodyHandle CreateRagdollOnLayer(
            AZ::u8 collisionLayer, const AZStd::string& collisionGroupName, AZ::u64 collisionGroupMask);

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

    //! Builds a two-node ragdoll whose colliders sit on the given collision layer and
    //! group, dropped from z=3.
    AzPhysics::SimulatedBodyHandle JoltCollisionFilteringTests::CreateRagdollOnLayer(
        AZ::u8 collisionLayer, const AZStd::string& collisionGroupName, AZ::u64 collisionGroupMask)
    {
        Physics::RagdollConfiguration config;
        const size_t noParent = static_cast<size_t>(-1);

        for (int i = 0; i < 2; ++i)
        {
            const char* name = (i == 0) ? "root" : "child";

            Physics::RagdollNodeConfiguration node;
            node.m_debugName = name;
            node.m_mass = 1.0f;
            config.m_nodes.push_back(node);
            config.m_parentIndices.push_back(i == 0 ? noParent : 0);

            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            colliderConfig->m_collisionLayer = AzPhysics::CollisionLayer(collisionLayer);
            colliderConfig->m_collisionGroupId = m_system->CreateCollisionGroupPreset(
                collisionGroupName, AzPhysics::CollisionGroup(collisionGroupMask));

            Physics::CharacterColliderNodeConfiguration collider;
            collider.m_name = name;
            collider.m_shapes.emplace_back(colliderConfig, AZStd::make_shared<Physics::SphereShapeConfiguration>(0.15f));
            config.m_colliders.m_nodes.push_back(collider);

            Physics::RagdollNodeState state;
            state.m_position = AZ::Vector3(0.0f, 0.0f, 3.0f - i * 0.5f);
            config.m_initialState.push_back(state);
        }

        auto handle = m_scene->AddSimulatedBody(&config);
        if (auto* ragdoll = azdynamic_cast<Physics::Ragdoll*>(m_scene->GetSimulatedBodyFromHandle(handle)))
        {
            ragdoll->EnableSimulation(config.m_initialState);
        }
        return handle;
    }

    // A CharacterVirtual sweeps its own movement instead of going through the
    // simulation, so its collision layer only reaches that sweep if the filters are
    // handed to ExtendedUpdate. These cover both backends so the two agree.

    TEST_F(JoltCollisionFilteringTests, VirtualCharacterStandsOnASlabItsGroupIncludes)
    {
        CreateSlab("SlabGroup", /*mask*/ 0b11);
        auto character = CreateCharacterOnLayer(/*layer*/ 1, "CharacterGroup", /*mask*/ 0b11, false);

        DriveCharacterDown(character, 1.5f);

        // Base rests on the slab surface at z=0.
        EXPECT_NEAR(GetBodyZ(character), 0.0f, 0.1f);
    }

    TEST_F(JoltCollisionFilteringTests, VirtualCharacterFallsThroughWhenItsGroupExcludesTheSlabLayer)
    {
        CreateSlab("SlabGroup", /*mask*/ 0b11, /*layer*/ 0);
        // Collides with layer 1 only, so the slab on layer 0 must not stop it.
        auto character = CreateCharacterOnLayer(/*layer*/ 1, "CharacterGroup", /*mask*/ 0b10, false);

        DriveCharacterDown(character, 1.5f);

        EXPECT_LT(GetBodyZ(character), -2.0f);
    }

    TEST_F(JoltCollisionFilteringTests, VirtualCharacterFallsThroughWhenTheSlabGroupExcludesItsLayer)
    {
        CreateSlab("SlabGroup", /*mask*/ 0b01); // contains only layer 0
        auto character = CreateCharacterOnLayer(/*layer*/ 1, "CharacterGroup", /*mask*/ 0b11, false);

        DriveCharacterDown(character, 1.5f);

        EXPECT_LT(GetBodyZ(character), -2.0f);
    }

    TEST_F(JoltCollisionFilteringTests, RigidBodyCharacterRespectsTheSameFiltering)
    {
        CreateSlab("SlabGroup", /*mask*/ 0b11, /*layer*/ 0);
        auto character = CreateCharacterOnLayer(/*layer*/ 1, "CharacterGroup", /*mask*/ 0b10, true);

        DriveCharacterDown(character, 1.5f);

        EXPECT_LT(GetBodyZ(character), -2.0f);
    }

    TEST_F(JoltCollisionFilteringTests, RagdollPartsRespectTheirCollisionLayer)
    {
        // The slab only collides with layer 0; the ragdoll is on layer 1, so it falls
        // through. Ragdoll parts carry Jolt's own collision group (for parent/child
        // filtering), so this only works because layer filtering lives in the object
        // layer rather than in the collision group.
        CreateSlab("SlabGroup", 0b01 /* layer 0 only */);
        auto ragdoll = CreateRagdollOnLayer(1, "RagdollGroup", 0xFFFFFFFFFFFFFFFFull);

        SimulateSeconds(2.0f);

        Physics::RagdollState state;
        azdynamic_cast<Physics::Ragdoll*>(m_scene->GetSimulatedBodyFromHandle(ragdoll))->GetState(state);
        EXPECT_LT(state[0].m_position.GetZ(), -1.0f);
    }

    TEST_F(JoltCollisionFilteringTests, RagdollLandsWhenItsLayerIsIncluded)
    {
        // Same setup with the ragdoll on the layer the slab does collide with.
        CreateSlab("SlabGroup", 0b11 /* layers 0 and 1 */);
        auto ragdoll = CreateRagdollOnLayer(1, "RagdollGroup", 0xFFFFFFFFFFFFFFFFull);

        SimulateSeconds(2.0f);

        Physics::RagdollState state;
        azdynamic_cast<Physics::Ragdoll*>(m_scene->GetSimulatedBodyFromHandle(ragdoll))->GetState(state);
        EXPECT_GT(state[1].m_position.GetZ(), 0.0f);
    }

    TEST_F(JoltCollisionFilteringTests, RagdollPartsStillDoNotCollideWithEachOther)
    {
        // Layer filtering must not have displaced Jolt's own use of the collision group:
        // the parent and child spheres overlap at creation (0.5 m apart, 0.15 m radius is
        // not enough to separate them once the joint pulls them together) and must not
        // push each other apart.
        CreateSlab("SlabGroup", 0xFFFFFFFFFFFFFFFFull);
        auto ragdoll = CreateRagdollOnLayer(0, "RagdollGroup", 0xFFFFFFFFFFFFFFFFull);

        SimulateSeconds(2.0f);

        Physics::RagdollState state;
        azdynamic_cast<Physics::Ragdoll*>(m_scene->GetSimulatedBodyFromHandle(ragdoll))->GetState(state);
        // The two nodes stay roughly their original distance apart rather than being
        // shoved apart by a contact between them.
        const float separation = state[0].m_position.GetDistance(state[1].m_position);
        EXPECT_LT(separation, 0.8f);
    }

    TEST_F(JoltCollisionFilteringTests, ChangingACollidersLayerThroughTheBusMovesTheLiveBody)
    {
        // Gameplay-driven filtering - phasing through enemies mid-dodge, dropping a corpse
        // to a no-player layer - is scripted through CollisionFilteringRequestBus in PhysX
        // projects. The gem had no handler at all, so every call reported success and
        // changed nothing, which is the worst of both worlds: no effect and no warning.
        auto entity = AZStd::make_unique<AZ::Entity>("FilteredCollider");
        entity->CreateComponent<AzFramework::TransformComponent>();
        entity->CreateComponent<JoltBoxColliderComponent>();
        entity->CreateComponent<JoltRigidBodyComponent>();
        entity->Init();
        entity->Activate();

        AZStd::string layerName;
        Physics::CollisionFilteringRequestBus::EventResult(
            layerName, entity->GetId(), &Physics::CollisionFilteringRequests::GetCollisionLayerName);
        EXPECT_STREQ(layerName.c_str(), "Default") << "the collider should start on the default layer";

        // Name a second layer in the gem's configuration, then move the collider onto it.
        JoltSystemConfiguration config = m_system->GetJoltConfiguration();
        config.m_collisionConfig.m_collisionLayers.SetName(1, "Phasing");
        m_system->UpdateConfiguration(&config);

        Physics::CollisionFilteringRequestBus::Event(
            entity->GetId(), &Physics::CollisionFilteringRequests::SetCollisionLayer,
            AZStd::string("Phasing"), AZ::Crc32());

        Physics::CollisionFilteringRequestBus::EventResult(
            layerName, entity->GetId(), &Physics::CollisionFilteringRequests::GetCollisionLayerName);
        EXPECT_STREQ(layerName.c_str(), "Phasing") << "the collider's layer did not change";

        entity->Deactivate();
    }

    TEST_F(JoltCollisionFilteringTests, AnUnknownLayerNameIsRefusedOutLoud)
    {
        auto entity = AZStd::make_unique<AZ::Entity>("FilteredCollider");
        entity->CreateComponent<AzFramework::TransformComponent>();
        entity->CreateComponent<JoltBoxColliderComponent>();
        entity->Init();
        entity->Activate();

        JoltWarningCatcher warnings;
        Physics::CollisionFilteringRequestBus::Event(
            entity->GetId(), &Physics::CollisionFilteringRequests::SetCollisionLayer,
            AZStd::string("NoSuchLayer"), AZ::Crc32());

        EXPECT_TRUE(warnings.ContainsWarningWith("no collision layer named"))
            << "a misspelt layer name should say so rather than silently doing nothing";

        entity->Deactivate();
    }

} // namespace JoltPhysics
