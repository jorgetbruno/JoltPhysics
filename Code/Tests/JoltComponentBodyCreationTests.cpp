#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <Clients/Components/JoltBoxColliderComponent.h>
#include <Clients/Components/JoltCapsuleColliderComponent.h>
#include <Clients/Components/JoltRigidBodyComponent.h>
#include <Clients/Components/JoltStaticCompoundColliderComponent.h>
#include <System/JoltSystem.h>
#include <Scene/JoltScene.h>
#include <Configuration/JoltSettingsRegistryManager.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Components/NonUniformScaleComponent.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Physics/SystemBus.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    // Exercises the full component path the editor uses: entity with collider +
    // rigid body components creating a simulated body in the default world.
    class JoltComponentBodyCreationTests : public ::testing::Test
    {
    protected:
        class TestDefaultWorldHandler : public Physics::DefaultWorldBus::Handler
        {
        public:
            explicit TestDefaultWorldHandler(AzPhysics::SceneHandle sceneHandle)
                : m_sceneHandle(sceneHandle)
            {
                Physics::DefaultWorldBus::Handler::BusConnect();
            }
            ~TestDefaultWorldHandler() override
            {
                Physics::DefaultWorldBus::Handler::BusDisconnect();
            }

            AzPhysics::SceneHandle GetDefaultSceneHandle() const override
            {
                return m_sceneHandle;
            }

        private:
            AzPhysics::SceneHandle m_sceneHandle;
        };

        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "ComponentTestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);

            m_defaultWorldHandler = AZStd::make_unique<TestDefaultWorldHandler>(m_sceneHandle);
        }

        void TearDown() override
        {
            m_defaultWorldHandler.reset();
            m_system->RemoveScene(m_sceneHandle);
            m_system->Shutdown();
            m_system.reset();
        }

        void SimulateSeconds(float seconds)
        {
            const float fixedDeltaTime = 1.0f / 60.0f;
            const int steps = static_cast<int>(seconds / fixedDeltaTime);
            for (int i = 0; i < steps; ++i)
            {
                m_scene->StartSimulation(fixedDeltaTime);
                m_scene->FinishSimulation();
                // Pump the tick bus like a frame does; deferred component work
                // (body creation, rebuilds) runs on ticks.
                AZ::TickBus::Broadcast(&AZ::TickBus::Events::OnTick, fixedDeltaTime, AZ::ScriptTimePoint());
            }
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle;
        AzPhysics::Scene* m_scene = nullptr;
        AZStd::unique_ptr<TestDefaultWorldHandler> m_defaultWorldHandler;
    };

    TEST_F(JoltComponentBodyCreationTests, CapsuleComponentProducesZUpCapsule)
    {
        auto entity = AZStd::make_unique<AZ::Entity>("CapsuleEntity");
        entity->CreateComponent<AzFramework::TransformComponent>();
        entity->CreateComponent<JoltCapsuleColliderComponent>();
        entity->CreateComponent<JoltRigidBodyComponent>();
        entity->Init();
        entity->Activate();
        AZ::TransformBus::Event(entity->GetId(), &AZ::TransformBus::Events::SetWorldTranslation, AZ::Vector3(0.0f, 0.0f, 4.0f));

        // Static slab with top at z=0 for the capsule to land on.
        auto slabCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto slabShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        slabShape->m_dimensions = AZ::Vector3(20.0f, 20.0f, 1.0f);
        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(slabCollider, slabShape);
        m_scene->AddSimulatedBody(&slabConfig);

        SimulateSeconds(4.0f);

        // Read the simulated body (the component's transform sync back to the
        // entity runs on the tick bus, which this test does not pump).
        const auto [foundSceneHandle, bodyHandle] =
            m_system->FindAttachedBodyHandleFromEntityId(entity->GetId());
        ASSERT_NE(bodyHandle, AzPhysics::InvalidSimulatedBodyHandle);
        EXPECT_EQ(foundSceneHandle, m_sceneHandle);

        const float capsuleZ = m_scene->GetSimulatedBodyFromHandle(bodyHandle)->GetPosition().GetZ();
        EXPECT_NEAR(capsuleZ, 0.5f, 0.05f);
    }

    TEST_F(JoltComponentBodyCreationTests, TwoCollidersFormOneCompoundBody)
    {
        auto entity = AZStd::make_unique<AZ::Entity>("CompoundEntity");
        entity->CreateComponent<AzFramework::TransformComponent>();
        [[maybe_unused]] auto* colliderA = entity->CreateComponent<JoltBoxColliderComponent>();
        auto* colliderB = entity->CreateComponent<JoltBoxColliderComponent>();
        colliderB->GetColliderConfiguration().m_position = AZ::Vector3(1.0f, 0.0f, 0.0f);
        entity->CreateComponent<JoltRigidBodyComponent>();
        entity->Init();
        entity->Activate();

        // The two collider components on one entity produce exactly one simulated body.
        const auto& bodyList = static_cast<JoltScene*>(m_scene)->GetSimulatedBodyList();
        AZStd::vector<AzPhysics::SimulatedBody*> bodies;
        for (const auto& [crc, body] : bodyList)
        {
            if (body)
            {
                bodies.push_back(body);
            }
        }
        EXPECT_EQ(bodies.size(), 1u);

        // The compound body covers both colliders: a raycast down through the
        // offset collider at x=1 hits; the gap outside both boxes (x=2) misses.
        AzPhysics::RayCastRequest request;
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 10.0f;

        request.m_start = AZ::Vector3(1.0f, 0.0f, 5.0f);
        EXPECT_TRUE(m_scene->QueryScene(&request).m_hits.size() == 1u);

        request.m_start = AZ::Vector3(2.0f, 0.0f, 5.0f);
        EXPECT_TRUE(m_scene->QueryScene(&request).m_hits.empty());

        entity.reset();
    }

    TEST_F(JoltComponentBodyCreationTests, SameTypeComponentsGetDistinctSerializedIdentifiers)
    {
        // The DPE inspector requires a non-empty serialized identifier on every
        // component (AZ::Component::GetSerializedIdentifier); same-type components
        // on one entity must be deduplicated ("{TypeName}" vs "{TypeName}_2").
        auto entity = AZStd::make_unique<AZ::Entity>("IdentifierEntity");
        entity->CreateComponent<AzFramework::TransformComponent>();
        auto* colliderA = entity->CreateComponent<JoltBoxColliderComponent>();
        auto* colliderB = entity->CreateComponent<JoltBoxColliderComponent>();
        entity->Init();

        EXPECT_FALSE(colliderA->GetSerializedIdentifier().empty());
        EXPECT_FALSE(colliderB->GetSerializedIdentifier().empty());
        EXPECT_NE(colliderA->GetSerializedIdentifier(), colliderB->GetSerializedIdentifier());

        entity->Activate();
        EXPECT_FALSE(colliderA->GetSerializedIdentifier().empty());
        EXPECT_NE(colliderA->GetSerializedIdentifier(), colliderB->GetSerializedIdentifier());

        entity.reset();
    }
    TEST_F(JoltComponentBodyCreationTests, AddingAndRemovingChildColliderAtRuntimeRebuildsBody)
    {
        auto makeColliderChild = [](const char* name, float x)
        {
            auto child = AZStd::make_unique<AZ::Entity>(name);
            child->CreateComponent<AzFramework::TransformComponent>();
            auto* collider = child->CreateComponent<JoltBoxColliderComponent>();
            collider->GetColliderConfiguration().m_position = AZ::Vector3(x, 0.0f, 0.0f);
            child->Init();
            return child;
        };

        auto parent = AZStd::make_unique<AZ::Entity>("MutableCompoundParent");
        parent->CreateComponent<AzFramework::TransformComponent>();
        parent->CreateComponent<JoltMutableCompoundColliderComponent>();
        parent->CreateComponent<JoltRigidBodyComponent>();
        parent->Init();

        auto childA = makeColliderChild("ChildA", -1.0f);
        parent->Activate();
        childA->Activate();
        AZ::TransformBus::Event(childA->GetId(), &AZ::TransformBus::Events::SetParent, parent->GetId());

        // Rebuilds after collider-set changes are deferred to the next tick; simulate one frame.
        SimulateSeconds(1.0f / 60.0f);

        AzPhysics::RayCastRequest request;
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 10.0f;

        // Only child A's collider (at x=-1) exists: x=-1 hits, x=+1 misses.
        request.m_start = AZ::Vector3(-1.0f, 0.0f, 5.0f);
        EXPECT_EQ(m_scene->QueryScene(&request).m_hits.size(), 1u);
        request.m_start = AZ::Vector3(1.0f, 0.0f, 5.0f);
        EXPECT_TRUE(m_scene->QueryScene(&request).m_hits.empty());

        // Add a second child collider entity at runtime (at x=+1).
        auto childB = makeColliderChild("ChildB", 1.0f);
        childB->Activate();
        AZ::TransformBus::Event(childB->GetId(), &AZ::TransformBus::Events::SetParent, parent->GetId());

        SimulateSeconds(1.0f / 60.0f);

        request.m_start = AZ::Vector3(1.0f, 0.0f, 5.0f);
        EXPECT_EQ(m_scene->QueryScene(&request).m_hits.size(), 1u);

        // Remove it again: x=+1 misses, x=-1 still hits.
        AZ::TransformBus::Event(childB->GetId(), &AZ::TransformBus::Events::SetParent, AZ::EntityId());
        childB->Deactivate();
        childB.reset();

        SimulateSeconds(1.0f / 60.0f);

        request.m_start = AZ::Vector3(1.0f, 0.0f, 5.0f);
        EXPECT_TRUE(m_scene->QueryScene(&request).m_hits.empty());
        request.m_start = AZ::Vector3(-1.0f, 0.0f, 5.0f);
        EXPECT_EQ(m_scene->QueryScene(&request).m_hits.size(), 1u);

        parent->Deactivate();
        parent.reset();
    }

    TEST_F(JoltComponentBodyCreationTests, MutableCompoundReparentOnly)
    {
        auto parent = AZStd::make_unique<AZ::Entity>("MutableParent");
        parent->CreateComponent<AzFramework::TransformComponent>();
        parent->CreateComponent<JoltMutableCompoundColliderComponent>();
        parent->Init();
        parent->Activate();

        auto childA = AZStd::make_unique<AZ::Entity>("ChildA");
        childA->CreateComponent<AzFramework::TransformComponent>();
        childA->CreateComponent<JoltBoxColliderComponent>();
        childA->Init();
        childA->Activate();
        AZ::TransformBus::Event(childA->GetId(), &AZ::TransformBus::Events::SetParent, parent->GetId());

        auto* compound = parent->FindComponent<JoltMutableCompoundColliderComponent>();
        ASSERT_NE(compound, nullptr);
        EXPECT_EQ(compound->GetShapeColliderPairs().size(), 1u);

        AZ::TransformBus::Event(childA->GetId(), &AZ::TransformBus::Events::SetParent, AZ::EntityId());
        EXPECT_TRUE(compound->GetShapeColliderPairs().empty());

        childA->Deactivate();
        parent->Deactivate();
    }

    TEST_F(JoltComponentBodyCreationTests, ScaledEntityBakesScaleIntoColliderPairs)
    {
        auto entity = AZStd::make_unique<AZ::Entity>("ScaledPairEntity");
        entity->CreateComponent<AzFramework::TransformComponent>();
        auto* scaleComponent = entity->CreateComponent<AzFramework::NonUniformScaleComponent>();
        scaleComponent->SetScale(AZ::Vector3(2.0f, 3.0f, 4.0f));
        auto* collider = entity->CreateComponent<JoltBoxColliderComponent>();
        collider->GetColliderConfiguration().m_position = AZ::Vector3(1.0f, 0.0f, 0.0f);
        entity->CreateComponent<JoltRigidBodyComponent>();
        entity->Init();
        entity->Activate();

        const AzPhysics::ShapeColliderPairList pairs = collider->GetShapeColliderPairs();
        ASSERT_EQ(pairs.size(), 1u);

        // The entity scale lands on the shape configuration (read by
        // CreateJoltShapeFromConfig) and the authored offset scales with it.
        EXPECT_TRUE(pairs[0].second->m_scale.IsClose(AZ::Vector3(2.0f, 3.0f, 4.0f)));
        EXPECT_TRUE(pairs[0].first->m_position.IsClose(AZ::Vector3(2.0f, 0.0f, 0.0f)));
        // The serialized offset on the component is not mutated by the pair expansion.
        EXPECT_TRUE(collider->GetColliderConfiguration().m_position.IsClose(AZ::Vector3(1.0f, 0.0f, 0.0f)));

        entity->Deactivate();
    }

    TEST_F(JoltComponentBodyCreationTests, ScaledBoxRestsAtItsScaledHalfHeight)
    {
        auto entity = AZStd::make_unique<AZ::Entity>("ScaledBoxEntity");
        entity->CreateComponent<AzFramework::TransformComponent>();
        auto* scaleComponent = entity->CreateComponent<AzFramework::NonUniformScaleComponent>();
        scaleComponent->SetScale(AZ::Vector3(2.0f, 2.0f, 2.0f));
        entity->CreateComponent<JoltBoxColliderComponent>();
        entity->CreateComponent<JoltRigidBodyComponent>();
        entity->Init();
        entity->Activate();
        AZ::TransformBus::Event(entity->GetId(), &AZ::TransformBus::Events::SetWorldTranslation, AZ::Vector3(0.0f, 0.0f, 4.0f));

        // Static slab with top at z=0 for the box to land on.
        auto slabCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto slabShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        slabShape->m_dimensions = AZ::Vector3(20.0f, 20.0f, 1.0f);
        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(slabCollider, slabShape);
        m_scene->AddSimulatedBody(&slabConfig);

        SimulateSeconds(4.0f);

        // The default 1 m box scaled by 2 has half height 1 m; before entity scale was
        // propagated it rested at 0.5 m - the unscaled half height.
        const auto [foundSceneHandle, bodyHandle] =
            m_system->FindAttachedBodyHandleFromEntityId(entity->GetId());
        ASSERT_NE(bodyHandle, AzPhysics::InvalidSimulatedBodyHandle);
        const float boxZ = m_scene->GetSimulatedBodyFromHandle(bodyHandle)->GetPosition().GetZ();
        EXPECT_NEAR(boxZ, 1.0f, 0.05f);

        entity->Deactivate();
    }

} // namespace JoltPhysics
