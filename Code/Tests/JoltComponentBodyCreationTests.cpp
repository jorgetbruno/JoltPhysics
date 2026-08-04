#include <Shape/JoltShapeUtils.h>
#include <Clients/Components/JoltShapeColliderComponent.h>
#include <LmbrCentral/Shape/ShapeComponentBus.h>
#include <LmbrCentral/Shape/PolygonPrismShapeComponentBus.h>
#include <AzCore/Math/PolygonPrism.h>
#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <Clients/Components/JoltBakedMeshColliderComponent.h>
#include <Clients/Components/JoltBoxColliderComponent.h>
#include <Clients/Components/JoltCapsuleColliderComponent.h>
#include <Clients/Components/JoltColliderComponentBase.h>
#include <Clients/Components/JoltRigidBodyComponent.h>
#include <Clients/Components/JoltSoftBodyAttachmentComponent.h>
#include <JoltPhysics/JoltSoftBodyBus.h>
#include <Clients/Components/JoltSoftBodyComponent.h>
#include <JoltPhysics/JoltCharacterGameplayBus.h>
#include <Clients/Components/JoltCharacterControllerComponent.h>
#include <Clients/Components/JoltStaticRigidBodyComponent.h>
#include <Pipeline/JoltMeshAssetHandler.h>
#include <Clients/Components/JoltMeshColliderComponent.h>
#include <Clients/Components/JoltSphereColliderComponent.h>
#include <Clients/Components/JoltStaticCompoundColliderComponent.h>
#include <System/JoltSystem.h>
#include <Scene/JoltScene.h>
#include <Shape/JoltMeshUtils.h>
#include <Configuration/JoltSettingsRegistryManager.h>

#include "JoltTestWarningCatcher.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Components/NonUniformScaleComponent.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzCore/Interface/Interface.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SystemBus.h>
#include <AzFramework/Physics/WindBus.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

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

        //! The half of the cooked-mesh cache contract that lives on the system component.
        //!
        //! JoltShapeUtils AddRefs a cooked shape onto the configuration it built it from,
        //! and CookedMeshShapeConfiguration's destructor balances that by broadcasting
        //! ReleaseNativeMeshObject. Tests that own their configuration can release it by
        //! hand, but a mesh-asset collider expands into *clones* that live and die inside
        //! the gem - there is nothing to release by hand, and with no handler on the bus
        //! the shape simply leaks and the suite fails at teardown with "There are still
        //! allocations". This answers the two release calls the same way the system
        //! component does; everything else is a stub, since nothing here calls it.
        class ScopedNativeMeshReleaser : public Physics::SystemRequestBus::Handler
        {
        public:
            ScopedNativeMeshReleaser()
            {
                Physics::SystemRequestBus::Handler::BusConnect();
                // Both, as the system component does: SystemRequestBus and
                // AZ::Interface<Physics::System> are separate registrations, and the
                // engine reaches for the interface on this path.
                AZ::Interface<Physics::System>::Register(this);
            }
            ~ScopedNativeMeshReleaser() override
            {
                AZ::Interface<Physics::System>::Unregister(this);
                Physics::SystemRequestBus::Handler::BusDisconnect();
            }

            AZStd::shared_ptr<Physics::Shape> CreateShape(
                const Physics::ColliderConfiguration& colliderConfiguration,
                const Physics::ShapeConfiguration& configuration) override
            {
                return JoltShapeUtils::CreateShape(colliderConfiguration, configuration);
            }

            void ReleaseNativeMeshObject(void* nativeMeshObject) override
            {
                if (nativeMeshObject)
                {
                    ++m_releases;
                    static_cast<JPH::Shape*>(nativeMeshObject)->Release();
                }
            }

            int m_releases = 0;

            void ReleaseNativeHeightfieldObject(void* nativeHeightfieldObject) override
            {
                if (nativeHeightfieldObject)
                {
                    static_cast<JPH::Shape*>(nativeHeightfieldObject)->Release();
                }
            }

            bool CookConvexMeshToFile(const AZStd::string&, const AZ::Vector3*, AZ::u32) override
            {
                return false;
            }
            bool CookConvexMeshToMemory(const AZ::Vector3*, AZ::u32, AZStd::vector<AZ::u8>&) override
            {
                return false;
            }
            bool CookTriangleMeshToFile(
                const AZStd::string&, const AZ::Vector3*, AZ::u32, const AZ::u32*, AZ::u32) override
            {
                return false;
            }
            bool CookTriangleMeshToMemory(
                const AZ::Vector3*, AZ::u32, const AZ::u32*, AZ::u32, AZStd::vector<AZ::u8>&) override
            {
                return false;
            }
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

    TEST_F(JoltComponentBodyCreationTests, ScalingACompoundMovesItsChildrenOutWithIt)
    {
        // A child collider's geometry already carries the world scale (which includes its
        // parent's), but its placement is authored in the parent's unscaled local space.
        // Leaving that alone bunched every child towards the compound's origin while the
        // render meshes moved out with the scale - collision and visuals disagreeing on a
        // scaled crate stack, unfixable except by never scaling compounds.
        //
        // The mutable compound is used because it re-gathers when a child is reparented,
        // which is how the test gets a gather to happen after everything is positioned.
        auto childOffsetForScale = [](float parentScale)
        {
            auto parent = AZStd::make_unique<AZ::Entity>("ScaledCompoundParent");
            parent->CreateComponent<AzFramework::TransformComponent>();
            parent->CreateComponent<JoltMutableCompoundColliderComponent>();
            parent->Init();
            parent->Activate();
            AZ::TransformBus::Event(
                parent->GetId(), &AZ::TransformBus::Events::SetLocalUniformScale, parentScale);

            auto child = AZStd::make_unique<AZ::Entity>("CompoundChild");
            child->CreateComponent<AzFramework::TransformComponent>();
            child->CreateComponent<JoltBoxColliderComponent>();
            child->Init();
            child->Activate();
            AZ::TransformBus::Event(child->GetId(), &AZ::TransformBus::Events::SetParent, parent->GetId());
            AZ::TransformBus::Event(
                child->GetId(), &AZ::TransformBus::Events::SetLocalTranslation, AZ::Vector3(2.0f, 0.0f, 0.0f));

            // Reparent so the compound gathers again now the child is where it belongs.
            AZ::TransformBus::Event(child->GetId(), &AZ::TransformBus::Events::SetParent, AZ::EntityId());
            AZ::TransformBus::Event(child->GetId(), &AZ::TransformBus::Events::SetParent, parent->GetId());

            auto* compound = parent->FindComponent<JoltMutableCompoundColliderComponent>();
            const AzPhysics::ShapeColliderPairList pairs =
                compound != nullptr ? compound->GetShapeColliderPairs() : AzPhysics::ShapeColliderPairList{};

            float offsetX = 0.0f;
            if (pairs.size() == 1 && pairs[0].first != nullptr)
            {
                offsetX = pairs[0].first->m_position.GetX();
            }
            else
            {
                ADD_FAILURE() << "expected exactly one gathered child collider, got " << pairs.size();
            }

            child->Deactivate();
            parent->Deactivate();
            return offsetX;
        };

        // Unscaled, the child sits where it was authored...
        EXPECT_NEAR(childOffsetForScale(1.0f), 2.0f, 0.01f);
        // ...and tripling the compound has to take it out to where its render mesh went.
        EXPECT_NEAR(childOffsetForScale(3.0f), 6.0f, 0.01f);
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

    // Entity with a NonUniformScale component set to the given scale, ready for a
    // collider component to be added before Init/Activate.
    static AZStd::unique_ptr<AZ::Entity> MakeScaledEntity(const char* name, const AZ::Vector3& scale)
    {
        auto entity = AZStd::make_unique<AZ::Entity>(name);
        entity->CreateComponent<AzFramework::TransformComponent>();
        auto* scaleComponent = entity->CreateComponent<AzFramework::NonUniformScaleComponent>();
        scaleComponent->SetScale(scale);
        return entity;
    }

    // Releases the native shape cached on a cooked configuration by
    // CreateJoltShapeFromConfig (in production this is balanced by
    // JoltPhysicsSystemComponent::ReleaseNativeMeshObject, which this test does not run).
    static void ReleaseCachedNativeMesh(Physics::CookedMeshShapeConfiguration& configuration)
    {
        if (auto* cachedMesh = static_cast<JPH::Shape*>(configuration.GetCachedNativeMesh()))
        {
            cachedMesh->Release();
            configuration.SetCachedNativeMesh(nullptr);
        }
    }

    // Cooked convex blob for a 1 m cube centered on the origin.
    static AZStd::vector<AZ::u8> MakeUnitCubeConvexBlob()
    {
        const AZ::Vector3 corners[8] = {
            AZ::Vector3(-0.5f, -0.5f, -0.5f), AZ::Vector3(0.5f, -0.5f, -0.5f),
            AZ::Vector3(-0.5f, 0.5f, -0.5f),  AZ::Vector3(0.5f, 0.5f, -0.5f),
            AZ::Vector3(-0.5f, -0.5f, 0.5f),  AZ::Vector3(0.5f, -0.5f, 0.5f),
            AZ::Vector3(-0.5f, 0.5f, 0.5f),   AZ::Vector3(0.5f, 0.5f, 0.5f),
        };
        return JoltMeshUtils::PackConvexMesh(corners, 8);
    }

    TEST_F(JoltComponentBodyCreationTests, ScaledEntityScalesEveryColliderKind)
    {
        // The box case is pinned above; this pins every other collider kind that flows
        // through the base's scale propagation: sphere, capsule, and the baked mesh.
        // This is the pair level only - what the shape does with a non-uniform scale is
        // per shape type, and pinned in ScaledColliderGeometryMatchesTheShapeTypesLimits.
        const AZ::Vector3 expectedScale(2.0f, 3.0f, 4.0f);

        auto expectPairsScaled = [expectedScale](const JoltColliderComponentBase* collider)
        {
            const AzPhysics::ShapeColliderPairList pairs = collider->GetShapeColliderPairs();
            EXPECT_EQ(pairs.size(), 1u);
            if (!pairs.empty())
            {
                EXPECT_TRUE(pairs[0].second->m_scale.IsClose(expectedScale));
            }
        };

        {
            auto entity = MakeScaledEntity("ScaledSphereEntity", expectedScale);
            auto* collider = entity->CreateComponent<JoltSphereColliderComponent>();
            entity->Init();
            entity->Activate();
            expectPairsScaled(collider);
            entity->Deactivate();
        }

        {
            auto entity = MakeScaledEntity("ScaledCapsuleEntity", expectedScale);
            auto* collider = entity->CreateComponent<JoltCapsuleColliderComponent>();
            entity->Init();
            entity->Activate();
            expectPairsScaled(collider);
            entity->Deactivate();
        }

        {
            auto entity = MakeScaledEntity("ScaledBakedMeshEntity", expectedScale);
            auto* collider = entity->CreateComponent<JoltBakedMeshColliderComponent>();
            const AZStd::vector<AZ::u8> blob = MakeUnitCubeConvexBlob();
            collider->GetShapeConfiguration().SetCookedMeshData(
                blob.data(), blob.size(), Physics::CookedMeshShapeConfiguration::MeshType::Convex);
            entity->Init();
            entity->Activate();
            expectPairsScaled(collider);
            entity->Deactivate();
            ReleaseCachedNativeMesh(collider->GetShapeConfiguration());
        }
    }

    //! The character controller falling under its own steam. Jolt gives a character no
    //! gravity of its own - it is driven entirely by requested velocity - so until this
    //! existed every project wrote the same accumulate-and-apply loop by hand, and the
    //! ones that did not had characters that hung in the air.
    class JoltCharacterGravityTests : public JoltComponentBodyCreationTests
    {
    protected:
        //! Floor with its top surface at z = 0.
        void AddFloor()
        {
            auto collider = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto shape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            shape->m_dimensions = AZ::Vector3(20.0f, 20.0f, 1.0f);
            AzPhysics::StaticRigidBodyConfiguration floorConfig;
            floorConfig.m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
            floorConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(collider, shape);
            m_scene->AddSimulatedBody(&floorConfig);
        }

        //! A character standing at the given height. Its transform is its base - the feet -
        //! which is the convention the rest of O3DE uses.
        AZStd::unique_ptr<AZ::Entity> CreateCharacterEntity(float height)
        {
            auto entity = AZStd::make_unique<AZ::Entity>("GravityCharacter");
            entity->CreateComponent<AzFramework::TransformComponent>();
            entity->CreateComponent<JoltCharacterControllerComponent>();
            entity->Init();
            entity->Activate();
            AZ::TransformBus::Event(
                entity->GetId(), &AZ::TransformBus::Events::SetWorldTM,
                AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, height)));
            return entity;
        }

        static float BaseHeightOf(const AZ::Entity& entity)
        {
            AZ::Vector3 position = AZ::Vector3::CreateZero();
            AZ::TransformBus::EventResult(position, entity.GetId(), &AZ::TransformBus::Events::GetWorldTranslation);
            return position.GetZ();
        }
    };

    TEST_F(JoltCharacterGravityTests, ACharacterFallsAndLandsWithNobodyDrivingIt)
    {
        AddFloor();
        auto entity = CreateCharacterEntity(3.0f);

        SimulateSeconds(3.0f);

        // Its feet end up on the floor, and it stays there rather than sinking through or
        // jittering on the spot.
        const float landed = BaseHeightOf(*entity);
        EXPECT_NEAR(landed, 0.0f, 0.1f) << "the character did not land on the floor";

        SimulateSeconds(1.0f);
        EXPECT_NEAR(BaseHeightOf(*entity), landed, 0.01f) << "the character did not settle";

        entity->Deactivate();
    }

    TEST_F(JoltCharacterGravityTests, GravityOffLeavesAnAnimationDrivenCharacterWhereItIs)
    {
        AddFloor();
        auto entity = CreateCharacterEntity(3.0f);

        JoltCharacterGameplayRequestBus::Event(
            entity->GetId(), &JoltCharacterGameplayRequests::SetGravityMultiplier, 0.0f);

        SimulateSeconds(2.0f);

        // Nothing else is driving it, so it must hang exactly where it was put: a
        // character whose vertical motion comes from animation has to be left alone.
        EXPECT_NEAR(BaseHeightOf(*entity), 3.0f, 0.01f);

        entity->Deactivate();
    }

    TEST_F(JoltCharacterGravityTests, AJumpSurvivesTheFrameItStartsOn)
    {
        // The frame a jump starts on is a frame the character is still touching the
        // floor. Shedding all of the accumulated velocity on contact - rather than only
        // the part pulling downwards - would eat the jump before it moved anything.
        AddFloor();
        auto entity = CreateCharacterEntity(0.0f);
        SimulateSeconds(0.5f); // settle onto the floor

        const float standing = BaseHeightOf(*entity);
        JoltCharacterGameplayRequestBus::Event(
            entity->GetId(), &JoltCharacterGameplayRequests::SetFallingVelocity, AZ::Vector3(0.0f, 0.0f, 5.0f));

        float peak = standing;
        for (int i = 0; i < 30; ++i)
        {
            SimulateSeconds(1.0f / 60.0f);
            peak = AZ::GetMax(peak, BaseHeightOf(*entity));
        }
        EXPECT_GT(peak, standing + 0.5f) << "the jump never left the ground";

        // And gravity takes it back down again rather than leaving it up there.
        SimulateSeconds(3.0f);
        EXPECT_NEAR(BaseHeightOf(*entity), standing, 0.1f);

        entity->Deactivate();
    }

    TEST_F(JoltCharacterGravityTests, TheFallingVelocityIsReadableAndDoesNotGrowWhileStanding)
    {
        AddFloor();
        auto entity = CreateCharacterEntity(0.0f);
        SimulateSeconds(2.0f);

        AZ::Vector3 fallingVelocity = AZ::Vector3::CreateZero();
        JoltCharacterGameplayRequestBus::EventResult(
            fallingVelocity, entity->GetId(), &JoltCharacterGameplayRequests::GetFallingVelocity);

        // One tick of gravity, not two seconds of it: standing on the floor sheds what
        // has built up, or a character that had been idle would launch itself the moment
        // it stepped off a ledge.
        EXPECT_LT(fallingVelocity.GetLength(), 1.0f)
            << "the falling velocity kept accumulating while the character stood still";

        bool onGround = false;
        JoltCharacterGameplayRequestBus::EventResult(
            onGround, entity->GetId(), &JoltCharacterGameplayRequests::IsOnGround);
        EXPECT_TRUE(onGround);

        entity->Deactivate();
    }

    //! Geometry handed to a soft body at runtime. The gem that does this (JoltCloth) reads
    //! a character's cloth mesh, welds it, and then has to map simulated particles back
    //! onto render vertices - so what it needs from this bus is not just that the surface
    //! simulates but that the particles come back in the order it sent them.
    class JoltSoftBodyCustomGeometryTests : public JoltComponentBodyCreationTests
    {
    protected:
        //! A soft body with no geometry yet, which is what one looks like for the frame or
        //! two before whatever feeds it is ready.
        AZStd::unique_ptr<AZ::Entity> CreateEmptyCustomBody(const AZ::Vector3& position)
        {
            auto entity = AZStd::make_unique<AZ::Entity>("CustomSoftBody");
            entity->CreateComponent<AzFramework::TransformComponent>();
            auto* softBody = entity->CreateComponent<JoltSoftBodyComponent>();

            JoltSoftBodySettings& settings = softBody->GetSettings();
            settings.m_shape = JoltSoftBodyShape::Custom;
            settings.m_mass = 1.0f;
            settings.m_allowSleeping = false;
            settings.m_updatePosition = false;

            entity->Init();
            AZ::TransformBus::Event(
                entity->GetId(), &AZ::TransformBus::Events::SetWorldTM, AZ::Transform::CreateTranslation(position));
            entity->Activate();
            return entity;
        }

        //! A flat grid of quads in the XY plane, every position distinct - what a welded
        //! render mesh looks like by the time a caller has finished with it.
        static void MakeGrid(
            AZ::u32 sideVertices, AZStd::vector<AZ::Vector3>& outVertices, AZStd::vector<AZ::u32>& outIndices)
        {
            for (AZ::u32 y = 0; y < sideVertices; ++y)
            {
                for (AZ::u32 x = 0; x < sideVertices; ++x)
                {
                    outVertices.push_back(
                        AZ::Vector3(aznumeric_cast<float>(x) * 0.25f, aznumeric_cast<float>(y) * 0.25f, 0.0f));
                }
            }
            for (AZ::u32 y = 0; y + 1 < sideVertices; ++y)
            {
                for (AZ::u32 x = 0; x + 1 < sideVertices; ++x)
                {
                    const AZ::u32 a = y * sideVertices + x;
                    const AZ::u32 b = a + 1;
                    const AZ::u32 c = a + sideVertices;
                    const AZ::u32 d = c + 1;
                    outIndices.insert(outIndices.end(), { a, b, d });
                    outIndices.insert(outIndices.end(), { a, d, c });
                }
            }
        }
    };

    TEST_F(JoltSoftBodyCustomGeometryTests, GeometryWithUniquePositionsKeepsItsCountAndItsOrder)
    {
        // The invariant JoltCloth is built on. It welds the render mesh itself - it has to,
        // because it needs the render-vertex-to-particle map for writing simulation back -
        // and then treats its own vertex indices as particle indices. If welding here
        // reordered or merged anything, every skin weight and every written-back vertex
        // would land on the wrong particle, and the cloth would look scrambled rather than
        // wrong in any way that points at this.
        const AZ::Vector3 bodyPosition(3.0f, -1.0f, 6.0f);
        auto entity = CreateEmptyCustomBody(bodyPosition);
        const AZ::EntityId entityId = entity->GetId();

        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        MakeGrid(4, vertices, indices);

        JoltSoftBodyRequestBus::Event(entityId, &JoltSoftBodyRequests::SetCustomGeometry, vertices, indices);

        AZ::u32 vertexCount = 0;
        JoltSoftBodyRequestBus::EventResult(vertexCount, entityId, &JoltSoftBodyRequests::GetVertexCount);
        ASSERT_EQ(vertexCount, vertices.size());

        for (AZ::u32 index = 0; index < vertexCount; ++index)
        {
            AZ::Vector3 particle = AZ::Vector3::CreateZero();
            JoltSoftBodyRequestBus::EventResult(
                particle, entityId, &JoltSoftBodyRequests::GetVertexPosition, index);
            // Supplied geometry is entity-local, so the entity transform places it.
            EXPECT_TRUE(particle.IsClose(bodyPosition + vertices[index], 1e-3f))
                << "particle " << index << " is not where vertex " << index << " was sent";
        }

        entity->Deactivate();
    }

    TEST_F(JoltSoftBodyCustomGeometryTests, VerticesSharingAPositionSimulateAsOneParticle)
    {
        // A render mesh splits vertices along every normal and UV seam. Simulated as they
        // arrive, the sheet would come apart at each seam, because constraints only connect
        // vertices that share a face.
        auto entity = CreateEmptyCustomBody(AZ::Vector3::CreateZero());
        const AZ::EntityId entityId = entity->GetId();

        // Two triangles that meet along an edge, with that edge duplicated - the seam.
        const AZStd::vector<AZ::Vector3> vertices = {
            AZ::Vector3(0.0f, 0.0f, 0.0f), AZ::Vector3(1.0f, 0.0f, 0.0f), AZ::Vector3(0.0f, 1.0f, 0.0f),
            AZ::Vector3(1.0f, 0.0f, 0.0f), AZ::Vector3(0.0f, 1.0f, 0.0f), AZ::Vector3(1.0f, 1.0f, 0.0f),
        };
        const AZStd::vector<AZ::u32> indices = { 0, 1, 2, 3, 5, 4 };

        JoltSoftBodyRequestBus::Event(entityId, &JoltSoftBodyRequests::SetCustomGeometry, vertices, indices);

        AZ::u32 vertexCount = 0;
        JoltSoftBodyRequestBus::EventResult(vertexCount, entityId, &JoltSoftBodyRequests::GetVertexCount);
        EXPECT_EQ(vertexCount, 4u) << "the duplicated seam vertices did not weld";

        entity->Deactivate();
    }

    TEST_F(JoltSoftBodyCustomGeometryTests, SliverFacesAreDroppedRatherThanHandedToJolt)
    {
        // A triangle with no area takes Jolt down. Colliding a convex shape against a soft
        // body seeds GJK with each face's raw cross product and asserts if it is near zero,
        // so a sliver simulates quietly until the first collision query reaches it and then
        // kills the process from inside Jolt, in a stack that names neither the mesh nor
        // this gem. Art meshes carry slivers routinely, so they must not reach Jolt at all.
        auto entity = CreateEmptyCustomBody(AZ::Vector3::CreateZero());
        const AZ::EntityId entityId = entity->GetId();

        // Four distinct particles - no weld merges them - making one honest triangle and
        // one whose three corners are collinear, which is the case index comparison misses.
        const AZStd::vector<AZ::Vector3> vertices = {
            AZ::Vector3(0.0f, 0.0f, 0.0f),
            AZ::Vector3(1.0f, 0.0f, 0.0f),
            AZ::Vector3(0.0f, 1.0f, 0.0f),
            AZ::Vector3(0.5f, 0.0f, 0.0f),
        };
        const AZStd::vector<AZ::u32> indices = { 0, 1, 2, 0, 3, 1 };

        JoltSoftBodyRequestBus::Event(entityId, &JoltSoftBodyRequests::SetCustomGeometry, vertices, indices);

        AZ::u32 vertexCount = 0;
        JoltSoftBodyRequestBus::EventResult(vertexCount, entityId, &JoltSoftBodyRequests::GetVertexCount);
        EXPECT_EQ(vertexCount, 4u) << "the sliver's corners are distinct positions and must survive welding";

        AZStd::vector<AZ::u32> triangles;
        JoltSoftBodyRequestBus::EventResult(triangles, entityId, &JoltSoftBodyRequests::GetTriangleIndices);
        EXPECT_EQ(triangles.size(), 3u) << "the collinear face was kept; Jolt will assert the moment anything hits it";
        if (triangles.size() == 3)
        {
            EXPECT_EQ(triangles[0], 0u);
            EXPECT_EQ(triangles[1], 1u);
            EXPECT_EQ(triangles[2], 2u);
        }

        entity->Deactivate();
    }

    TEST_F(JoltSoftBodyCustomGeometryTests, EveryFaceBeingASliverLeavesNoBodyRatherThanABrokenOne)
    {
        // Nothing usable came out the other side, so there is no body - as opposed to an
        // empty one that reports a vertex count and simulates nothing.
        auto entity = CreateEmptyCustomBody(AZ::Vector3::CreateZero());
        const AZ::EntityId entityId = entity->GetId();

        const AZStd::vector<AZ::Vector3> vertices = {
            AZ::Vector3(0.0f, 0.0f, 0.0f),
            AZ::Vector3(1.0f, 0.0f, 0.0f),
            AZ::Vector3(0.5f, 0.0f, 0.0f),
        };
        const AZStd::vector<AZ::u32> indices = { 0, 1, 2 };

        // The warning this logs is the point of it, so it is left to print.
        JoltSoftBodyRequestBus::Event(entityId, &JoltSoftBodyRequests::SetCustomGeometry, vertices, indices);

        AZ::u32 vertexCount = 0;
        JoltSoftBodyRequestBus::EventResult(vertexCount, entityId, &JoltSoftBodyRequests::GetVertexCount);
        EXPECT_EQ(vertexCount, 0u) << "a body was built out of geometry with no usable face in it";

        entity->Deactivate();
    }

    TEST_F(JoltSoftBodyCustomGeometryTests, SuppliedGeometryIsSimulatedAndNotJustStored)
    {
        auto entity = CreateEmptyCustomBody(AZ::Vector3(0.0f, 0.0f, 10.0f));
        const AZ::EntityId entityId = entity->GetId();

        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        MakeGrid(4, vertices, indices);
        JoltSoftBodyRequestBus::Event(entityId, &JoltSoftBodyRequests::SetCustomGeometry, vertices, indices);

        AZ::Aabb before = AZ::Aabb::CreateNull();
        JoltSoftBodyRequestBus::EventResult(before, entityId, &JoltSoftBodyRequests::GetWorldBounds);
        ASSERT_TRUE(before.IsValid());

        SimulateSeconds(1.0f);

        AZ::Aabb after = AZ::Aabb::CreateNull();
        JoltSoftBodyRequestBus::EventResult(after, entityId, &JoltSoftBodyRequests::GetWorldBounds);
        ASSERT_TRUE(after.IsValid());
        // Nothing is pinned, so a second of gravity should have taken it several metres.
        EXPECT_LT(after.GetCenter().GetZ(), before.GetCenter().GetZ() - 1.0f);

        entity->Deactivate();
    }

    TEST_F(JoltSoftBodyCustomGeometryTests, ReadingParticlesIntoTheCallersBufferReusesIt)
    {
        // Anything drawing a soft body reads every particle every frame. GetVertexPositions
        // hands back a new vector each time, which on a character's cloth mesh is a
        // fifty-thousand-element allocation sixty times a second - so the read that reuses
        // the caller's buffer has to be on the bus, not just inside the gem.
        auto entity = CreateEmptyCustomBody(AZ::Vector3::CreateZero());
        const AZ::EntityId entityId = entity->GetId();

        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        MakeGrid(5, vertices, indices);
        JoltSoftBodyRequestBus::Event(entityId, &JoltSoftBodyRequests::SetCustomGeometry, vertices, indices);

        AZStd::vector<AZ::Vector3> buffer;
        bool copied = false;
        JoltSoftBodyRequestBus::EventResult(
            copied, entityId, &JoltSoftBodyRequests::CopyVertexPositions, buffer);
        ASSERT_TRUE(copied);
        ASSERT_EQ(buffer.size(), vertices.size());

        const size_t capacityAfterFirstRead = buffer.capacity();
        const AZ::Vector3* dataAfterFirstRead = buffer.data();

        SimulateSeconds(0.2f);

        JoltSoftBodyRequestBus::EventResult(
            copied, entityId, &JoltSoftBodyRequests::CopyVertexPositions, buffer);
        ASSERT_TRUE(copied);
        EXPECT_EQ(buffer.capacity(), capacityAfterFirstRead);
        EXPECT_EQ(buffer.data(), dataAfterFirstRead) << "the buffer was reallocated on a steady-state read";

        // And it agrees with the allocating read it replaces.
        AZStd::vector<AZ::Vector3> byValue;
        JoltSoftBodyRequestBus::EventResult(byValue, entityId, &JoltSoftBodyRequests::GetVertexPositions);
        ASSERT_EQ(byValue.size(), buffer.size());
        for (size_t index = 0; index < byValue.size(); ++index)
        {
            EXPECT_TRUE(byValue[index].IsClose(buffer[index], 1e-3f));
        }

        entity->Deactivate();
    }

    TEST_F(JoltSoftBodyCustomGeometryTests, ReadingParticlesOfABodyThatIsNotThereEmptiesTheBuffer)
    {
        // A caller polls this every frame, including the frames before the geometry lands
        // and after the body goes away. Leaving the previous frame's particles in the buffer
        // would draw a cloth that is no longer simulating anywhere.
        auto entity = CreateEmptyCustomBody(AZ::Vector3::CreateZero());
        const AZ::EntityId entityId = entity->GetId();

        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        MakeGrid(3, vertices, indices);
        JoltSoftBodyRequestBus::Event(entityId, &JoltSoftBodyRequests::SetCustomGeometry, vertices, indices);

        AZStd::vector<AZ::Vector3> buffer;
        bool copied = false;
        JoltSoftBodyRequestBus::EventResult(
            copied, entityId, &JoltSoftBodyRequests::CopyVertexPositions, buffer);
        ASSERT_TRUE(copied);
        ASSERT_FALSE(buffer.empty());

        JoltSoftBodyRequestBus::Event(
            entityId, &JoltSoftBodyRequests::SetCustomGeometry, AZStd::vector<AZ::Vector3>(),
            AZStd::vector<AZ::u32>());

        JoltSoftBodyRequestBus::EventResult(
            copied, entityId, &JoltSoftBodyRequests::CopyVertexPositions, buffer);
        EXPECT_FALSE(copied);
        EXPECT_TRUE(buffer.empty());

        entity->Deactivate();
    }

    TEST_F(JoltSoftBodyCustomGeometryTests, ABodyWaitingForItsGeometryIsEmptyRatherThanBroken)
    {
        // A Custom body activates before whatever feeds it has anything to feed - an actor
        // asset and a physics scene do not become ready in a fixed order. That gap must
        // read as "no particles yet", not as a failed component.
        auto entity = CreateEmptyCustomBody(AZ::Vector3::CreateZero());
        const AZ::EntityId entityId = entity->GetId();

        AZ::u32 vertexCount = 1;
        JoltSoftBodyRequestBus::EventResult(vertexCount, entityId, &JoltSoftBodyRequests::GetVertexCount);
        EXPECT_EQ(vertexCount, 0u);

        SimulateSeconds(0.1f);

        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        MakeGrid(3, vertices, indices);
        JoltSoftBodyRequestBus::Event(entityId, &JoltSoftBodyRequests::SetCustomGeometry, vertices, indices);

        JoltSoftBodyRequestBus::EventResult(vertexCount, entityId, &JoltSoftBodyRequests::GetVertexCount);
        EXPECT_EQ(vertexCount, vertices.size());

        // And handing back nothing empties it again rather than leaving the old surface.
        JoltSoftBodyRequestBus::Event(
            entityId, &JoltSoftBodyRequests::SetCustomGeometry, AZStd::vector<AZ::Vector3>(),
            AZStd::vector<AZ::u32>());
        JoltSoftBodyRequestBus::EventResult(vertexCount, entityId, &JoltSoftBodyRequests::GetVertexCount);
        EXPECT_EQ(vertexCount, 0u);

        entity->Deactivate();
    }

    //! Soft body skinning as it is reached from outside this gem - through the bus,
    //! in world space. The sibling gem that drives it (JoltCloth) has an actor's pose and
    //! nothing else, so this is the contract that matters.
    class JoltSoftBodySkinningBusTests : public JoltComponentBodyCreationTests
    {
    protected:
        //! A free-hanging cloth on an entity, at the given world position.
        AZStd::unique_ptr<AZ::Entity> CreateCloth(const AZ::Vector3& position)
        {
            auto entity = AZStd::make_unique<AZ::Entity>("SkinnedCloth");
            entity->CreateComponent<AzFramework::TransformComponent>();
            auto* softBody = entity->CreateComponent<JoltSoftBodyComponent>();

            JoltSoftBodySettings& settings = softBody->GetSettings();
            settings.m_shape = JoltSoftBodyShape::Cloth;
            settings.m_pinning = JoltSoftBodyPinning::None;
            settings.m_size = AZ::Vector3(1.0f, 1.0f, 1.0f);
            settings.m_resolution = 4;
            settings.m_mass = 1.0f;
            settings.m_allowSleeping = false;
            // The body must not chase its own frame while the joints move it.
            settings.m_updatePosition = false;

            entity->Init();
            AZ::TransformBus::Event(
                entity->GetId(), &AZ::TransformBus::Events::SetWorldTM, AZ::Transform::CreateTranslation(position));
            entity->Activate();
            return entity;
        }

        static AZ::Vector3 ClothCentre(const AZ::EntityId& entityId)
        {
            AZ::Aabb bounds = AZ::Aabb::CreateNull();
            JoltSoftBodyRequestBus::EventResult(bounds, entityId, &JoltSoftBodyRequests::GetWorldBounds);
            return bounds.IsValid() ? bounds.GetCenter() : AZ::Vector3::CreateZero();
        }

        //! Every particle tied to one joint, allowed a little drift.
        static AZStd::vector<JoltSoftBodySkinnedVertex> SkinAllToOneJoint(const AZ::EntityId& entityId)
        {
            AZ::u32 vertexCount = 0;
            JoltSoftBodyRequestBus::EventResult(vertexCount, entityId, &JoltSoftBodyRequests::GetVertexCount);

            AZStd::vector<JoltSoftBodySkinnedVertex> skinnedVertices;
            skinnedVertices.reserve(vertexCount);
            for (AZ::u32 index = 0; index < vertexCount; ++index)
            {
                JoltSoftBodySkinnedVertex vertex;
                vertex.m_vertexIndex = index;
                vertex.m_influences.push_back({ 0, 1.0f });
                vertex.m_maxDistance = 0.05f;
                skinnedVertices.push_back(vertex);
            }
            return skinnedVertices;
        }
    };

    //! Wind acting on cloth. The force-region tests cover how regions become a wind
    //! vector; this covers the other half - a wind vector reaching the interface must
    //! move a soft body - so the provider here is a stub, not the region plumbing.
    class JoltSoftBodyWindTests : public JoltSoftBodySkinningBusTests
    {
    protected:
        //! Constant wind everywhere, standing in for the wind provider the test
        //! environment does not create.
        class ScopedTestWind final : public AZ::Interface<Physics::WindRequests>::Registrar
        {
        public:
            AZ::Vector3 m_wind = AZ::Vector3::CreateZero();

            AZ::Vector3 GetGlobalWind() const override
            {
                return m_wind;
            }
            AZ::Vector3 GetWind([[maybe_unused]] const AZ::Vector3& worldPosition) const override
            {
                return m_wind;
            }
            AZ::Vector3 GetWind([[maybe_unused]] const AZ::Aabb& aabb) const override
            {
                return m_wind;
            }
        };

        //! A free cloth with gravity off, so any motion is the wind's doing.
        AZStd::unique_ptr<AZ::Entity> CreateFloatingCloth(const AZ::Vector3& position)
        {
            auto entity = CreateCloth(position);
            JoltSoftBodyRequestBus::Event(entity->GetId(), &JoltSoftBodyRequests::SetGravityFactor, 0.0f);
            return entity;
        }
    };

    TEST_F(JoltSoftBodyWindTests, WindCarriesAClothDownwindAndNoInfluenceOptsOut)
    {
        ScopedTestWind wind;
        // Along the cloth's face normal - the sheets are built in their local XY plane.
        // A wind along the plane instead would be the edge-on case, whose answer is
        // legitimately "no force"; the directional test below pins that.
        wind.m_wind = AZ::Vector3(0.0f, 0.0f, 5.0f);

        // Two identical cloths in the same wind; one has opted out. The pair is the test:
        // the difference between them can only be ApplyWind, not solver drift.
        auto blown = CreateFloatingCloth(AZ::Vector3(0.0f, 0.0f, 5.0f));
        auto optedOut = CreateFloatingCloth(AZ::Vector3(0.0f, 20.0f, 5.0f));
        JoltSoftBodyRequestBus::Event(optedOut->GetId(), &JoltSoftBodyRequests::SetWindInfluence, 0.0f);

        const AZ::Vector3 blownStart = ClothCentre(blown->GetId());
        const AZ::Vector3 optedOutStart = ClothCentre(optedOut->GetId());

        SimulateSeconds(1.0f);

        EXPECT_GT(ClothCentre(blown->GetId()).GetZ() - blownStart.GetZ(), 0.5f)
            << "the wind did not move the cloth";
        EXPECT_NEAR(ClothCentre(optedOut->GetId()).GetZ(), optedOutStart.GetZ(), 0.05f)
            << "a cloth with zero wind influence was still blown";

        blown->Deactivate();
        optedOut->Deactivate();
    }

    TEST_F(JoltSoftBodyWindTests, AClothFaceOnToTheWindCatchesFarMoreThanOneEdgeOn)
    {
        // The property that makes a sail gameplay rather than decoration: trim it across
        // the wind and it fills, feather it into the wind and it luffs. The cloth is
        // built in its entity's local XY plane, so its faces look along Z.
        ScopedTestWind wind;

        auto faceOn = CreateFloatingCloth(AZ::Vector3(0.0f, 0.0f, 5.0f));
        auto edgeOn = CreateFloatingCloth(AZ::Vector3(0.0f, 20.0f, 5.0f));

        const float faceOnStartZ = ClothCentre(faceOn->GetId()).GetZ();
        const float edgeOnStartX = ClothCentre(edgeOn->GetId()).GetX();

        // Blow along the first cloth's normal and along the second one's plane. One wind
        // vector cannot do both, so run them one at a time.
        wind.m_wind = AZ::Vector3(0.0f, 0.0f, 4.0f);
        SimulateSeconds(0.5f);
        const float faceOnDrift = ClothCentre(faceOn->GetId()).GetZ() - faceOnStartZ;

        wind.m_wind = AZ::Vector3(4.0f, 0.0f, 0.0f);
        SimulateSeconds(0.5f);
        const float edgeOnDrift = ClothCentre(edgeOn->GetId()).GetX() - edgeOnStartX;

        EXPECT_GT(faceOnDrift, 0.2f);
        EXPECT_GT(faceOnDrift, 3.0f * AZ::GetMax(edgeOnDrift, 0.0f))
            << "an edge-on cloth caught nearly as much wind as a face-on one";

        faceOn->Deactivate();
        edgeOn->Deactivate();
    }

    //! Cloth rigged to a moving entity - the attachment component. The pinning presets
    //! anchor in world space, which is right for a curtain and wrong for a sail; these
    //! pin the difference.
    class JoltSoftBodyAttachmentTests : public JoltSoftBodyWindTests
    {
    protected:
        //! A kinematic block to hang cloth from - a yard, as far as the cloth knows.
        AZStd::unique_ptr<AZ::Entity> CreateYard(const AZ::Vector3& position)
        {
            auto entity = AZStd::make_unique<AZ::Entity>("Yard");
            entity->CreateComponent<AzFramework::TransformComponent>();
            auto* body = entity->CreateComponent<JoltRigidBodyComponent>();
            body->GetConfiguration().m_kinematic = true;
            auto* collider = entity->CreateComponent<JoltBoxColliderComponent>();
            collider->GetShapeConfiguration().m_dimensions = AZ::Vector3(2.0f, 0.2f, 0.2f);

            entity->Init();
            AZ::TransformBus::Event(
                entity->GetId(), &AZ::TransformBus::Events::SetWorldTM, AZ::Transform::CreateTranslation(position));
            entity->Activate();
            return entity;
        }

        //! A cloth hanging with its top row inside attach range of the given point.
        //! CreateCloth builds a 1x1 sheet in the XY plane centred on its position, so the
        //! +y edge is 0.5 from centre.
        AZStd::unique_ptr<AZ::Entity> CreateHangingCloth(
            const AZ::Vector3& position, const AZ::EntityId& targetEntity, const AZ::EntityId& pushEntity)
        {
            auto entity = CreateCloth(position);
            entity->Deactivate();
            auto* attachment = entity->CreateComponent<JoltSoftBodyAttachmentComponent>();
            JoltSoftBodyAttachTarget target;
            target.m_entity = targetEntity;
            target.m_attachDistance = 0.3f;
            attachment->GetTargets().push_back(target);
            attachment->GetPushEntity() = pushEntity;
            entity->Activate();
            return entity;
        }

        //! Ticks so the attachment's bind retry runs, then simulates.
        void TickAndSimulate(float seconds)
        {
            AZ::TickBus::Broadcast(&AZ::TickEvents::OnTick, 1.0f / 60.0f, AZ::ScriptTimePoint());
            SimulateSeconds(seconds);
        }
    };

    TEST_F(JoltSoftBodyAttachmentTests, ARiggedClothHoldsItsShapeWithTheDefaultUpdatePosition)
    {
        // The fixture above turns "update position" off by hand, which is what every
        // skinned test has quietly relied on. An author will not: it defaults on. With it
        // on, Jolt moves the body's centre of mass to follow the particles - and that is
        // the very frame the skinning maths works in, so the targets drift out from under
        // the cloth, which drags it further, which moves the frame again. A sail rigged to
        // a boat collapsed from a 2 m sheet into a flat rag inside a second.
        auto yard = CreateYard(AZ::Vector3(0.0f, 0.0f, 8.0f));

        auto entity = AZStd::make_unique<AZ::Entity>("RiggedSail");
        entity->CreateComponent<AzFramework::TransformComponent>();
        auto* softBody = entity->CreateComponent<JoltSoftBodyComponent>();

        JoltSoftBodySettings& settings = softBody->GetSettings();
        settings.m_shape = JoltSoftBodyShape::Cloth;
        settings.m_pinning = JoltSoftBodyPinning::None;
        settings.m_size = AZ::Vector3(2.0f, 2.0f, 1.0f);
        settings.m_resolution = 6;
        settings.m_mass = 2.0f;
        settings.m_allowSleeping = false;
        // Left at its default on purpose - that default is the subject.
        ASSERT_TRUE(settings.m_updatePosition);

        auto* attachment = entity->CreateComponent<JoltSoftBodyAttachmentComponent>();
        JoltSoftBodyAttachTarget target;
        target.m_entity = yard->GetId();
        target.m_attachDistance = 0.4f;
        attachment->GetTargets().push_back(target);

        entity->Init();
        // Standing up, hung so its top edge is within reach of the yard.
        AZ::TransformBus::Event(
            entity->GetId(), &AZ::TransformBus::Events::SetWorldTM,
            AZ::Transform::CreateFromQuaternionAndTranslation(
                AZ::Quaternion::CreateRotationX(AZ::Constants::HalfPi), AZ::Vector3(0.0f, 0.0f, 7.0f)));
        entity->Activate();

        TickAndSimulate(1.0f);

        AZ::Aabb bounds = AZ::Aabb::CreateNull();
        JoltSoftBodyRequestBus::EventResult(bounds, entity->GetId(), &JoltSoftBodyRequests::GetWorldBounds);
        ASSERT_TRUE(bounds.IsValid());

        // Still standing in the plane its rotation put it in. Asserting on the standing
        // dimension specifically, not on "some two dimensions survived": a cloth that
        // collapses back into its *unrotated* layout is still 2 m by 2 m, so a looser
        // check calls that healthy - which is exactly how the first version of this test
        // passed while the demo's sail was lying flat on its back.
        const AZ::Vector3 extents = bounds.GetExtents();
        EXPECT_NEAR(extents.GetX(), 2.0f, 0.3f) << "the rigged cloth lost its width";
        EXPECT_GT(extents.GetZ(), 1.4f) << "the rigged cloth collapsed out of the plane it was rotated into";

        entity->Deactivate();
        yard->Deactivate();
    }

    TEST_F(JoltSoftBodyAttachmentTests, AClothParentedToSomethingElseStillStandsUp)
    {
        // Cloth hung off a vehicle is authored as a child of it, so the rotation that
        // stands the sheet up is a *local* one and has to compose through the parent. This
        // was filed as a bug on the strength of a demo measurement - worth a test either
        // way, since the demo could only ever say "something in this scene is wrong".
        auto parent = AZStd::make_unique<AZ::Entity>("Boat");
        parent->CreateComponent<AzFramework::TransformComponent>();
        parent->Init();
        parent->Activate();
        AZ::TransformBus::Event(
            parent->GetId(), &AZ::TransformBus::Events::SetWorldTM,
            AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, -2.0f, 2.4f)));

        auto child = AZStd::make_unique<AZ::Entity>("Sail");
        auto* childTransform = child->CreateComponent<AzFramework::TransformComponent>();
        childTransform->SetParent(parent->GetId());
        auto* softBody = child->CreateComponent<JoltSoftBodyComponent>();

        JoltSoftBodySettings& settings = softBody->GetSettings();
        settings.m_shape = JoltSoftBodyShape::Cloth;
        settings.m_pinning = JoltSoftBodyPinning::TopEdge;
        settings.m_size = AZ::Vector3(2.0f, 2.0f, 1.0f);
        settings.m_resolution = 6;

        child->Init();
        // Local: a quarter turn about x, offset up and leeward - exactly how a sail is
        // authored against a hull.
        AZ::TransformBus::Event(
            child->GetId(), &AZ::TransformBus::Events::SetLocalTM,
            AZ::Transform::CreateFromQuaternionAndTranslation(
                AZ::Quaternion::CreateRotationX(AZ::Constants::HalfPi), AZ::Vector3(0.0f, 0.3f, 2.2f)));
        child->Activate();

        AZ::Aabb bounds = AZ::Aabb::CreateNull();
        JoltSoftBodyRequestBus::EventResult(bounds, child->GetId(), &JoltSoftBodyRequests::GetWorldBounds);
        ASSERT_TRUE(bounds.IsValid());

        // Standing, and standing where the parent puts it.
        const AZ::Vector3 extents = bounds.GetExtents();
        EXPECT_NEAR(extents.GetX(), 2.0f, 0.1f);
        EXPECT_NEAR(extents.GetZ(), 2.0f, 0.1f) << "the parented cloth is lying flat";
        EXPECT_LT(extents.GetY(), 0.1f);
        EXPECT_TRUE(bounds.GetCenter().IsClose(AZ::Vector3(0.0f, -1.7f, 4.6f), 0.1f))
            << "the parented cloth was not built where its parent puts it";

        child->Deactivate();
        parent->Deactivate();
    }

    TEST_F(JoltSoftBodyAttachmentTests, TurningATargetSwingsTheClothRoundWithIt)
    {
        // Trimming a sail is turning the spar it is bent to, so an attachment has to carry
        // a target's rotation and not just its position. Everything else here has only
        // ever moved a target in a straight line, which the inverse binds would survive
        // even if the rotation were being dropped.
        auto yard = CreateYard(AZ::Vector3(0.0f, 0.0f, 8.0f));
        auto cloth = CreateHangingCloth(AZ::Vector3(0.0f, 0.0f, 8.0f), yard->GetId(), AZ::EntityId());

        TickAndSimulate(0.2f);
        const AZ::Vector3 centreBefore = ClothCentre(cloth->GetId());

        // A quarter turn about z: the cloth hangs on the other axis afterwards.
        AZ::TransformBus::Event(
            yard->GetId(), &AZ::TransformBus::Events::SetWorldTM,
            AZ::Transform::CreateFromQuaternionAndTranslation(
                AZ::Quaternion::CreateRotationZ(AZ::Constants::HalfPi), AZ::Vector3(0.0f, 0.0f, 8.0f)));
        TickAndSimulate(1.0f);

        AZ::Aabb bounds = AZ::Aabb::CreateNull();
        JoltSoftBodyRequestBus::EventResult(bounds, cloth->GetId(), &JoltSoftBodyRequests::GetWorldBounds);
        ASSERT_TRUE(bounds.IsValid());

        // The sheet was a metre wide along x and is now a metre wide along y.
        const AZ::Vector3 extents = bounds.GetExtents();
        EXPECT_GT(extents.GetY(), extents.GetX())
            << "the cloth did not turn with its target - extents " << extents.GetX() << ", " << extents.GetY();
        // And it stayed where the target is rather than being flung.
        EXPECT_LT(ClothCentre(cloth->GetId()).GetDistance(centreBefore), 1.5f);

        cloth->Deactivate();
        yard->Deactivate();
    }

    TEST_F(JoltSoftBodyAttachmentTests, ASailFastenedAtHeadAndFootStaysStandingBetweenThem)
    {
        // The demo's rig, reproduced: a standing sail bent to a yard above and sheeted to
        // a boom below. Two fastenings rather than one, which is what stops a sail
        // flogging - and what a single-target test cannot cover.
        auto yard = CreateYard(AZ::Vector3(0.0f, 0.0f, 8.0f));
        auto boom = CreateYard(AZ::Vector3(0.0f, 0.0f, 6.0f));

        auto entity = AZStd::make_unique<AZ::Entity>("RiggedSail");
        entity->CreateComponent<AzFramework::TransformComponent>();
        auto* softBody = entity->CreateComponent<JoltSoftBodyComponent>();

        JoltSoftBodySettings& settings = softBody->GetSettings();
        settings.m_shape = JoltSoftBodyShape::Cloth;
        settings.m_pinning = JoltSoftBodyPinning::None;
        settings.m_size = AZ::Vector3(2.0f, 2.0f, 1.0f);
        settings.m_resolution = 10;
        settings.m_mass = 2.0f;
        settings.m_allowSleeping = false;

        auto* attachment = entity->CreateComponent<JoltSoftBodyAttachmentComponent>();
        for (const AZ::EntityId& spar : { yard->GetId(), boom->GetId() })
        {
            JoltSoftBodyAttachTarget target;
            target.m_entity = spar;
            target.m_attachDistance = 0.4f;
            attachment->GetTargets().push_back(target);
        }
        entity->Init();
        // Standing, spanning exactly the two spars: head at z=8, foot at z=6.
        AZ::TransformBus::Event(
            entity->GetId(), &AZ::TransformBus::Events::SetWorldTM,
            AZ::Transform::CreateFromQuaternionAndTranslation(
                AZ::Quaternion::CreateRotationX(AZ::Constants::HalfPi), AZ::Vector3(0.0f, 0.0f, 7.0f)));
        entity->Activate();

        TickAndSimulate(1.0f);

        AZ::Aabb bounds = AZ::Aabb::CreateNull();
        JoltSoftBodyRequestBus::EventResult(bounds, entity->GetId(), &JoltSoftBodyRequests::GetWorldBounds);
        ASSERT_TRUE(bounds.IsValid());

        const AZ::Vector3 extents = bounds.GetExtents();
        EXPECT_NEAR(extents.GetX(), 2.0f, 0.3f) << "the sail lost its width";
        EXPECT_GT(extents.GetZ(), 1.4f) << "the sail collapsed out of the plane it was rotated into";

        entity->Deactivate();
        boom->Deactivate();
        yard->Deactivate();
    }

    TEST_F(JoltSoftBodyAttachmentTests, AClothStandsUpWhenItsEntityIsRotated)
    {
        // A cloth is generated in its entity's local XY plane, so standing one up - a
        // sail, a banner, a curtain - is done by rotating the entity. If that rotation is
        // dropped the sheet stays flat on its back, and a wind blowing across it is
        // perfectly edge-on: it catches nothing and looks like the wind is broken.
        auto entity = AZStd::make_unique<AZ::Entity>("StandingCloth");
        entity->CreateComponent<AzFramework::TransformComponent>();
        auto* softBody = entity->CreateComponent<JoltSoftBodyComponent>();

        JoltSoftBodySettings& settings = softBody->GetSettings();
        settings.m_shape = JoltSoftBodyShape::Cloth;
        settings.m_pinning = JoltSoftBodyPinning::TopEdge;
        settings.m_size = AZ::Vector3(2.0f, 2.0f, 1.0f);
        settings.m_resolution = 6;

        entity->Init();
        // A quarter turn about x: the cloth's local +y edge becomes the top, and its
        // faces come to look along y.
        AZ::TransformBus::Event(
            entity->GetId(), &AZ::TransformBus::Events::SetWorldTM,
            AZ::Transform::CreateFromQuaternionAndTranslation(
                AZ::Quaternion::CreateRotationX(AZ::Constants::HalfPi), AZ::Vector3(0.0f, 0.0f, 6.0f)));
        entity->Activate();

        AZ::Aabb bounds = AZ::Aabb::CreateNull();
        JoltSoftBodyRequestBus::EventResult(bounds, entity->GetId(), &JoltSoftBodyRequests::GetWorldBounds);
        ASSERT_TRUE(bounds.IsValid());

        const AZ::Vector3 extents = bounds.GetExtents();
        EXPECT_NEAR(extents.GetX(), 2.0f, 0.1f) << "the cloth lost its width";
        EXPECT_NEAR(extents.GetZ(), 2.0f, 0.1f) << "the cloth is lying flat instead of standing up";
        EXPECT_LT(extents.GetY(), 0.1f) << "the cloth is not in the plane its rotation put it in";

        entity->Deactivate();
    }

    TEST_F(JoltSoftBodyAttachmentTests, AKinematicBodyAnswersItsMassInsteadOfAsserting)
    {
        // Jolt's checked inverse-mass accessor asserts on any non-dynamic body, and
        // GetMass reached it. Found the expensive way: a force region asking a kinematic
        // yard its mass took the whole editor down. A kinematic body's mass is a fair
        // question; the answer is its configured mass.
        auto yard = CreateYard(AZ::Vector3(0.0f, 0.0f, 4.0f));
        SimulateSeconds(0.1f);

        float mass = -1.0f;
        Physics::RigidBodyRequestBus::EventResult(mass, yard->GetId(), &Physics::RigidBodyRequests::GetMass);
        EXPECT_GT(mass, 0.0f);

        yard->Deactivate();
    }

    TEST_F(JoltSoftBodyAttachmentTests, AttachedClothFollowsItsTargetInsteadOfAnchoringInPlace)
    {
        // The reason this component exists: a top-pinned cloth would stay at the world
        // position it was built at while the yard sailed away.
        auto yard = CreateYard(AZ::Vector3(0.0f, 0.5f, 6.0f));
        auto cloth = CreateHangingCloth(AZ::Vector3(0.0f, 0.0f, 6.0f), yard->GetId(), AZ::EntityId());

        TickAndSimulate(0.2f);
        const AZ::Vector3 centreBefore = ClothCentre(cloth->GetId());
        ASSERT_TRUE(centreBefore.IsFinite());

        // Sail the yard 3 m along x, kinematically, the way a boat carries its rig.
        AZ::TransformBus::Event(
            yard->GetId(), &AZ::TransformBus::Events::SetWorldTM,
            AZ::Transform::CreateTranslation(AZ::Vector3(3.0f, 0.5f, 6.0f)));
        TickAndSimulate(1.0f);

        const AZ::Vector3 centreAfter = ClothCentre(cloth->GetId());
        EXPECT_GT(centreAfter.GetX() - centreBefore.GetX(), 2.0f)
            << "the cloth stayed behind when its target moved";

        cloth->Deactivate();
        yard->Deactivate();
    }

    TEST_F(JoltSoftBodyAttachmentTests, TheWindOnAnAttachedSailPushesTheHullItIsRiggedTo)
    {
        // The other half of sailing: the boat must be driven by the same wind the sail
        // visibly catches. The hull here floats free of gravity so the only thing that
        // can move it is the canvas.
        ScopedTestWind wind;

        // The hull: a free dynamic box, gravity off.
        auto hull = AZStd::make_unique<AZ::Entity>("Hull");
        hull->CreateComponent<AzFramework::TransformComponent>();
        auto* hullBody = hull->CreateComponent<JoltRigidBodyComponent>();
        hullBody->GetConfiguration().m_gravityEnabled = false;
        // Light and explicit, so a one-square-metre sail's ~20 N moves it by an amount a
        // test can assert rather than a rounding error on a computed-density tonne.
        hullBody->GetConfiguration().m_computeMass = false;
        hullBody->GetConfiguration().m_mass = 10.0f;
        hull->CreateComponent<JoltBoxColliderComponent>();
        hull->Init();
        AZ::TransformBus::Event(
            hull->GetId(), &AZ::TransformBus::Events::SetWorldTM,
            AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, -20.0f, 2.0f)));
        hull->Activate();

        // The yard above it, and the sail hung from the yard, face-on to a +z wind.
        auto yard = CreateYard(AZ::Vector3(0.0f, 0.5f, 8.0f));
        auto sail = CreateHangingCloth(AZ::Vector3(0.0f, 0.0f, 8.0f), yard->GetId(), hull->GetId());
        JoltSoftBodyRequestBus::Event(sail->GetId(), &JoltSoftBodyRequests::SetGravityFactor, 0.0f);

        TickAndSimulate(0.2f); // attach in still air

        AZ::Vector3 velocityBefore = AZ::Vector3::CreateZero();
        Physics::RigidBodyRequestBus::EventResult(
            velocityBefore, hull->GetId(), &Physics::RigidBodyRequests::GetLinearVelocity);

        wind.m_wind = AZ::Vector3(0.0f, 0.0f, 6.0f);
        TickAndSimulate(1.0f);

        AZ::Vector3 velocityAfter = AZ::Vector3::CreateZero();
        Physics::RigidBodyRequestBus::EventResult(
            velocityAfter, hull->GetId(), &Physics::RigidBodyRequests::GetLinearVelocity);

        EXPECT_GT(velocityAfter.GetZ() - velocityBefore.GetZ(), 0.3f)
            << "the sail caught wind and the hull felt none of it";

        sail->Deactivate();
        yard->Deactivate();
        hull->Deactivate();
    }

    TEST_F(JoltSoftBodyAttachmentTests, AClothNowhereNearItsTargetStaysFreeInsteadOfTeleporting)
    {
        // A misplaced rig should read as "nothing attached" - warned once - not as cloth
        // snapping across the level to its target.
        auto yard = CreateYard(AZ::Vector3(30.0f, 0.0f, 6.0f));

        AZStd::unique_ptr<AZ::Entity> cloth;
        {
            // The warning is expected exactly once, when the bind attempt gives up.
            JoltWarningCatcher warnings;
            cloth = CreateHangingCloth(AZ::Vector3(0.0f, 0.0f, 6.0f), yard->GetId(), AZ::EntityId());
            TickAndSimulate(0.2f);
            EXPECT_TRUE(warnings.ContainsWarningWith("nothing attached"));
        }

        const AZ::Vector3 centre = ClothCentre(cloth->GetId());
        EXPECT_LT(centre.GetX(), 5.0f) << "the cloth teleported towards a target it never touched";

        // And an out-of-reach rig must not weld anything: the cloth still falls.
        SimulateSeconds(0.5f);
        EXPECT_LT(ClothCentre(cloth->GetId()).GetZ(), centre.GetZ() - 0.5f);

        cloth->Deactivate();
        yard->Deactivate();
    }

    TEST_F(JoltSoftBodySkinningBusTests, JointsGivenInWorldSpaceMoveTheClothToWhereTheyActuallyAre)
    {
        // The body deliberately does not sit at the origin. Jolt wants joint matrices
        // relative to the body's centre of mass, so a caller handing over world transforms
        // without that conversion would move the cloth by the body's own offset on top of
        // the joint's - which is why the conversion is on this side of the bus rather than
        // left to whoever calls it.
        const AZ::Vector3 clothPosition(5.0f, 0.0f, 2.0f);
        auto entity = CreateCloth(clothPosition);
        const AZ::EntityId entityId = entity->GetId();

        const AZ::Vector3 startCentre = ClothCentre(entityId);
        ASSERT_TRUE(startCentre.IsClose(clothPosition, 0.5f)) << "the cloth was not built where it was placed";

        // Bound at the pose it is in: one joint sitting on the cloth, in world space.
        const AZ::Transform bindPose = AZ::Transform::CreateTranslation(startCentre);
        JoltSoftBodyRequestBus::Event(
            entityId, &JoltSoftBodyRequests::SetSkinningData, AZStd::vector<AZ::Transform>{ bindPose },
            SkinAllToOneJoint(entityId));

        bool hasSkinning = false;
        JoltSoftBodyRequestBus::EventResult(hasSkinning, entityId, &JoltSoftBodyRequests::HasSkinningData);
        ASSERT_TRUE(hasSkinning);

        // Move the joint two metres along x, in world space, and let it settle.
        const AZ::Transform movedJoint = AZ::Transform::CreateTranslation(startCentre + AZ::Vector3(2.0f, 0.0f, 0.0f));
        bool updated = false;
        JoltSoftBodyRequestBus::EventResult(
            updated, entityId, &JoltSoftBodyRequests::UpdateSkinnedJoints, AZStd::vector<AZ::Transform>{ movedJoint },
            /*hardSkinAll*/ true);
        ASSERT_TRUE(updated);

        for (int step = 0; step < 60; ++step)
        {
            JoltSoftBodyRequestBus::Event(
                entityId, &JoltSoftBodyRequests::UpdateSkinnedJoints, AZStd::vector<AZ::Transform>{ movedJoint },
                /*hardSkinAll*/ false);
            SimulateSeconds(1.0f / 60.0f);
        }

        // Two metres from where it started, not seven: the body's own offset must not be
        // applied twice.
        const AZ::Vector3 endCentre = ClothCentre(entityId);
        EXPECT_NEAR(endCentre.GetX(), startCentre.GetX() + 2.0f, 0.3f);
        EXPECT_NEAR(endCentre.GetZ(), startCentre.GetZ(), 0.3f);

        entity->Deactivate();
    }

    TEST_F(JoltSoftBodySkinningBusTests, SkinnedClothHangsWhereTheSkeletonPutsItInsteadOfFalling)
    {
        // Nothing is pinned, so without the constraints holding it to the joint this
        // sheet is in free fall.
        auto entity = CreateCloth(AZ::Vector3(0.0f, 0.0f, 3.0f));
        const AZ::EntityId entityId = entity->GetId();

        const AZ::Vector3 startCentre = ClothCentre(entityId);
        const AZ::Transform bindPose = AZ::Transform::CreateTranslation(startCentre);
        JoltSoftBodyRequestBus::Event(
            entityId, &JoltSoftBodyRequests::SetSkinningData, AZStd::vector<AZ::Transform>{ bindPose },
            SkinAllToOneJoint(entityId));

        for (int step = 0; step < 120; ++step)
        {
            JoltSoftBodyRequestBus::Event(
                entityId, &JoltSoftBodyRequests::UpdateSkinnedJoints, AZStd::vector<AZ::Transform>{ bindPose },
                /*hardSkinAll*/ step == 0);
            SimulateSeconds(1.0f / 60.0f);
        }

        // Two seconds of gravity would have taken it about 20 m down.
        EXPECT_NEAR(ClothCentre(entityId).GetZ(), startCentre.GetZ(), 0.3f);

        entity->Deactivate();
    }

    TEST_F(JoltComponentBodyCreationTests, AnAssetMeshColliderGivesGeometryToEitherKindOfBody)
    {
        // A .joltmesh collider reaches its body differently from every other collider:
        // the shapes are not the component's own, they are expanded out of an asset when
        // the body asks for them. The static and dynamic bodies ask at different moments,
        // so both are pinned here - by the body's world bounds, which is the one reading a
        // resting-body probe cannot confound (a hull that happens to be a sphere lets a
        // ball roll off its apex, which looks exactly like having no collider at all).
        // CreateMeshAsset goes through the asset manager, which refuses to make one
        // without a handler for the type - the same handler the system component
        // registers in production, and which the test environment does not run.
        Pipeline::JoltMeshAssetHandler assetHandler;
        assetHandler.Register();

        // The asset expands into cloned configurations that live and die inside the gem,
        // so the shapes cached on them can only be released through the bus.
        ScopedNativeMeshReleaser nativeMeshReleaser;

        const AZStd::vector<AZ::u8> blob = MakeUnitCubeConvexBlob();

        auto makeAsset = [&blob]()
        {
            auto cooked = AZStd::make_shared<Physics::CookedMeshShapeConfiguration>();
            cooked->SetCookedMeshData(
                blob.data(), blob.size(), Physics::CookedMeshShapeConfiguration::MeshType::Convex);

            Pipeline::JoltMeshAssetData assetData;
            assetData.m_colliderShapes.emplace_back(
                AZStd::make_shared<Pipeline::JoltAssetColliderConfiguration>(), cooked);
            assetData.m_materialIndexPerShape = { 0 };
            return assetData.CreateMeshAsset();
        };

        auto halfExtentsOfBodyFor = [this](AZ::Entity* entity)
        {
            const auto [foundSceneHandle, bodyHandle] =
                m_system->FindAttachedBodyHandleFromEntityId(entity->GetId());
            EXPECT_NE(bodyHandle, AzPhysics::InvalidSimulatedBodyHandle);
            if (bodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
            {
                return AZ::Vector3::CreateZero();
            }
            return m_scene->GetSimulatedBodyFromHandle(bodyHandle)->GetAabb().GetExtents() * 0.5f;
        };

        {
            auto entity = AZStd::make_unique<AZ::Entity>("AssetMeshStaticBody");
            entity->CreateComponent<AzFramework::TransformComponent>();
            auto* collider = entity->CreateComponent<JoltMeshColliderComponent>();
            entity->CreateComponent<JoltStaticRigidBodyComponent>();
            collider->GetShapeConfiguration().m_asset = makeAsset();
            entity->Init();
            entity->Activate();

            EXPECT_TRUE(halfExtentsOfBodyFor(entity.get()).IsClose(AZ::Vector3(0.5f), 0.01f))
                << "a static body took no geometry from its asset mesh collider";
            entity->Deactivate();
            // The scene defers body deletion to the next step, and the body owns the
            // expanded configurations - so without a step the shapes cached on them
            // outlive this test rather than being released by it.
            SimulateSeconds(0.05f);
        }

        {
            auto entity = AZStd::make_unique<AZ::Entity>("AssetMeshDynamicBody");
            entity->CreateComponent<AzFramework::TransformComponent>();
            auto* collider = entity->CreateComponent<JoltMeshColliderComponent>();
            entity->CreateComponent<JoltRigidBodyComponent>();
            collider->GetShapeConfiguration().m_asset = makeAsset();
            entity->Init();
            entity->Activate();

            EXPECT_TRUE(halfExtentsOfBodyFor(entity.get()).IsClose(AZ::Vector3(0.5f), 0.01f))
                << "a dynamic body took no geometry from its asset mesh collider";
            entity->Deactivate();
            SimulateSeconds(0.05f);
        }

        // And the shapes cached on those expanded configurations were handed back
        // rather than leaked - the failure this would otherwise surface as is a suite
        // that reports every test passing and then fails at teardown.
        EXPECT_GT(nativeMeshReleaser.m_releases, 0)
            << "no cooked shape was released, so the expansion leaked its geometry";
        assetHandler.Unregister();
    }

    TEST_F(JoltComponentBodyCreationTests, ScaledColliderGeometryMatchesTheShapeTypesLimits)
    {
        // Above pins the scale reaching the shape configuration; this pins what the
        // native shape does with it, read off the created body's world bounds. Jolt
        // scales a shape by wrapping it in a JPH::ScaledShape, and not every shape
        // accepts every scale (JPH::Shape::IsValidScale): boxes and meshes take a
        // non-uniform scale, spheres and capsules only a uniform one.
        auto halfExtentsOfBodyFor = [this](AZ::Entity* entity)
        {
            const auto [foundSceneHandle, bodyHandle] =
                m_system->FindAttachedBodyHandleFromEntityId(entity->GetId());
            EXPECT_NE(bodyHandle, AzPhysics::InvalidSimulatedBodyHandle);
            if (bodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
            {
                return AZ::Vector3::CreateZero();
            }
            return m_scene->GetSimulatedBodyFromHandle(bodyHandle)->GetAabb().GetExtents() * 0.5f;
        };

        {
            // Default 1 m box, scaled per axis: half extents are the scale halved.
            auto entity = MakeScaledEntity("GeometryBoxEntity", AZ::Vector3(2.0f, 3.0f, 4.0f));
            entity->CreateComponent<JoltBoxColliderComponent>();
            entity->CreateComponent<JoltRigidBodyComponent>();
            entity->Init();
            entity->Activate();
            EXPECT_TRUE(halfExtentsOfBodyFor(entity.get()).IsClose(AZ::Vector3(1.0f, 1.5f, 2.0f), 0.01f));
            entity->Deactivate();
        }

        {
            // Default 0.5 m radius sphere at uniform scale 2 -> 1 m radius.
            auto entity = MakeScaledEntity("GeometrySphereEntity", AZ::Vector3(2.0f));
            entity->CreateComponent<JoltSphereColliderComponent>();
            entity->CreateComponent<JoltRigidBodyComponent>();
            entity->Init();
            entity->Activate();
            EXPECT_TRUE(halfExtentsOfBodyFor(entity.get()).IsClose(AZ::Vector3(1.0f), 0.01f));
            entity->Deactivate();
        }

        {
            // Default capsule (1 m tall, 0.25 m radius) at uniform scale 2 -> 2 m tall,
            // 0.5 m radius, still z-up.
            auto entity = MakeScaledEntity("GeometryCapsuleEntity", AZ::Vector3(2.0f));
            entity->CreateComponent<JoltCapsuleColliderComponent>();
            entity->CreateComponent<JoltRigidBodyComponent>();
            entity->Init();
            entity->Activate();
            EXPECT_TRUE(halfExtentsOfBodyFor(entity.get()).IsClose(AZ::Vector3(0.5f, 0.5f, 1.0f), 0.01f));
            entity->Deactivate();
        }

        {
            // A baked convex mesh is a hull, which takes a non-uniform scale like a box.
            auto entity = MakeScaledEntity("GeometryBakedMeshEntity", AZ::Vector3(2.0f, 3.0f, 4.0f));
            auto* collider = entity->CreateComponent<JoltBakedMeshColliderComponent>();
            const AZStd::vector<AZ::u8> blob = MakeUnitCubeConvexBlob();
            collider->GetShapeConfiguration().SetCookedMeshData(
                blob.data(), blob.size(), Physics::CookedMeshShapeConfiguration::MeshType::Convex);
            entity->CreateComponent<JoltRigidBodyComponent>();
            entity->Init();
            entity->Activate();
            EXPECT_TRUE(halfExtentsOfBodyFor(entity.get()).IsClose(AZ::Vector3(1.0f, 1.5f, 2.0f), 0.01f));
            entity->Deactivate();
            ReleaseCachedNativeMesh(collider->GetShapeConfiguration());
        }

        {
            // A sphere has no non-uniform scale in Jolt (JPH::SphereShape::IsValidScale
            // requires a uniform one), so CreateJoltShapeFromConfig clamps to what the
            // shape does accept - the mean of the components, 3 here - and warns. Without
            // the clamp Jolt asserts inside the shape, so this case is worth pinning.
            JoltWarningCatcher warnings;

            auto entity = MakeScaledEntity("GeometryNonUniformSphereEntity", AZ::Vector3(2.0f, 3.0f, 4.0f));
            entity->CreateComponent<JoltSphereColliderComponent>();
            entity->CreateComponent<JoltRigidBodyComponent>();
            entity->Init();
            entity->Activate();
            EXPECT_TRUE(halfExtentsOfBodyFor(entity.get()).IsClose(AZ::Vector3(1.5f), 0.01f));
            EXPECT_TRUE(warnings.ContainsWarningWith("cannot take the non-uniform scale"));
            entity->Deactivate();
        }

        {
            // Same for the capsule, whose z-up rotation wrapper passes the scale through
            // to the capsule: uniform 3 gives a 3 m tall, 0.75 m radius capsule.
            JoltWarningCatcher warnings;

            auto entity = MakeScaledEntity("GeometryNonUniformCapsuleEntity", AZ::Vector3(2.0f, 3.0f, 4.0f));
            entity->CreateComponent<JoltCapsuleColliderComponent>();
            entity->CreateComponent<JoltRigidBodyComponent>();
            entity->Init();
            entity->Activate();
            EXPECT_TRUE(halfExtentsOfBodyFor(entity.get()).IsClose(AZ::Vector3(0.75f, 0.75f, 1.5f), 0.01f));
            EXPECT_TRUE(warnings.ContainsWarningWith("cannot take the non-uniform scale"));
            entity->Deactivate();
        }
    }

    TEST_F(JoltComponentBodyCreationTests, RotatedColliderOnANonUniformlyScaledEntityScalesInEntitySpace)
    {
        // The entity's non-uniform scale lives in entity space, outside any collider
        // rotation: the render mesh of a rotated child squashes along the *entity's* axis.
        // Applying the scale in the shape's local frame (inside the rotation) squashes
        // along whichever axis the rotation maps there instead - a box rotated 90 degrees
        // about X with entity scale z=0.5 came out squashed along entity Y.
        auto entity = MakeScaledEntity("RotatedScaledBoxEntity", AZ::Vector3(1.0f, 1.0f, 0.5f));
        auto* collider = entity->CreateComponent<JoltBoxColliderComponent>();
        collider->GetShapeConfiguration().m_dimensions = AZ::Vector3(2.0f, 4.0f, 6.0f);
        collider->GetColliderConfiguration().m_rotation = AZ::Quaternion::CreateRotationX(AZ::Constants::HalfPi);
        entity->CreateComponent<JoltRigidBodyComponent>();
        entity->Init();
        entity->Activate();

        // Half extents (1,2,3), rotated 90 about X -> spans (1,3,2) in entity space,
        // then z halves: (1,3,1). The wrong frame gives (1,1.5,2).
        const auto [foundSceneHandle, bodyHandle] =
            m_system->FindAttachedBodyHandleFromEntityId(entity->GetId());
        ASSERT_NE(bodyHandle, AzPhysics::InvalidSimulatedBodyHandle);
        const AZ::Vector3 halfExtents =
            m_scene->GetSimulatedBodyFromHandle(bodyHandle)->GetAabb().GetExtents() * 0.5f;
        EXPECT_TRUE(halfExtents.IsClose(AZ::Vector3(1.0f, 3.0f, 1.0f), 0.01f))
            << "half extents: " << halfExtents.GetX() << ", " << halfExtents.GetY() << ", " << halfExtents.GetZ();

        entity->Deactivate();
    }

    TEST_F(JoltComponentBodyCreationTests, OffAxisRotatedColliderApproximatesTheScaleAlongItsOwnAxes)
    {
        // A rotation that does not map axes onto axes cannot take a non-uniform scale
        // exactly - that would shear. The mesh pipeline's primitive fit produces such
        // rotations routinely (the PCA frame tilts a few degrees on tapered geometry),
        // and clamping to a uniform scale there turned a plank's 0.1 thickness scale
        // into 0.7 in every direction. Instead the scale is approximated along the
        // collider's own axes, which degrades continuously: 3 degrees off gives a
        // shape a few percent off, not a different shape.
        JoltWarningCatcher warnings;

        auto entity = MakeScaledEntity("OffAxisRotatedBoxEntity", AZ::Vector3(1.0f, 1.0f, 0.5f));
        auto* collider = entity->CreateComponent<JoltBoxColliderComponent>();
        collider->GetShapeConfiguration().m_dimensions = AZ::Vector3(2.0f, 4.0f, 6.0f);
        collider->GetColliderConfiguration().m_rotation = AZ::Quaternion::CreateRotationX(AZ::DegToRad(93.0f));
        entity->CreateComponent<JoltRigidBodyComponent>();
        entity->Init();
        entity->Activate();

        // Per-axis image lengths under the scale: (1, 0.502, 0.999); the scaled local
        // box spans (1, 3.045, 1.160) in entity space. The uniform-mean clamp gave
        // (0.83, 2.58, 1.80) and the exact 90-degree answer is (1, 3.10, 1.08) - the
        // approximation must land beside the latter.
        const auto [foundSceneHandle, bodyHandle] =
            m_system->FindAttachedBodyHandleFromEntityId(entity->GetId());
        ASSERT_NE(bodyHandle, AzPhysics::InvalidSimulatedBodyHandle);
        const AZ::Vector3 halfExtents =
            m_scene->GetSimulatedBodyFromHandle(bodyHandle)->GetAabb().GetExtents() * 0.5f;
        EXPECT_TRUE(halfExtents.IsClose(AZ::Vector3(1.0f, 3.045f, 1.16f), 0.02f))
            << "half extents: " << halfExtents.GetX() << ", " << halfExtents.GetY() << ", " << halfExtents.GetZ();
        EXPECT_TRUE(warnings.ContainsWarningWith("approximating"));

        entity->Deactivate();
    }

    TEST_F(JoltComponentBodyCreationTests, RotatedColliderInACompoundScalesInEntitySpaceToo)
    {
        // Same as above through the multi-collider compound path: an unrotated cube at
        // the origin plus the rotated box offset along x.
        auto entity = MakeScaledEntity("RotatedScaledCompoundEntity", AZ::Vector3(1.0f, 1.0f, 0.5f));
        entity->CreateComponent<JoltBoxColliderComponent>();
        auto* rotated = entity->CreateComponent<JoltBoxColliderComponent>();
        rotated->GetShapeConfiguration().m_dimensions = AZ::Vector3(2.0f, 4.0f, 6.0f);
        rotated->GetColliderConfiguration().m_rotation = AZ::Quaternion::CreateRotationX(AZ::Constants::HalfPi);
        rotated->GetColliderConfiguration().m_position = AZ::Vector3(5.0f, 0.0f, 0.0f);
        entity->CreateComponent<JoltRigidBodyComponent>();
        entity->Init();
        entity->Activate();

        // Cube: half (0.5,0.5,0.25) at the origin. Rotated box: half (1,3,1) at (5,0,0).
        // Union: x in [-0.5,6], y in [-3,3], z in [-1,1].
        const auto [foundSceneHandle, bodyHandle] =
            m_system->FindAttachedBodyHandleFromEntityId(entity->GetId());
        ASSERT_NE(bodyHandle, AzPhysics::InvalidSimulatedBodyHandle);
        const AZ::Aabb aabb = m_scene->GetSimulatedBodyFromHandle(bodyHandle)->GetAabb();
        EXPECT_TRUE(aabb.GetMin().IsClose(AZ::Vector3(-0.5f, -3.0f, -1.0f), 0.01f))
            << "min: " << aabb.GetMin().GetX() << ", " << aabb.GetMin().GetY() << ", " << aabb.GetMin().GetZ();
        EXPECT_TRUE(aabb.GetMax().IsClose(AZ::Vector3(6.0f, 3.0f, 1.0f), 0.01f))
            << "max: " << aabb.GetMax().GetX() << ", " << aabb.GetMax().GetY() << ", " << aabb.GetMax().GetZ();

        entity->Deactivate();
    }

    TEST_F(JoltComponentBodyCreationTests, ColliderDiagnosticsNameTheEntityTheyCameFrom)
    {
        // Shape diagnostics fire several calls below the component that knows the entity,
        // so the body's name travels down with the shape configuration. Without it, a
        // warning about a collider is unactionable in a level of any size.
        JoltWarningCatcher warnings;

        auto entity = AZStd::make_unique<AZ::Entity>("SquashedCapsuleEntity");
        entity->CreateComponent<AzFramework::TransformComponent>();
        auto* collider = entity->CreateComponent<JoltCapsuleColliderComponent>();
        // Height below twice the radius is not a capsule; the shape degrades to a sphere.
        collider->GetShapeConfiguration().m_height = 0.5f;
        collider->GetShapeConfiguration().m_radius = 0.5f;
        entity->CreateComponent<JoltRigidBodyComponent>();
        entity->Init();
        entity->Activate();

        EXPECT_TRUE(warnings.ContainsWarningWith("SquashedCapsuleEntity"));
        EXPECT_TRUE(warnings.ContainsWarningWith("less than twice its radius"));

        entity->Deactivate();
    }

    // A collider whose geometry comes from an LmbrCentral shape component. Polygon Prism
    // is the case that matters: box-like shapes can be re-authored with this gem's own
    // primitives, but an extruded prism - the stock O3DE way to draw a kill volume or a
    // level blocker - had no representation in this backend at all.
    class JoltShapeColliderTests : public JoltComponentBodyCreationTests
    {
    protected:
        //! Stands in for an LmbrCentral polygon prism shape component.
        class MockPolygonPrismShape
            : public LmbrCentral::ShapeComponentRequestsBus::Handler
            , public LmbrCentral::PolygonPrismShapeComponentRequestBus::Handler
        {
        public:
            MockPolygonPrismShape(AZ::EntityId entityId, const AZStd::vector<AZ::Vector2>& footprint, float height)
                : m_prism(AZStd::make_shared<AZ::PolygonPrism>())
            {
                m_prism->SetHeight(height);
                for (const AZ::Vector2& vertex : footprint)
                {
                    m_prism->m_vertexContainer.AddVertex(vertex);
                }
                LmbrCentral::ShapeComponentRequestsBus::Handler::BusConnect(entityId);
                LmbrCentral::PolygonPrismShapeComponentRequestBus::Handler::BusConnect(entityId);
            }

            ~MockPolygonPrismShape() override
            {
                LmbrCentral::PolygonPrismShapeComponentRequestBus::Handler::BusDisconnect();
                LmbrCentral::ShapeComponentRequestsBus::Handler::BusDisconnect();
            }

            AZ::Crc32 GetShapeType() const override
            {
                return AZ_CRC_CE("PolygonPrism");
            }
            AZ::Aabb GetEncompassingAabb() const override
            {
                return AZ::Aabb::CreateNull();
            }
            void GetTransformAndLocalBounds(AZ::Transform& transform, AZ::Aabb& bounds) const override
            {
                transform = AZ::Transform::CreateIdentity();
                bounds = AZ::Aabb::CreateNull();
            }
            bool IsPointInside([[maybe_unused]] const AZ::Vector3& point) const override
            {
                return false;
            }
            float DistanceSquaredFromPoint([[maybe_unused]] const AZ::Vector3& point) const override
            {
                return 0.0f;
            }

            AZ::PolygonPrismPtr GetPolygonPrism() override
            {
                return m_prism;
            }
            void SetHeight(float height) override
            {
                m_prism->SetHeight(height);
            }
            bool GetVertex([[maybe_unused]] size_t index, AZ::Vector2& vertex) const override
            {
                vertex = AZ::Vector2::CreateZero();
                return false;
            }
            void AddVertex([[maybe_unused]] const AZ::Vector2& vertex) override {}
            bool UpdateVertex([[maybe_unused]] size_t index, [[maybe_unused]] const AZ::Vector2& vertex) override
            {
                return false;
            }
            bool InsertVertex([[maybe_unused]] size_t index, [[maybe_unused]] const AZ::Vector2& vertex) override
            {
                return false;
            }
            bool RemoveVertex([[maybe_unused]] size_t index) override
            {
                return false;
            }
            void SetVertices([[maybe_unused]] const AZStd::vector<AZ::Vector2>& vertices) override {}
            void ClearVertices() override {}
            size_t Size() const override
            {
                return m_prism->m_vertexContainer.Size();
            }
            bool Empty() const override
            {
                return m_prism->m_vertexContainer.Empty();
            }

            AZ::PolygonPrismPtr m_prism;
        };
    };

    TEST_F(JoltShapeColliderTests, APolygonPrismBecomesCollisionGeometry)
    {
        auto entity = AZStd::make_unique<AZ::Entity>("PrismCollider");
        entity->CreateComponent<AzFramework::TransformComponent>();
        entity->CreateComponent<JoltShapeColliderComponent>();
        entity->Init();

        // A 2x2 square footprint, extruded 3 m up.
        const AZStd::vector<AZ::Vector2> footprint = {
            AZ::Vector2(-1.0f, -1.0f), AZ::Vector2(1.0f, -1.0f), AZ::Vector2(1.0f, 1.0f), AZ::Vector2(-1.0f, 1.0f)
        };
        MockPolygonPrismShape shape(entity->GetId(), footprint, 3.0f);

        // Not activated: the component requires ShapeService, which only a real shape
        // component provides, and the conversion under test reads the shape buses the mock
        // answers rather than anything activation sets up.
        auto* collider = entity->FindComponent<JoltShapeColliderComponent>();
        ASSERT_NE(collider, nullptr);

        const AzPhysics::ShapeColliderPair pair = collider->GetShapeColliderPair();
        ASSERT_NE(pair.second, nullptr) << "the prism produced no shape configuration";
        EXPECT_EQ(pair.second->GetShapeType(), Physics::ShapeType::CookedMesh);

        // The cooked hull has to be the extruded solid, not the flat outline.
        const JPH::RefConst<JPH::Shape> nativeShape = JoltShapeUtils::CreateJoltShapeFromConfig(*pair.second);
        ASSERT_NE(nativeShape, nullptr);
        const JPH::AABox bounds = nativeShape->GetLocalBounds();
        EXPECT_NEAR(bounds.GetSize().GetX(), 2.0f, 0.05f);
        EXPECT_NEAR(bounds.GetSize().GetY(), 2.0f, 0.05f);
        EXPECT_NEAR(bounds.GetSize().GetZ(), 3.0f, 0.05f) << "the prism was not extruded";

        // Creating a shape from a cooked configuration caches the native shape on it with
        // a reference; in production JoltPhysicsSystemComponent::ReleaseNativeMeshObject
        // balances that, so the test does it here.
        if (auto* cooked = azdynamic_cast<Physics::CookedMeshShapeConfiguration*>(pair.second.get()))
        {
            if (auto* cachedMesh = static_cast<JPH::Shape*>(cooked->GetCachedNativeMesh()))
            {
                cachedMesh->Release();
                cooked->SetCachedNativeMesh(nullptr);
            }
        }
    }

} // namespace JoltPhysics
