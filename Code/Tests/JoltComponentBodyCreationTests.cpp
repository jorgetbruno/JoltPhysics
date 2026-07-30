#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <Clients/Components/JoltBakedMeshColliderComponent.h>
#include <Clients/Components/JoltBoxColliderComponent.h>
#include <Clients/Components/JoltCapsuleColliderComponent.h>
#include <Clients/Components/JoltColliderComponentBase.h>
#include <Clients/Components/JoltRigidBodyComponent.h>
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
#include <AzFramework/Physics/SystemBus.h>
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

} // namespace JoltPhysics
