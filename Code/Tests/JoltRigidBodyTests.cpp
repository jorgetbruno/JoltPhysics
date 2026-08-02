#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <System/JoltSystem.h>
#include <Scene/JoltScene.h>
#include <Shape/JoltMeshUtils.h>
#include <Shape/JoltShapeUtils.h>
#include <Configuration/JoltSettingsRegistryManager.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>

namespace JoltPhysics
{
    class JoltRigidBodyTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "RigidBodyTestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);
        }

        void TearDown() override
        {
            m_system->RemoveScene(m_sceneHandle);
            m_system->Shutdown();
            m_system.reset();
        }

        AzPhysics::RigidBody* CreateDynamicBox(const AZ::Vector3& position)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();

            AzPhysics::RigidBodyConfiguration config;
            config.m_position = position;
            config.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);
            auto handle = m_scene->AddSimulatedBody(&config);
            return static_cast<AzPhysics::RigidBody*>(m_scene->GetSimulatedBodyFromHandle(handle));
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

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle;
        AzPhysics::Scene* m_scene = nullptr;
    };

    TEST_F(JoltRigidBodyTests, SetMassUpdatesMassProperties)
    {
        auto* body = CreateDynamicBox(AZ::Vector3::CreateZero());
        ASSERT_NE(body, nullptr);

        body->SetMass(10.0f);
        EXPECT_NEAR(body->GetMass(), 10.0f, 0.01f);
        EXPECT_NEAR(body->GetInverseMass(), 0.1f, 0.001f);
    }

    TEST_F(JoltRigidBodyTests, DefaultInertiaMatchesUnitBox)
    {
        auto* body = CreateDynamicBox(AZ::Vector3::CreateZero());
        ASSERT_NE(body, nullptr);

        // A unit box computing its own mass weighs its volume times the default material
        // density: 1 m^3 at 1000 kg/m^3. Inertia follows: I = m/12 * (h^2 + d^2) = m/6.
        const float mass = body->GetMass();
        ASSERT_NEAR(mass, 1000.0f, 0.01f);

        const AZ::Matrix3x3 inertia = body->GetInertiaLocal();
        EXPECT_NEAR(inertia(0, 0), mass / 6.0f, 0.1f);
        EXPECT_NEAR(inertia(1, 1), mass / 6.0f, 0.1f);
        EXPECT_NEAR(inertia(2, 2), mass / 6.0f, 0.1f);

        const AZ::Matrix3x3 inverseInertia = body->GetInverseInertiaLocal();
        EXPECT_NEAR(inverseInertia(0, 0), 6.0f / mass, 0.001f);
    }

    TEST_F(JoltRigidBodyTests, MassComesFromVolumeWhenComputed)
    {
        // Compute Mass is the engine default and what PhysX does, so a body ported from
        // PhysX - which leaves m_mass at a meaningless 1 - has to end up weighing what its
        // geometry implies rather than one kilogram.
        auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        boxShape->m_dimensions = AZ::Vector3(2.0f, 3.0f, 4.0f); // 24 m^3

        AzPhysics::RigidBodyConfiguration config;
        config.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);
        ASSERT_TRUE(config.m_computeMass) << "the engine default this fix depends on has changed";
        ASSERT_NEAR(config.m_mass, 1.0f, 0.01f) << "the meaningless mass a PhysX body arrives with";

        auto* body = static_cast<AzPhysics::RigidBody*>(
            m_scene->GetSimulatedBodyFromHandle(m_scene->AddSimulatedBody(&config)));
        ASSERT_NE(body, nullptr);
        EXPECT_NEAR(body->GetMass(), 24.0f * 1000.0f, 1.0f);
    }

    TEST_F(JoltRigidBodyTests, AnAuthoredMassWinsOverTheGeometry)
    {
        auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();

        AzPhysics::RigidBodyConfiguration config;
        config.m_computeMass = false;
        config.m_mass = 7.0f;
        config.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);

        auto* body = static_cast<AzPhysics::RigidBody*>(
            m_scene->GetSimulatedBodyFromHandle(m_scene->AddSimulatedBody(&config)));
        ASSERT_NE(body, nullptr);
        EXPECT_NEAR(body->GetMass(), 7.0f, 0.01f);

        // Inertia is still derived from the shape, scaled to the authored mass.
        EXPECT_NEAR(body->GetInertiaLocal()(0, 0), 7.0f / 6.0f, 0.01f);
    }

    TEST_F(JoltRigidBodyTests, UpdateMassPropertiesAppliesOverridesWhenNotAskedToCompute)
    {
        // The engine documents each override as ignored when its COMPUTE flag is set, so
        // an override only lands when the matching flag is *absent*. This used to be read
        // backwards, which made the overrides work and computing impossible.
        auto* body = CreateDynamicBox(AZ::Vector3::CreateZero());
        ASSERT_NE(body, nullptr);

        body->UpdateMassProperties(
            AzPhysics::MassComputeFlags::NONE,
            AZ::Vector3::CreateZero(),
            AZ::Matrix3x3::CreateDiagonal(AZ::Vector3(1.0f, 2.0f, 3.0f)),
            5.0f);

        EXPECT_NEAR(body->GetMass(), 5.0f, 0.01f);

        const AZ::Matrix3x3 inertia = body->GetInertiaLocal();
        EXPECT_NEAR(inertia(0, 0), 1.0f, 0.01f);
        EXPECT_NEAR(inertia(1, 1), 2.0f, 0.01f);
        EXPECT_NEAR(inertia(2, 2), 3.0f, 0.01f);
    }

    TEST_F(JoltRigidBodyTests, UpdateMassPropertiesRecomputesFromGeometryByDefault)
    {
        auto* body = CreateDynamicBox(AZ::Vector3::CreateZero());
        ASSERT_NE(body, nullptr);

        body->SetMass(3.0f);
        ASSERT_NEAR(body->GetMass(), 3.0f, 0.01f);

        // A bare UpdateMassProperties() means "recompute everything from the shapes".
        // Inverted, it set the body to the parameter defaults instead: 1 kg, identity
        // inertia, zero centre of mass - quietly destroying real mass properties.
        body->UpdateMassProperties();

        EXPECT_NEAR(body->GetMass(), 1000.0f, 1.0f);
        EXPECT_NEAR(body->GetInertiaLocal()(0, 0), 1000.0f / 6.0f, 1.0f);
    }

    TEST_F(JoltRigidBodyTests, CenterOfMassOffsetShiftsMassFrame)
    {
        auto* body = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 10.0f));
        ASSERT_NE(body, nullptr);

        body->SetCenterOfMassOffset(AZ::Vector3(0.0f, 0.0f, 1.0f));

        // Jolt-native semantics: the actor frame stays put while the collision
        // geometry (and with it the mass frame) shifts by -offset. Jolt cannot
        // express PhysX's "geometry fixed, mass frame moved" model.
        EXPECT_NEAR(body->GetCenterOfMassWorld().GetZ(), 9.0f, 0.05f);
        EXPECT_NEAR(body->GetPosition().GetZ(), 10.0f, 0.05f);

        // The 1m box now spans z in [8.5, 9.5], so a ray down from above hits z=9.5.
        AzPhysics::RayCastRequest request;
        request.m_start = AZ::Vector3(0.0f, 0.0f, 20.0f);
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 50.0f;
        AzPhysics::SceneQueryHits hits = m_scene->QueryScene(&request);
        ASSERT_EQ(hits.m_hits.size(), 1u);
        EXPECT_NEAR(hits.m_hits[0].m_position.GetZ(), 9.5f, 0.05f);
    }

    TEST_F(JoltRigidBodyTests, AttachingAShapeLeavesConvexHullGeometryWhereItWas)
    {
        // Rebuilding a body's compound has to undo Jolt's sub-shape storage exactly:
        // positions are kept relative to the compound's centre of mass AND offset by the
        // child's own. Dropping the second term moves every child with a non-zero centre
        // of mass - which is every convex hull, since hulls are centroid-relative - so a
        // hull-based body silently detaches from its render mesh the first time anything
        // attaches or detaches a shape. Boxes cannot catch this: their centre of mass is
        // already the origin.
        AZStd::vector<AZ::Vector3> points;
        for (const float dx : { -0.5f, 0.5f })
        {
            for (const float dy : { -0.5f, 0.5f })
            {
                for (const float dz : { -0.5f, 0.5f })
                {
                    points.push_back(AZ::Vector3(4.0f + dx, dy, dz));
                }
            }
        }

        auto cookedConfig = AZStd::make_shared<Physics::CookedMeshShapeConfiguration>();
        const AZStd::vector<AZ::u8> blob = JoltMeshUtils::PackConvexMesh(points.data(), static_cast<AZ::u32>(points.size()));
        cookedConfig->SetCookedMeshData(
            blob.data(), blob.size(), Physics::CookedMeshShapeConfiguration::MeshType::Convex);

        AzPhysics::RigidBodyConfiguration config;
        config.m_position = AZ::Vector3::CreateZero();
        config.m_kinematic = true; // hold it still so only the geometry rebuild can move anything
        config.m_colliderAndShapeData =
            AzPhysics::ShapeColliderPair(AZStd::make_shared<Physics::ColliderConfiguration>(), cookedConfig);

        auto* body = azdynamic_cast<AzPhysics::RigidBody*>(
            m_scene->GetSimulatedBodyFromHandle(m_scene->AddSimulatedBody(&config)));
        ASSERT_NE(body, nullptr);

        // Ask the body itself where its geometry is, rather than reading bounds: a ray
        // straight down through the hull's own column either hits it or it has moved.
        auto rayDownAt = [](const AZ::Vector3& xy)
        {
            AzPhysics::RayCastRequest request;
            request.m_start = AZ::Vector3(xy.GetX(), xy.GetY(), 5.0f);
            request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
            request.m_distance = 20.0f;
            return request;
        };

        auto hullRay = rayDownAt(AZ::Vector3(4.0f, 0.0f, 0.0f));
        ASSERT_NE(body->RayCast(hullRay).m_bodyHandle, AzPhysics::InvalidSimulatedBodyHandle)
            << "the hull is not where it was authored before anything was attached";

        Physics::ColliderConfiguration attachedCollider;
        attachedCollider.m_position = AZ::Vector3(0.0f, 6.0f, 0.0f);
        Physics::BoxShapeConfiguration attachedShapeConfig;
        AZStd::shared_ptr<Physics::Shape> attachedShape =
            JoltShapeUtils::CreateShape(attachedCollider, attachedShapeConfig);
        ASSERT_NE(attachedShape, nullptr);
        body->AddShape(attachedShape);

        // The hull has not moved, and the attached box landed where it was asked to.
        EXPECT_NE(body->RayCast(hullRay).m_bodyHandle, AzPhysics::InvalidSimulatedBodyHandle)
            << "attaching a shape displaced the hull by its centroid";
        auto attachedRay = rayDownAt(AZ::Vector3(0.0f, 6.0f, 0.0f));
        EXPECT_NE(body->RayCast(attachedRay).m_bodyHandle, AzPhysics::InvalidSimulatedBodyHandle);

        // Detaching removes the attachment and leaves the hull untouched - a lossless
        // round trip through the mutable-compound rebuild.
        body->RemoveShape(attachedShape);
        EXPECT_NE(body->RayCast(hullRay).m_bodyHandle, AzPhysics::InvalidSimulatedBodyHandle)
            << "detaching a shape displaced the hull by its centroid";
        EXPECT_EQ(body->RayCast(attachedRay).m_bodyHandle, AzPhysics::InvalidSimulatedBodyHandle)
            << "RemoveShape removed the wrong sub-shape";

        if (auto* cachedMesh = static_cast<JPH::Shape*>(cookedConfig->GetCachedNativeMesh()))
        {
            cachedMesh->Release();
            cookedConfig->SetCachedNativeMesh(nullptr);
        }
    }

    TEST_F(JoltRigidBodyTests, KinematicTargetMovesKinematicBody)
    {
        auto* body = CreateDynamicBox(AZ::Vector3::CreateZero());
        ASSERT_NE(body, nullptr);
        body->SetKinematic(true);

        body->SetKinematicTarget(AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 1.0f)));
        SimulateSeconds(1.0f / 60.0f);

        EXPECT_NEAR(body->GetPosition().GetZ(), 1.0f, 0.1f);
    }

    TEST_F(JoltRigidBodyTests, DisabledSimulationFreezesBodyUntilReenabled)
    {
        auto* body = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 10.0f));
        ASSERT_NE(body, nullptr);

        body->SetSimulationEnabled(false);
        SimulateSeconds(0.5f);
        EXPECT_NEAR(body->GetPosition().GetZ(), 10.0f, 0.01f);

        body->SetSimulationEnabled(true);
        SimulateSeconds(0.5f);
        EXPECT_LT(body->GetPosition().GetZ(), 9.0f);
    }

    TEST_F(JoltRigidBodyTests, PerBodyRayCastHitsAndMisses)
    {
        auto* body = CreateDynamicBox(AZ::Vector3::CreateZero());
        ASSERT_NE(body, nullptr);

        AzPhysics::RayCastRequest request;
        request.m_start = AZ::Vector3(0.0f, 0.0f, 5.0f);
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 20.0f;

        AzPhysics::SceneQueryHit hit = body->RayCast(request);
        EXPECT_NEAR(hit.m_position.GetZ(), 0.5f, 0.01f);
        EXPECT_EQ(hit.m_bodyHandle, body->m_bodyHandle);

        request.m_start = AZ::Vector3(5.0f, 0.0f, 5.0f);
        AzPhysics::SceneQueryHit miss = body->RayCast(request);
        EXPECT_EQ(miss.m_bodyHandle, AzPhysics::InvalidSimulatedBodyHandle);
    }

    TEST_F(JoltRigidBodyTests, AttachedShapeExtendsBodyGeometryAndDetachRemovesIt)
    {
        auto* body = CreateDynamicBox(AZ::Vector3::CreateZero());
        ASSERT_NE(body, nullptr);

        // A ray 2m out along x misses the unit box the body was created with.
        AzPhysics::RayCastRequest request;
        request.m_start = AZ::Vector3(2.0f, 0.0f, 5.0f);
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 20.0f;
        EXPECT_EQ(body->RayCast(request).m_bodyHandle, AzPhysics::InvalidSimulatedBodyHandle);

        // Attach a second box offset 2m along x; the ray now hits the attached geometry.
        Physics::ColliderConfiguration attachedCollider;
        attachedCollider.m_position = AZ::Vector3(2.0f, 0.0f, 0.0f);
        Physics::BoxShapeConfiguration attachedShapeConfig;
        AZStd::shared_ptr<Physics::Shape> attachedShape =
            JoltShapeUtils::CreateShape(attachedCollider, attachedShapeConfig);
        ASSERT_NE(attachedShape, nullptr);

        body->AddShape(attachedShape);
        AzPhysics::SceneQueryHit hit = body->RayCast(request);
        EXPECT_EQ(hit.m_bodyHandle, body->m_bodyHandle);
        EXPECT_NEAR(hit.m_position.GetZ(), 0.5f, 0.01f);

        // The original geometry is still there.
        request.m_start = AZ::Vector3(0.0f, 0.0f, 5.0f);
        EXPECT_EQ(body->RayCast(request).m_bodyHandle, body->m_bodyHandle);

        // Detaching removes only the attached geometry.
        body->RemoveShape(attachedShape);
        request.m_start = AZ::Vector3(2.0f, 0.0f, 5.0f);
        EXPECT_EQ(body->RayCast(request).m_bodyHandle, AzPhysics::InvalidSimulatedBodyHandle);
        request.m_start = AZ::Vector3(0.0f, 0.0f, 5.0f);
        EXPECT_EQ(body->RayCast(request).m_bodyHandle, body->m_bodyHandle);
    }

    TEST_F(JoltRigidBodyTests, AttachedShapeCollidesAndPreservesConfiguredMass)
    {
        // Static slab with its top surface at z=0.
        auto slabCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        slabCollider->m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        auto slabShape = AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(20.0f, 20.0f, 1.0f));
        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(slabCollider, slabShape);
        m_scene->AddSimulatedBody(&slabConfig);

        // Dynamic body with an explicit mass, dropped from a height.
        auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        AzPhysics::RigidBodyConfiguration config;
        config.m_position = AZ::Vector3(0.0f, 0.0f, 3.0f);
        config.m_computeMass = false; // an authored mass is the point of this test
        config.m_mass = 7.0f;
        config.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);
        auto handle = m_scene->AddSimulatedBody(&config);
        auto* body = static_cast<AzPhysics::RigidBody*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(body, nullptr);
        ASSERT_NEAR(body->GetMass(), 7.0f, 0.01f);

        // Attach a box below the body, extending its geometry downwards by one unit.
        Physics::ColliderConfiguration attachedCollider;
        attachedCollider.m_position = AZ::Vector3(0.0f, 0.0f, -1.0f);
        Physics::BoxShapeConfiguration attachedShapeConfig;
        AZStd::shared_ptr<Physics::Shape> attachedShape =
            JoltShapeUtils::CreateShape(attachedCollider, attachedShapeConfig);
        ASSERT_NE(attachedShape, nullptr);
        body->AddShape(attachedShape);

        // The configured mass survives the shape swap (which recomputes inertia).
        EXPECT_NEAR(body->GetMass(), 7.0f, 0.01f);

        SimulateSeconds(2.0f);

        // The attached box is what lands on the slab, so the body's origin rests one
        // unit higher than a lone box would (which would settle at z=0.5).
        EXPECT_NEAR(body->GetPosition().GetZ(), 1.5f, 0.1f);
    }

    TEST_F(JoltRigidBodyTests, ZeroSleepThresholdKeepsTheBodyAwake)
    {
        // Slab with its top at z=0 for the body to settle on.
        auto slabCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        slabCollider->m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        auto slabShape = AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(20.0f, 20.0f, 1.0f));
        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(slabCollider, slabShape);
        m_scene->AddSimulatedBody(&slabConfig);

        auto* sleeper = CreateDynamicBox(AZ::Vector3(-3.0f, 0.0f, 0.5f));
        auto* insomniac = CreateDynamicBox(AZ::Vector3(3.0f, 0.0f, 0.5f));
        ASSERT_NE(sleeper, nullptr);
        ASSERT_NE(insomniac, nullptr);

        // Zero means "never sleep" (Jolt has no per-body threshold magnitude, so this maps
        // to the body's allow-sleeping flag).
        insomniac->SetSleepThreshold(0.0f);
        EXPECT_FLOAT_EQ(insomniac->GetSleepThreshold(), 0.0f);

        // Long enough for a resting body to fall asleep (Jolt's default is 0.5s).
        SimulateSeconds(3.0f);

        EXPECT_FALSE(sleeper->IsAwake());
        EXPECT_TRUE(insomniac->IsAwake());
    }

    TEST_F(JoltRigidBodyTests, ZeroSleepThresholdWakesASleepingBody)
    {
        auto slabCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        slabCollider->m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        auto slabShape = AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(20.0f, 20.0f, 1.0f));
        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(slabCollider, slabShape);
        m_scene->AddSimulatedBody(&slabConfig);

        auto* body = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 0.5f));
        ASSERT_NE(body, nullptr);

        SimulateSeconds(3.0f);
        ASSERT_FALSE(body->IsAwake());

        // Disallowing sleep on an already sleeping body wakes it back up.
        body->SetSleepThreshold(0.0f);
        EXPECT_TRUE(body->IsAwake());
    }

    TEST_F(JoltRigidBodyTests, RemoveShapeIgnoresShapesNotAttached)
    {
        auto* body = CreateDynamicBox(AZ::Vector3::CreateZero());
        ASSERT_NE(body, nullptr);

        Physics::ColliderConfiguration colliderConfig;
        Physics::BoxShapeConfiguration shapeConfig;
        AZStd::shared_ptr<Physics::Shape> strangerShape = JoltShapeUtils::CreateShape(colliderConfig, shapeConfig);
        ASSERT_NE(strangerShape, nullptr);

        // Removing a shape that was never attached warns and leaves the body intact.
        body->RemoveShape(strangerShape);


        AzPhysics::RayCastRequest request;
        request.m_start = AZ::Vector3(0.0f, 0.0f, 5.0f);
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 20.0f;
        EXPECT_EQ(body->RayCast(request).m_bodyHandle, body->m_bodyHandle);
    }

} // namespace JoltPhysics
