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

    TEST_F(JoltRigidBodyTests, ANonSimulatedColliderDoesNotHoldABodyUp)
    {
        // Simulated unticked is how PhysX authors make query-only geometry: hitboxes,
        // camera probes. Jolt has no per-sub-shape simulation flag, so the gem rejects
        // the contact instead - and the observable difference is that a body lands on
        // the floor rather than resting on a hitbox.
        auto slabCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        slabCollider->m_isSimulated = false;
        auto slabShape = AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(20.0f, 20.0f, 1.0f));
        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_position = AZ::Vector3(0.0f, 0.0f, 2.0f);
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(slabCollider, slabShape);
        m_scene->AddSimulatedBody(&slabConfig);

        // A real floor further down, so the falling body has something to land on.
        auto floorCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto floorShape = AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(20.0f, 20.0f, 1.0f));
        AzPhysics::StaticRigidBodyConfiguration floorConfig;
        floorConfig.m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        floorConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(floorCollider, floorShape);
        m_scene->AddSimulatedBody(&floorConfig);

        auto* body = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 6.0f));
        ASSERT_NE(body, nullptr);

        SimulateSeconds(3.0f);

        // Through the query-only slab at z=2.5 and down onto the floor at z=0.
        EXPECT_LT(body->GetPosition().GetZ(), 1.0f)
            << "the body came to rest on a collider that is not supposed to simulate";
        EXPECT_NEAR(body->GetPosition().GetZ(), 0.5f, 0.1f);
    }

    TEST_F(JoltRigidBodyTests, CenterOfMassOffsetShiftsMassFrame)
    {
        auto* body = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 10.0f));
        ASSERT_NE(body, nullptr);

        body->SetCenterOfMassOffset(AZ::Vector3(0.0f, 0.0f, 1.0f));

        // The mass frame moves by +offset; the geometry does not move at all. This test
        // used to assert the opposite (geometry displaced by -offset), because the gem
        // wrapped the shape in a RotatedTranslatedShape rather than Jolt's
        // OffsetCenterOfMassShape - the test agreed with the bug.
        EXPECT_NEAR(body->GetCenterOfMassWorld().GetZ(), 11.0f, 0.05f);
        EXPECT_NEAR(body->GetPosition().GetZ(), 10.0f, 0.05f);

        // The 1m box still spans z in [9.5, 10.5], so a ray down from above hits z=10.5.
        AzPhysics::RayCastRequest request;
        request.m_start = AZ::Vector3(0.0f, 0.0f, 20.0f);
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 50.0f;
        AzPhysics::SceneQueryHits hits = m_scene->QueryScene(&request);
        ASSERT_EQ(hits.m_hits.size(), 1u);
        EXPECT_NEAR(hits.m_hits[0].m_position.GetZ(), 10.5f, 0.05f);
    }

    //! Helper for the configuration-driven tests below: a 1m box body built from a
    //! configuration the caller has already tuned, rather than from the defaults.
    static AzPhysics::RigidBody* AddConfiguredBox(AzPhysics::Scene* scene, AzPhysics::RigidBodyConfiguration& config)
    {
        auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        config.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);
        auto handle = scene->AddSimulatedBody(&config);
        return static_cast<AzPhysics::RigidBody*>(scene->GetSimulatedBodyFromHandle(handle));
    }

    TEST_F(JoltRigidBodyTests, AComOffsetIsIgnoredWhileComputeComIsSet)
    {
        AzPhysics::RigidBodyConfiguration config;
        config.m_position = AZ::Vector3(0.0f, 0.0f, 10.0f);
        ASSERT_TRUE(config.m_computeCenterOfMass) << "the engine default this test depends on has changed";
        config.m_centerOfMassOffset = AZ::Vector3(0.0f, 0.0f, 1.0f);

        auto* body = AddConfiguredBox(m_scene, config);
        ASSERT_NE(body, nullptr);

        // The shapes decide the centre of mass, so the stale offset must not move it.
        EXPECT_NEAR(body->GetCenterOfMassWorld().GetZ(), 10.0f, 0.05f);
    }

    TEST_F(JoltRigidBodyTests, AComOffsetAppliesOnceComputeComIsUnticked)
    {
        AzPhysics::RigidBodyConfiguration config;
        config.m_position = AZ::Vector3(0.0f, 0.0f, 10.0f);
        config.m_computeCenterOfMass = false;
        config.m_centerOfMassOffset = AZ::Vector3(0.0f, 0.0f, 1.0f);

        auto* body = AddConfiguredBox(m_scene, config);
        ASSERT_NE(body, nullptr);

        EXPECT_NEAR(body->GetCenterOfMassWorld().GetZ(), 11.0f, 0.05f);
        EXPECT_NEAR(body->GetPosition().GetZ(), 10.0f, 0.05f) << "the entity frame should not move";
    }

    TEST_F(JoltRigidBodyTests, TheComOffsetIsAbsoluteEvenWhenTheGeometryIsNotCentred)
    {
        // Every other test here uses a collider centred on the entity, where the shape's
        // own centre of mass is zero and "absolute" and "delta" are the same number. A
        // mesh collider is a compound sitting whereever the geometry is, and the offset
        // used to be added to that - putting a car's mass frame at geometry + offset,
        // above its suspension mounts, and making it unstable.
        auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
        colliderConfig->m_position = AZ::Vector3(2.0f, 0.0f, 1.0f); // geometry well off the origin
        auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();

        AzPhysics::RigidBodyConfiguration config;
        config.m_position = AZ::Vector3(0.0f, 0.0f, 10.0f);
        config.m_computeCenterOfMass = false;
        config.m_centerOfMassOffset = AZ::Vector3(0.0f, 0.0f, 0.25f);
        config.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);

        auto handle = m_scene->AddSimulatedBody(&config);
        auto* body = static_cast<AzPhysics::RigidBody*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(body, nullptr);

        // The centre of mass is at the entity origin plus the authored offset - it does
        // not pick up the collider's own displacement of (2, 0, 1).
        const AZ::Vector3 com = body->GetCenterOfMassWorld();
        EXPECT_NEAR(com.GetX(), 0.0f, 0.02f);
        EXPECT_NEAR(com.GetY(), 0.0f, 0.02f);
        EXPECT_NEAR(com.GetZ(), 10.25f, 0.02f);
    }

    TEST_F(JoltRigidBodyTests, AZeroComOffsetStillPinsTheMassFrameToTheOrigin)
    {
        // Zero is a meaningful value once the field is absolute: with Compute COM off it
        // means "put the mass frame on the entity origin", not "leave it on the geometry".
        auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
        colliderConfig->m_position = AZ::Vector3(0.0f, 0.0f, 3.0f);
        auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();

        AzPhysics::RigidBodyConfiguration config;
        config.m_position = AZ::Vector3::CreateZero();
        config.m_computeCenterOfMass = false;
        config.m_centerOfMassOffset = AZ::Vector3::CreateZero();
        config.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);

        auto handle = m_scene->AddSimulatedBody(&config);
        auto* body = static_cast<AzPhysics::RigidBody*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(body, nullptr);

        EXPECT_NEAR(body->GetCenterOfMassWorld().GetZ(), 0.0f, 0.02f);
    }

    TEST_F(JoltRigidBodyTests, ComputeComLeavesTheCentreOfMassOnTheGeometry)
    {
        // COMPUTE_COM means "let the shapes decide", so the centre of mass must land on
        // the collider, not on the entity origin. Routing this through
        // SetCenterOfMassOffset used to pin it to the origin, because that setter reads an
        // offset as "stop computing" and the shape was built before the flag was restored.
        auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
        colliderConfig->m_position = AZ::Vector3(2.0f, 0.0f, 1.0f);
        auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();

        AzPhysics::RigidBodyConfiguration config;
        config.m_position = AZ::Vector3::CreateZero();
        config.m_computeCenterOfMass = false;
        config.m_centerOfMassOffset = AZ::Vector3(0.0f, 0.0f, 0.25f);
        config.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);

        auto handle = m_scene->AddSimulatedBody(&config);
        auto* body = static_cast<AzPhysics::RigidBody*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(body, nullptr);
        ASSERT_NEAR(body->GetCenterOfMassWorld().GetZ(), 0.25f, 0.02f) << "the authored offset should apply first";

        body->UpdateMassProperties(
            AzPhysics::MassComputeFlags::COMPUTE_COM | AzPhysics::MassComputeFlags::COMPUTE_MASS |
            AzPhysics::MassComputeFlags::COMPUTE_INERTIA);

        // The collider sits at (2, 0, 1), so that is where the geometry puts the mass.
        const AZ::Vector3 com = body->GetCenterOfMassWorld();
        EXPECT_NEAR(com.GetX(), 2.0f, 0.02f);
        EXPECT_NEAR(com.GetZ(), 1.0f, 0.02f);
    }

    TEST_F(JoltRigidBodyTests, ALockedLinearAxisDoesNotTranslate)
    {
        AzPhysics::RigidBodyConfiguration config;
        config.m_position = AZ::Vector3::CreateZero();
        config.m_gravityEnabled = false;
        config.m_lockLinearX = true;

        auto* body = AddConfiguredBox(m_scene, config);
        ASSERT_NE(body, nullptr);

        // The default 1m box weighs ~1000kg, so the impulse has to be scaled to match or
        // the "free axis moved" half of the assertion measures nothing.
        body->ApplyLinearImpulse(AZ::Vector3(10000.0f, 10000.0f, 0.0f));
        SimulateSeconds(0.5f);

        EXPECT_NEAR(body->GetPosition().GetX(), 0.0f, 0.01f) << "the locked axis moved";
        EXPECT_GT(body->GetPosition().GetY(), 1.0f) << "the free axis should still move";
    }

    TEST_F(JoltRigidBodyTests, ALockedAngularAxisDoesNotRotate)
    {
        AzPhysics::RigidBodyConfiguration config;
        config.m_position = AZ::Vector3::CreateZero();
        config.m_gravityEnabled = false;
        config.m_lockAngularZ = true;

        auto* body = AddConfiguredBox(m_scene, config);
        ASSERT_NE(body, nullptr);

        body->ApplyAngularImpulse(AZ::Vector3(0.0f, 0.0f, 5.0f));
        SimulateSeconds(0.5f);

        EXPECT_NEAR(body->GetAngularVelocity().GetZ(), 0.0f, 0.01f) << "the locked axis span up";
    }

    TEST_F(JoltRigidBodyTests, EveryAxisLockedLeavesTheBodySimulatableRatherThanCrashing)
    {
        AzPhysics::RigidBodyConfiguration config;
        config.m_position = AZ::Vector3::CreateZero();
        config.m_lockLinearX = config.m_lockLinearY = config.m_lockLinearZ = true;
        config.m_lockAngularX = config.m_lockAngularY = config.m_lockAngularZ = true;

        // Jolt documents EAllowedDOFs::None as invalid and crashing, so the gem warns and
        // ignores the locks instead of handing that to Jolt.
        AZ_TEST_START_TRACE_SUPPRESSION;
        auto* body = AddConfiguredBox(m_scene, config);
        AZ_TEST_STOP_TRACE_SUPPRESSION_NO_COUNT;

        ASSERT_NE(body, nullptr);
        SimulateSeconds(0.25f);
        SUCCEED();
    }

    TEST_F(JoltRigidBodyTests, AnAuthoredInertiaTensorResistsRotationMoreThanTheComputedOne)
    {
        AzPhysics::RigidBodyConfiguration computedConfig;
        computedConfig.m_position = AZ::Vector3(0.0f, 0.0f, 0.0f);
        computedConfig.m_gravityEnabled = false;
        auto* computedBody = AddConfiguredBox(m_scene, computedConfig);
        ASSERT_NE(computedBody, nullptr);

        AzPhysics::RigidBodyConfiguration authoredConfig;
        authoredConfig.m_position = AZ::Vector3(10.0f, 0.0f, 0.0f);
        authoredConfig.m_gravityEnabled = false;
        authoredConfig.m_computeInertiaTensor = false;
        // Has to be well above the ~167 kg*m^2 Jolt computes for a 1000kg 1m box, or the
        // authored body spins up faster and the assertion below measures the wrong sign.
        constexpr float AuthoredInertia = 100000.0f;
        authoredConfig.m_inertiaTensor =
            AZ::Matrix3x3::CreateDiagonal(AZ::Vector3(AuthoredInertia, AuthoredInertia, AuthoredInertia));
        auto* authoredBody = AddConfiguredBox(m_scene, authoredConfig);
        ASSERT_NE(authoredBody, nullptr);

        const AZ::Vector3 impulse(0.0f, 0.0f, 1.0f);
        computedBody->ApplyAngularImpulse(impulse);
        authoredBody->ApplyAngularImpulse(impulse);

        // Same impulse, far larger tensor: the authored body must spin up much slower, and
        // its rate must be the one the tensor implies rather than the geometry's.
        EXPECT_LT(
            AZStd::abs(authoredBody->GetAngularVelocity().GetZ()),
            AZStd::abs(computedBody->GetAngularVelocity().GetZ()) * 0.1f)
            << "the authored inertia tensor was not applied";
        EXPECT_NEAR(authoredBody->GetAngularVelocity().GetZ(), 1.0f / AuthoredInertia, 1e-6f);
    }

    TEST_F(JoltRigidBodyTests, IncludeAllShapesCountsQueryOnlyCollidersTowardsTheMass)
    {
        // Two colliders on one body: a simulated 1m box and a query-only 1m box. With the
        // flag off only the simulated one weighs anything, which is the engine default.
        auto makeBody = [this](bool includeAll, const AZ::Vector3& position)
        {
            auto simulatedCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto simulatedShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();

            auto queryOnlyCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
            queryOnlyCollider->m_isSimulated = false;
            auto queryOnlyShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();

            AzPhysics::RigidBodyConfiguration config;
            config.m_position = position;
            config.m_includeAllShapesInMassCalculation = includeAll;
            config.m_colliderAndShapeData = AzPhysics::ShapeColliderPairList{
                AzPhysics::ShapeColliderPair(simulatedCollider, simulatedShape),
                AzPhysics::ShapeColliderPair(queryOnlyCollider, queryOnlyShape)
            };
            auto handle = m_scene->AddSimulatedBody(&config);
            return static_cast<AzPhysics::RigidBody*>(m_scene->GetSimulatedBodyFromHandle(handle));
        };

        auto* excluded = makeBody(false, AZ::Vector3::CreateZero());
        auto* included = makeBody(true, AZ::Vector3(10.0f, 0.0f, 0.0f));
        ASSERT_NE(excluded, nullptr);
        ASSERT_NE(included, nullptr);

        EXPECT_NEAR(included->GetMass(), excluded->GetMass() * 2.0f, excluded->GetMass() * 0.05f)
            << "the query-only collider did not contribute its share of the mass";
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

    TEST_F(JoltRigidBodyTests, StartAsleepStartsAsleep)
    {
        // Found by grepping every DataElement in the gem for a read outside reflection:
        // RigidBodyConfiguration::m_startAsleep had none. The engine declares it, the
        // inspector shows it, and the body was created with EActivation::Activate
        // regardless - so a crate authored to sit still until touched fell on its first
        // step like every other. Same family as CCD, which was found the same week by a
        // project rather than by a grep.
        // Well apart: the first version of this put both at one point, and the awake box
        // fell onto the sleeping one and woke it, which read as the flag not working.
        auto make = [this](bool startAsleep, float x)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            AzPhysics::RigidBodyConfiguration config;
            config.m_position = AZ::Vector3(x, 0.0f, 10.0f);
            config.m_startAsleep = startAsleep;
            config.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);
            auto handle = m_scene->AddSimulatedBody(&config);
            return static_cast<AzPhysics::RigidBody*>(m_scene->GetSimulatedBodyFromHandle(handle));
        };

        auto* asleep = make(true, -5.0f);
        auto* awake = make(false, 5.0f);
        ASSERT_NE(asleep, nullptr);
        ASSERT_NE(awake, nullptr);
        EXPECT_FALSE(asleep->IsAwake()) << "a body authored to start asleep was created awake";
        EXPECT_TRUE(awake->IsAwake());

        SimulateSeconds(0.5f);

        // Nothing touched it, so a sleeping body has not moved; the awake one has fallen.
        EXPECT_NEAR(asleep->GetPosition().GetZ(), 10.0f, 1e-3f)
            << "the sleeping body fell; it was activated at creation whatever the flag said";
        EXPECT_LT(awake->GetPosition().GetZ(), 9.5f);
    }

    //! A small fast body fired at a thin wall - the case continuous collision exists for.
    class JoltContinuousCollisionTests : public JoltRigidBodyTests
    {
    protected:
        //! Fires a 0.1 m bullet at 200 m/s along +x at a 0.05 m thick wall two metres away,
        //! and reports whether it was stopped. At that speed a step advances it 3.3 m, so a
        //! discrete body is past the wall before anything is tested.
        bool BulletIsStoppedByThinWall(bool ccdEnabled)
        {
            auto wallCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto wallShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            wallShape->m_dimensions = AZ::Vector3(0.05f, 4.0f, 4.0f);
            AzPhysics::StaticRigidBodyConfiguration wallConfig;
            wallConfig.m_position = AZ::Vector3(2.0f, 0.0f, 0.0f);
            wallConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(wallCollider, wallShape);
            m_scene->AddSimulatedBody(&wallConfig);

            auto bulletCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto bulletShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            bulletShape->m_dimensions = AZ::Vector3(0.1f, 0.1f, 0.1f);
            AzPhysics::RigidBodyConfiguration bulletConfig;
            bulletConfig.m_position = AZ::Vector3::CreateZero();
            bulletConfig.m_gravityEnabled = false;
            bulletConfig.m_ccdEnabled = ccdEnabled;
            bulletConfig.m_initialLinearVelocity = AZ::Vector3(200.0f, 0.0f, 0.0f);
            bulletConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(bulletCollider, bulletShape);
            auto handle = m_scene->AddSimulatedBody(&bulletConfig);
            auto* bullet = static_cast<AzPhysics::RigidBody*>(m_scene->GetSimulatedBodyFromHandle(handle));

            SimulateSeconds(0.5f);
            return bullet->GetPosition().GetX() < 2.0f;
        }
    };

    TEST_F(JoltContinuousCollisionTests, ACcdBodyDoesNotTunnelThroughAThinWall)
    {
        // The CCD Enabled flag on the rigid body configuration was read by nothing.
        // SetCCDEnabled existed, was an override, and was never called from anywhere - so
        // every body was created Discrete whatever the checkbox said, and the editor showed
        // a setting that did nothing at all. Reported from a project as rounds tunnelling
        // through crates at 45 m/s, worked around there by making the rounds wider and
        // slower than the game wanted them.
        EXPECT_TRUE(BulletIsStoppedByThinWall(/*ccdEnabled*/ true))
            << "a body with CCD enabled passed straight through a wall thinner than one step of its travel";
    }

    TEST_F(JoltContinuousCollisionTests, ADiscreteBodyStillTunnels)
    {
        // The other half, so the test above is known to be measuring CCD rather than a
        // wall that would have stopped anything. Discrete bodies tunnel; that is what the
        // flag is for, and leaving it off must still behave that way.
        EXPECT_FALSE(BulletIsStoppedByThinWall(/*ccdEnabled*/ false))
            << "the wall stopped a discrete body, so this pair proves nothing about CCD";
    }

} // namespace JoltPhysics
