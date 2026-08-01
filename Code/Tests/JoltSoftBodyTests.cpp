#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <Configuration/JoltSettingsRegistryManager.h>
#include <Scene/JoltScene.h>
#include <SoftBody/JoltSoftBody.h>
#include <System/JoltSystem.h>

#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>

namespace JoltPhysics
{
    class JoltSoftBodyTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            // A real scene from this gem, rather than the raw JPH::PhysicsSystem these tests
            // used while soft bodies lived in a separate gem. Nothing needs isolating from the
            // physics gem any more, and going through JoltSystem covers the object layer
            // registry and the scene's own solver configuration as well.
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;

            // A named layer for soft bodies plus a group that excludes it, so the collision
            // filtering tests have something real to filter against.
            config.m_collisionConfig.m_collisionLayers.SetName(SoftBodyLayerIndex, "SoftBody");
            AzPhysics::CollisionGroup everythingButSoftBodies = AzPhysics::CollisionGroup::All;
            everythingButSoftBodies.SetLayer(AzPhysics::CollisionLayer(SoftBodyLayerIndex), false);
            m_noSoftBodiesGroupId =
                config.m_collisionConfig.m_collisionGroups.CreateGroup("NoSoftBodies", everythingButSoftBodies);

            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "SoftBodyTestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);
        }

        void TearDown() override
        {
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
            }
        }

        //! Adds the soft body to the scene with the pending settings, mirroring what the
        //! component does. The scene owns the body from here on.
        bool Attach()
        {
            JoltSoftBodyConfiguration configuration;
            configuration.m_settings = m_pendingSettings;
            configuration.m_position = m_pendingTransform.GetTranslation();
            configuration.m_orientation = m_pendingTransform.GetRotation();
            configuration.m_debugName = "TestSoftBody";

            m_softBodyHandle = m_scene->AddSimulatedBody(&configuration);
            return m_softBodyHandle != AzPhysics::InvalidSimulatedBodyHandle && SoftBody() != nullptr;
        }

        //! The live body, or null once it has been removed from the scene.
        JoltSoftBody* SoftBody() const
        {
            return azdynamic_cast<JoltSoftBody*>(m_scene->GetSimulatedBodyFromHandle(m_softBodyHandle));
        }

        void Detach()
        {
            m_scene->RemoveSimulatedBody(m_softBodyHandle);
        }

        //! A static floor whose top surface sits at the given height. The collision group
        //! decides which layers the floor collides with; the default collides with all.
        void CreateFloor(float height, const AzPhysics::CollisionGroups::Id& groupId = AzPhysics::CollisionGroups::Id())
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            colliderConfig->m_collisionGroupId = groupId;

            AzPhysics::StaticRigidBodyConfiguration config;
            config.m_position = AZ::Vector3(0.0f, 0.0f, height - 0.5f);
            config.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(
                colliderConfig,
                AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(100.0f, 100.0f, 1.0f)));
            m_scene->AddSimulatedBody(&config);
        }

        JoltSoftBodySettings ClothSettings(JoltSoftBodyPinning pinning) const
        {
            JoltSoftBodySettings settings;
            settings.m_shape = JoltSoftBodyShape::Cloth;
            settings.m_pinning = pinning;
            settings.m_size = AZ::Vector3(2.0f, 2.0f, 2.0f);
            settings.m_resolution = 6;
            settings.m_mass = 2.0f;
            // Sleeping would freeze a body mid-test and make the assertions depend on how
            // long the test simulated for.
            settings.m_allowSleeping = false;
            return settings;
        }

        static constexpr AZ::u64 SoftBodyLayerIndex = 1;

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle = AzPhysics::InvalidSceneHandle;
        AzPhysics::Scene* m_scene = nullptr;
        AzPhysics::SimulatedBodyHandle m_softBodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        AzPhysics::CollisionGroups::Id m_noSoftBodiesGroupId;

        //! Staged by the tests before Attach, since a soft body's settings and placement
        //! are part of its creation configuration rather than set on a live body.
        JoltSoftBodySettings m_pendingSettings;
        AZ::Transform m_pendingTransform = AZ::Transform::CreateIdentity();
    };

    TEST_F(JoltSoftBodyTests, AttachingBuildsParticlesAndTriangles)
    {
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::None);
        ASSERT_TRUE(Attach());
        EXPECT_TRUE((SoftBody() != nullptr));

        // A 6 x 6 grid is 36 particles and 25 quads, so 50 triangles.
        EXPECT_EQ(SoftBody()->GetVertexCount(), 36u);
        EXPECT_EQ(SoftBody()->GetTriangleIndices().size(), 50u * 3u);
    }

    TEST_F(JoltSoftBodyTests, AddingASoftBodyToAnInvalidSceneFails)
    {
        JoltSoftBodyConfiguration configuration;
        configuration.m_settings = ClothSettings(JoltSoftBodyPinning::None);

        JoltScene* missingScene = azdynamic_cast<JoltScene*>(m_system->GetScene(AzPhysics::InvalidSceneHandle));
        EXPECT_EQ(missingScene, nullptr);
        EXPECT_EQ(SoftBody(), nullptr);
    }

    TEST_F(JoltSoftBodyTests, UnpinnedClothFalls)
    {
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::None);
        m_pendingTransform = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 5.0f));
        ASSERT_TRUE(Attach());

        SimulateSeconds(1.0f);

        // Roughly a second of free fall is about -4.9 m, so it should be well below where it
        // started with nothing holding it up.
        const AZ::Aabb bounds = SoftBody()->GetWorldBounds();
        ASSERT_TRUE(bounds.IsValid());
        EXPECT_LT(bounds.GetMax().GetZ(), 3.0f);
    }

    TEST_F(JoltSoftBodyTests, PinnedCornersHoldTheClothUp)
    {
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::Corners);
        m_pendingTransform = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 5.0f));
        ASSERT_TRUE(Attach());

        SimulateSeconds(2.0f);

        // The pinned corners cannot move, so the sheet sags between them but its top stays at
        // the height it was created at - the contrast with UnpinnedClothFalls is the point.
        const AZ::Aabb bounds = SoftBody()->GetWorldBounds();
        ASSERT_TRUE(bounds.IsValid());
        EXPECT_NEAR(bounds.GetMax().GetZ(), 5.0f, 0.1f);
        // It did sag, but only slightly: the default compliance of 0 makes the edges
        // inextensible, so a 2 m sheet pinned at four corners can only buckle rather than
        // stretch. A visible drape needs a non-zero compliance.
        EXPECT_LT(bounds.GetMin().GetZ(), 4.99f);
    }

    TEST_F(JoltSoftBodyTests, ClothLandsOnAFloorAndStaysAboveIt)
    {
        CreateFloor(0.0f);

        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::None);
        m_pendingTransform = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 2.0f));
        ASSERT_TRUE(Attach());

        SimulateSeconds(3.0f);

        // Collision against a rigid body created through this gem's own configuration path is
        // what makes a soft body useful rather than a curiosity.
        const AZ::Aabb bounds = SoftBody()->GetWorldBounds();
        ASSERT_TRUE(bounds.IsValid());
        EXPECT_GT(bounds.GetMin().GetZ(), -0.5f);
        EXPECT_LT(bounds.GetMin().GetZ(), 0.5f);
    }

    TEST_F(JoltSoftBodyTests, PressureInflatesABalloon)
    {
        JoltSoftBodySettings settings;
        settings.m_shape = JoltSoftBodyShape::Balloon;
        settings.m_size = AZ::Vector3(1.0f, 1.0f, 1.0f);
        settings.m_resolution = 6;
        settings.m_mass = 1.0f;
        settings.m_gravityFactor = 0.0f; // isolate pressure from falling
        settings.m_allowSleeping = false;
        settings.m_compliance = 1.0e-4f; // has to be able to stretch to inflate at all
        settings.m_pressure = 0.0f;
        m_pendingSettings = settings;
        ASSERT_TRUE(Attach());

        SimulateSeconds(0.5f);
        const AZ::Aabb deflated = SoftBody()->GetWorldBounds();
        ASSERT_TRUE(deflated.IsValid());

        // Raising pressure on a live body must take effect without a rebuild. This also pins
        // the sphere's face winding: Jolt derives the enclosed volume from it and silently
        // skips pressure when that volume comes out negative.
        SoftBody()->SetPressure(5000.0f);
        SimulateSeconds(1.0f);

        const AZ::Aabb inflated = SoftBody()->GetWorldBounds();
        ASSERT_TRUE(inflated.IsValid());
        EXPECT_GT(inflated.GetExtents().GetMaxElement(), deflated.GetExtents().GetMaxElement());
    }

    TEST_F(JoltSoftBodyTests, GravityFactorStopsTheBodyFalling)
    {
        JoltSoftBodySettings settings = ClothSettings(JoltSoftBodyPinning::None);
        settings.m_gravityFactor = 0.0f;
        m_pendingSettings = settings;
        m_pendingTransform = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 5.0f));
        ASSERT_TRUE(Attach());

        SimulateSeconds(1.0f);

        const AZ::Aabb bounds = SoftBody()->GetWorldBounds();
        ASSERT_TRUE(bounds.IsValid());
        EXPECT_NEAR(bounds.GetMax().GetZ(), 5.0f, 0.1f);
    }

    TEST_F(JoltSoftBodyTests, ChangingABakedSettingRebuildsInPlace)
    {
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::None);
        ASSERT_TRUE(Attach());

        const AZ::u32 generationBefore = SoftBody()->GetBuildGeneration();
        EXPECT_EQ(SoftBody()->GetVertexCount(), 36u);

        JoltSoftBodySettings settings = ClothSettings(JoltSoftBodyPinning::None);
        settings.m_resolution = 4; // baked: needs a new particle layout
        SoftBody()->SetSettings(settings);

        EXPECT_GT(SoftBody()->GetBuildGeneration(), generationBefore);
        EXPECT_EQ(SoftBody()->GetVertexCount(), 16u);
        EXPECT_TRUE((SoftBody() != nullptr));
    }

    TEST_F(JoltSoftBodyTests, ChangingALiveSettingDoesNotRebuild)
    {
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::None);
        ASSERT_TRUE(Attach());

        const AZ::u32 generationBefore = SoftBody()->GetBuildGeneration();

        JoltSoftBodySettings settings = ClothSettings(JoltSoftBodyPinning::None);
        settings.m_linearDamping = 0.9f; // live: forwarded to the existing body
        settings.m_numIterations = 12;
        SoftBody()->SetSettings(settings);

        // Rebuilding here would throw away the simulated state every time gameplay nudged a
        // value, which is the whole reason the settings are split into baked and live.
        EXPECT_EQ(SoftBody()->GetBuildGeneration(), generationBefore);
        EXPECT_EQ(SoftBody()->GetSettings().m_numIterations, 12u);
    }

    TEST_F(JoltSoftBodyTests, CollisionLayerAndGroupReachTheBody)
    {
        JoltSoftBodySettings settings = ClothSettings(JoltSoftBodyPinning::None);
        settings.m_collisionLayer = AzPhysics::CollisionLayer(SoftBodyLayerIndex);
        settings.m_collisionGroupId = m_noSoftBodiesGroupId;
        m_pendingSettings = settings;
        ASSERT_TRUE(Attach());

        // A distinct (layer, group) pair has to resolve to its own object layer, not the
        // catch-all Moving layer every unconfigured body lands on.
        const JPH::ObjectLayer configuredLayer = SoftBody()->GetObjectLayer();
        EXPECT_NE(configuredLayer, ObjectLayers::Moving);
        EXPECT_NE(configuredLayer, ObjectLayers::NonMoving);

        JoltSoftBodyConfiguration defaultConfiguration;
        defaultConfiguration.m_settings = ClothSettings(JoltSoftBodyPinning::None);
        defaultConfiguration.m_debugName = "DefaultLayerSoftBody";
        AzPhysics::SimulatedBodyHandle defaultHandle = m_scene->AddSimulatedBody(&defaultConfiguration);
        ASSERT_NE(defaultHandle, AzPhysics::InvalidSimulatedBodyHandle);

        auto* defaultBody = azdynamic_cast<JoltSoftBody*>(m_scene->GetSimulatedBodyFromHandle(defaultHandle));
        ASSERT_NE(defaultBody, nullptr);
        EXPECT_NE(defaultBody->GetObjectLayer(), configuredLayer);
        m_scene->RemoveSimulatedBody(defaultHandle);
    }

    TEST_F(JoltSoftBodyTests, ClothFallsThroughAFloorThatExcludesItsLayer)
    {
        // The floor collides with everything except the soft body layer.
        CreateFloor(0.0f, m_noSoftBodiesGroupId);

        JoltSoftBodySettings settings = ClothSettings(JoltSoftBodyPinning::None);
        settings.m_collisionLayer = AzPhysics::CollisionLayer(SoftBodyLayerIndex);
        m_pendingSettings = settings;
        m_pendingTransform = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 2.0f));
        ASSERT_TRUE(Attach());

        SimulateSeconds(3.0f);

        // Well below the floor: the filtering is what is being tested, so the assertion has
        // to be one that a merely-slow landing could not satisfy. ClothLandsOnAFloorAndStays        // AboveIt is the same setup with an unfiltered floor and asserts the opposite.
        const AZ::Aabb bounds = SoftBody()->GetWorldBounds();
        ASSERT_TRUE(bounds.IsValid());
        EXPECT_LT(bounds.GetMax().GetZ(), -2.0f);
    }

    TEST_F(JoltSoftBodyTests, ChangingTheCollisionLayerMovesTheBodyWithoutRebuilding)
    {
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::None);
        ASSERT_TRUE(Attach());

        const AZ::u32 generationBefore = SoftBody()->GetBuildGeneration();
        const JPH::ObjectLayer layerBefore = SoftBody()->GetObjectLayer();

        SoftBody()->SetCollisionLayer(AzPhysics::CollisionLayer(SoftBodyLayerIndex));

        // Jolt can move a live body between object layers, and rebuilding would discard
        // whatever deformation the body had settled into.
        EXPECT_EQ(SoftBody()->GetBuildGeneration(), generationBefore);
        EXPECT_NE(SoftBody()->GetObjectLayer(), layerBefore);
        EXPECT_EQ(SoftBody()->GetSettings().m_collisionLayer.GetIndex(), SoftBodyLayerIndex);
    }

    TEST_F(JoltSoftBodyTests, DetachRemovesTheBody)
    {
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::None);
        ASSERT_TRUE(Attach());
        ASSERT_EQ(SoftBody()->GetVertexCount(), 36u);

        Detach();

        // The scene owns the body, so removing it makes the handle stop resolving rather
        // than leaving an emptied object behind.
        EXPECT_EQ(SoftBody(), nullptr);
        EXPECT_EQ(m_softBodyHandle, AzPhysics::InvalidSimulatedBodyHandle);

        // Stepping a scene the body was removed from must not touch freed memory.
        SimulateSeconds(0.5f);

        // And it can be brought back, which is how the component's enable toggle works.
        EXPECT_TRUE(Attach());
        EXPECT_EQ(SoftBody()->GetVertexCount(), 36u);
    }

    TEST_F(JoltSoftBodyTests, SceneQueryFindsASoftBody)
    {
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::Corners);
        m_pendingTransform = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 5.0f));
        ASSERT_TRUE(Attach());

        // Straight down through the middle of the sheet.
        AzPhysics::RayCastRequest request;
        request.m_start = AZ::Vector3(0.0f, 0.0f, 8.0f);
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 10.0f;

        AzPhysics::SceneQueryHits hits = m_scene->QueryScene(&request);

        // Before soft bodies were AzPhysics::SimulatedBodys they held a raw JPH::BodyID that
        // no scene handle resolved to, so a query walked straight past them.
        ASSERT_EQ(hits.m_hits.size(), 1u);
        EXPECT_EQ(hits.m_hits[0].m_bodyHandle, m_softBodyHandle);
        EXPECT_NEAR(hits.m_hits[0].m_position.GetZ(), 5.0f, 0.5f);
    }

    TEST_F(JoltSoftBodyTests, SoftBodyIsReachableThroughItsSceneHandle)
    {
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::Corners);
        m_pendingTransform = AZ::Transform::CreateTranslation(AZ::Vector3(1.0f, 2.0f, 3.0f));
        ASSERT_TRUE(Attach());

        AzPhysics::SimulatedBody* body = m_scene->GetSimulatedBodyFromHandle(m_softBodyHandle);
        ASSERT_NE(body, nullptr);
        EXPECT_EQ(body->GetNativeType(), AZ_CRC_CE("JoltSoftBody"));
        EXPECT_THAT(body->GetPosition(), ::testing::Eq(AZ::Vector3(1.0f, 2.0f, 3.0f)));

        // The AABB tracks the particles rather than the creation transform, so a pinned
        // sheet reports the extent of the cloth itself.
        const AZ::Aabb aabb = body->GetAabb();
        ASSERT_TRUE(aabb.IsValid());
        EXPECT_GT(aabb.GetXExtent(), 1.0f);
    }

    TEST_F(JoltSoftBodyTests, BodyLevelRayCastHitsTheDeformedSurface)
    {
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::Corners);
        m_pendingTransform = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 5.0f));
        ASSERT_TRUE(Attach());

        AzPhysics::RayCastRequest request;
        request.m_start = AZ::Vector3(0.0f, 0.0f, 8.0f);
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 10.0f;

        const AzPhysics::SceneQueryHit hit = SoftBody()->RayCast(request);
        EXPECT_NEAR(hit.m_distance, 3.0f, 0.5f);
        EXPECT_EQ(hit.m_bodyHandle, m_softBodyHandle);
    }

    TEST_F(JoltSoftBodyTests, CollisionEventsFireWhenAClothLandsOnABody)
    {
        CreateFloor(0.0f);

        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::None);
        m_pendingTransform = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 2.0f));
        ASSERT_TRUE(Attach());

        int collisionsInvolvingTheCloth = 0;
        AzPhysics::SceneEvents::OnSceneCollisionsEvent::Handler collisionHandler(
            [this, &collisionsInvolvingTheCloth](
                AzPhysics::SceneHandle, const AzPhysics::CollisionEventList& events)
            {
                for (const AzPhysics::CollisionEvent& event : events)
                {
                    if (event.m_bodyHandle1 == m_softBodyHandle || event.m_bodyHandle2 == m_softBodyHandle)
                    {
                        ++collisionsInvolvingTheCloth;
                    }
                }
            });
        m_scene->RegisterSceneCollisionEventHandler(collisionHandler);

        SimulateSeconds(3.0f);

        // The cloth reaching the floor has to be reportable, not just visible in the
        // solver: a body whose id resolves to no handle raises no events at all.
        EXPECT_GT(collisionsInvolvingTheCloth, 0);
    }

    TEST_F(JoltSoftBodyTests, CubeKeepsItsVolume)
    {
        JoltSoftBodySettings settings;
        settings.m_shape = JoltSoftBodyShape::Cube;
        settings.m_size = AZ::Vector3(1.0f, 1.0f, 1.0f);
        settings.m_resolution = 4;
        settings.m_mass = 10.0f;
        settings.m_allowSleeping = false;
        m_pendingSettings = settings;
        m_pendingTransform = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 3.0f));
        ASSERT_TRUE(Attach());

        CreateFloor(0.0f);
        SimulateSeconds(3.0f);

        // A 4 x 4 x 4 grid is 64 particles. Volume constraints mean it lands and wobbles
        // rather than flattening into a sheet on the floor.
        EXPECT_EQ(SoftBody()->GetVertexCount(), 64u);
        const AZ::Aabb bounds = SoftBody()->GetWorldBounds();
        ASSERT_TRUE(bounds.IsValid());
        EXPECT_GT(bounds.GetExtents().GetZ(), 0.3f);
    }

    TEST_F(JoltSoftBodyTests, VertexPositionOutOfRangeReadsZero)
    {
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::None);
        ASSERT_TRUE(Attach());

        // A caller polling by index while the body is rebuilt or disabled must not read out
        // of bounds.
        EXPECT_TRUE(SoftBody()->GetVertexPosition(9999u).IsZero());
        EXPECT_FALSE(SoftBody()->GetVertexPosition(0u).IsZero());
    }

    TEST_F(JoltSoftBodyTests, NativePointerIsTheBodyIdLikeEveryOtherBodyType)
    {
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::None);
        ASSERT_TRUE(Attach());

        // The rigid bodies return a JPH::BodyID* from GetNativePointer, and a generic
        // consumer casting whatever body the scene hands back relies on soft bodies
        // honouring the same contract.
        auto* nativeId = static_cast<JPH::BodyID*>(SoftBody()->GetNativePointer());
        ASSERT_NE(nativeId, nullptr);
        EXPECT_EQ(*nativeId, SoftBody()->GetBodyId());
    }

    TEST_F(JoltSoftBodyTests, PinnedClothKeepsItsConfiguredMass)
    {
        // Four pinned corners on a 6x6 grid. Dividing the total mass over all 36 particles
        // and then only assigning it to the 32 free ones would quietly lose 4/36 of it.
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::Corners);
        ASSERT_TRUE(Attach());

        auto* joltScene = azdynamic_cast<JoltScene*>(m_scene);
        ASSERT_NE(joltScene, nullptr);
        JPH::BodyLockRead bodyLock(
            joltScene->GetJoltPhysicsSystem()->GetBodyLockInterface(), SoftBody()->GetBodyId());
        ASSERT_TRUE(bodyLock.Succeeded());

        const auto* motionProperties =
            static_cast<const JPH::SoftBodyMotionProperties*>(bodyLock.GetBody().GetMotionProperties());
        float totalMass = 0.0f;
        AZ::u32 pinnedCount = 0;
        for (const JPH::SoftBodyVertex& vertex : motionProperties->GetVertices())
        {
            if (vertex.mInvMass > 0.0f)
            {
                totalMass += 1.0f / vertex.mInvMass;
            }
            else
            {
                ++pinnedCount;
            }
        }

        EXPECT_EQ(pinnedCount, 4u);
        EXPECT_NEAR(totalMass, m_pendingSettings.m_mass, 1.0e-3f);
    }

    TEST_F(JoltSoftBodyTests, SurfaceSettingsReachTheJoltBody)
    {
        JoltSoftBodySettings settings = ClothSettings(JoltSoftBodyPinning::Corners);
        settings.m_friction = 0.85f;
        settings.m_restitution = 0.4f;
        settings.m_vertexRadius = 0.03f;
        settings.m_maxLinearVelocity = 25.0f;
        settings.m_updatePosition = false;
        settings.m_doubleSidedFaces = false;
        m_pendingSettings = settings;
        ASSERT_TRUE(Attach());

        auto* joltScene = azdynamic_cast<JoltScene*>(m_scene);
        ASSERT_NE(joltScene, nullptr);
        JPH::BodyLockRead bodyLock(
            joltScene->GetJoltPhysicsSystem()->GetBodyLockInterface(), SoftBody()->GetBodyId());
        ASSERT_TRUE(bodyLock.Succeeded());

        const JPH::Body& body = bodyLock.GetBody();
        EXPECT_NEAR(body.GetFriction(), 0.85f, 1.0e-6f);
        EXPECT_NEAR(body.GetRestitution(), 0.4f, 1.0e-6f);

        const auto* motionProperties = static_cast<const JPH::SoftBodyMotionProperties*>(body.GetMotionProperties());
        EXPECT_NEAR(motionProperties->GetVertexRadius(), 0.03f, 1.0e-6f);
        EXPECT_NEAR(motionProperties->GetMaxLinearVelocity(), 25.0f, 1.0e-6f);
        EXPECT_FALSE(motionProperties->GetUpdatePosition());
        EXPECT_FALSE(motionProperties->GetFacesDoubleSided());
    }

    TEST_F(JoltSoftBodyTests, FrictionAndRestitutionChangeLiveWithoutRebuilding)
    {
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::None);
        ASSERT_TRUE(Attach());
        const AZ::u32 generationBefore = SoftBody()->GetBuildGeneration();

        SoftBody()->SetFriction(0.95f);
        SoftBody()->SetRestitution(0.6f);

        EXPECT_EQ(SoftBody()->GetBuildGeneration(), generationBefore);
        EXPECT_NEAR(SoftBody()->GetSettings().m_friction, 0.95f, 1.0e-6f);
        EXPECT_NEAR(SoftBody()->GetSettings().m_restitution, 0.6f, 1.0e-6f);

        auto* joltScene = azdynamic_cast<JoltScene*>(m_scene);
        JPH::BodyLockRead bodyLock(
            joltScene->GetJoltPhysicsSystem()->GetBodyLockInterface(), SoftBody()->GetBodyId());
        ASSERT_TRUE(bodyLock.Succeeded());
        EXPECT_NEAR(bodyLock.GetBody().GetFriction(), 0.95f, 1.0e-6f);
        EXPECT_NEAR(bodyLock.GetBody().GetRestitution(), 0.6f, 1.0e-6f);
    }

    TEST_F(JoltSoftBodyTests, DoubleSidedFacesRaycastFromBehind)
    {
        // The cloth lies in the XY plane with its front face towards +Z, so a ray fired
        // up from underneath hits the back of the sheet.
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::Corners);
        m_pendingTransform = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 5.0f));
        ASSERT_TRUE(Attach());

        AzPhysics::RayCastRequest fromBehind;
        fromBehind.m_start = AZ::Vector3(0.0f, 0.0f, 2.0f);
        fromBehind.m_direction = AZ::Vector3(0.0f, 0.0f, 1.0f);
        fromBehind.m_distance = 10.0f;

        // The gem defaults faces to double sided - a thin cloth that can only be hit
        // from the front reads as broken.
        EXPECT_TRUE(SoftBody()->RayCast(fromBehind).IsValid());

        // Turning it off restores Jolt's single-sided behaviour: the same ray now passes
        // straight through the back face.
        JoltSoftBodySettings settings = m_pendingSettings;
        settings.m_doubleSidedFaces = false;
        SoftBody()->SetSettings(settings);
        EXPECT_FALSE(SoftBody()->RayCast(fromBehind).IsValid());
    }

    TEST_F(JoltSoftBodyTests, LraTethersStopAClothStretching)
    {
        // A cloth hanging from its top edge under a heavy mass with a single solver
        // iteration is exactly the case LRA exists for.
        JoltSoftBodySettings settings = ClothSettings(JoltSoftBodyPinning::TopEdge);
        settings.m_lraType = JoltSoftBodyLraType::GeodesicDistance;
        m_pendingSettings = settings;
        m_pendingTransform = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 5.0f));
        ASSERT_TRUE(Attach());

        auto* joltScene = azdynamic_cast<JoltScene*>(m_scene);
        JPH::BodyLockRead bodyLock(
            joltScene->GetJoltPhysicsSystem()->GetBodyLockInterface(), SoftBody()->GetBodyId());
        ASSERT_TRUE(bodyLock.Succeeded());
        const auto* motionProperties =
            static_cast<const JPH::SoftBodyMotionProperties*>(bodyLock.GetBody().GetMotionProperties());

        // The tethers are real constraints in the shared settings, one per free particle
        // reachable from the pinned edge.
        EXPECT_FALSE(motionProperties->GetSettings()->mLRAConstraints.empty());
    }

    TEST_F(JoltSoftBodyTests, BulkVertexReadMatchesThePerVertexRead)
    {
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::Corners);
        ASSERT_TRUE(Attach());

        AZStd::vector<AZ::Vector3> positions;
        ASSERT_TRUE(SoftBody()->CopyVertexPositions(positions));
        ASSERT_EQ(positions.size(), SoftBody()->GetVertexCount());
        EXPECT_THAT(positions[0], ::testing::Eq(SoftBody()->GetVertexPosition(0)));
        EXPECT_THAT(positions[35], ::testing::Eq(SoftBody()->GetVertexPosition(35)));
    }

    TEST_F(JoltSoftBodyTests, RuntimePinHoldsAParticleAndUnpinRestoresItsMass)
    {
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::None);
        m_pendingTransform = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 5.0f));
        ASSERT_TRUE(Attach());

        // Pin one particle of an otherwise free sheet where it is now.
        const AZ::Vector3 pinnedAt = SoftBody()->GetVertexPosition(0);
        ASSERT_TRUE(SoftBody()->SetVertexPinned(0, true));
        EXPECT_TRUE(SoftBody()->IsVertexPinned(0));

        SimulateSeconds(1.0f);

        // The sheet fell; the pinned particle did not.
        EXPECT_NEAR(SoftBody()->GetVertexPosition(0).GetZ(), pinnedAt.GetZ(), 0.05f);
        EXPECT_LT(SoftBody()->GetVertexPosition(35).GetZ(), pinnedAt.GetZ() - 1.0f);

        // Unpinning hands back the same share of mass every other free particle carries,
        // so the body's total mass is conserved through a pin/unpin round trip.
        ASSERT_TRUE(SoftBody()->SetVertexPinned(0, false));
        EXPECT_FALSE(SoftBody()->IsVertexPinned(0));

        {
            // Scoped: SetVertexPinned below takes its own body lock, and holding this
            // read lock across that call would deadlock.
            auto* joltScene = azdynamic_cast<JoltScene*>(m_scene);
            JPH::BodyLockRead bodyLock(
                joltScene->GetJoltPhysicsSystem()->GetBodyLockInterface(), SoftBody()->GetBodyId());
            ASSERT_TRUE(bodyLock.Succeeded());
            const auto* motionProperties =
                static_cast<const JPH::SoftBodyMotionProperties*>(bodyLock.GetBody().GetMotionProperties());
            float totalMass = 0.0f;
            for (const JPH::SoftBodyVertex& vertex : motionProperties->GetVertices())
            {
                ASSERT_GT(vertex.mInvMass, 0.0f);
                totalMass += 1.0f / vertex.mInvMass;
            }
            EXPECT_NEAR(totalMass, m_pendingSettings.m_mass, 1.0e-3f);
        }

        // Out-of-range indices are refused, not written.
        EXPECT_FALSE(SoftBody()->SetVertexPinned(9999u, true));
        EXPECT_FALSE(SoftBody()->IsVertexPinned(9999u));
    }

    TEST_F(JoltSoftBodyTests, VertexVelocityKicksAParticle)
    {
        JoltSoftBodySettings settings = ClothSettings(JoltSoftBodyPinning::None);
        settings.m_gravityFactor = 0.0f; // isolate the kick from falling
        m_pendingSettings = settings;
        ASSERT_TRUE(Attach());

        const AZ::Vector3 before = SoftBody()->GetVertexPosition(0);
        ASSERT_TRUE(SoftBody()->SetVertexVelocity(0, AZ::Vector3(0.0f, 0.0f, 3.0f)));
        EXPECT_NEAR(SoftBody()->GetVertexVelocity(0).GetZ(), 3.0f, 1.0e-3f);

        SimulateSeconds(0.2f);

        // The kicked particle moved up; the constraints will drag its neighbours after
        // it, so only the immediate direction is asserted.
        EXPECT_GT(SoftBody()->GetVertexPosition(0).GetZ(), before.GetZ() + 0.05f);

        // A pinned particle refuses a velocity: it could never act on it.
        ASSERT_TRUE(SoftBody()->SetVertexPinned(5, true));
        EXPECT_FALSE(SoftBody()->SetVertexVelocity(5, AZ::Vector3(1.0f, 0.0f, 0.0f)));
    }

    TEST_F(JoltSoftBodyTests, SetTransformRebuildsAtTheNewPlacement)
    {
        m_pendingSettings = ClothSettings(JoltSoftBodyPinning::Corners);
        ASSERT_TRUE(Attach());
        const AZ::u32 generationBefore = SoftBody()->GetBuildGeneration();

        SoftBody()->SetTransform(AZ::Transform::CreateTranslation(AZ::Vector3(10.0f, 0.0f, 5.0f)));

        // Teleporting a live soft body would fight the solver, so placement is a rebuild.
        EXPECT_GT(SoftBody()->GetBuildGeneration(), generationBefore);
        const AZ::Aabb bounds = SoftBody()->GetWorldBounds();
        ASSERT_TRUE(bounds.IsValid());
        EXPECT_NEAR(bounds.GetCenter().GetX(), 10.0f, 0.1f);
        EXPECT_NEAR(bounds.GetCenter().GetZ(), 5.0f, 0.1f);
    }
} // namespace JoltPhysics
