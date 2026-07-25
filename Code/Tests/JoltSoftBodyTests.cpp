#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <Configuration/JoltSettingsRegistryManager.h>
#include <Scene/JoltScene.h>
#include <SoftBody/JoltSoftBody.h>
#include <System/JoltSystem.h>

#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

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
            m_softBody.Detach();
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

        bool Attach()
        {
            return m_softBody.Attach(m_sceneHandle);
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
        JoltSoftBody m_softBody;
        AzPhysics::CollisionGroups::Id m_noSoftBodiesGroupId;
    };

    TEST_F(JoltSoftBodyTests, AttachingBuildsParticlesAndTriangles)
    {
        m_softBody.SetSettings(ClothSettings(JoltSoftBodyPinning::None));
        ASSERT_TRUE(Attach());
        EXPECT_TRUE(m_softBody.IsAttached());

        // A 6 x 6 grid is 36 particles and 25 quads, so 50 triangles.
        EXPECT_EQ(m_softBody.GetVertexCount(), 36u);
        EXPECT_EQ(m_softBody.GetTriangleIndices().size(), 50u * 3u);
    }

    TEST_F(JoltSoftBodyTests, AttachingToAnInvalidSceneFails)
    {
        EXPECT_FALSE(m_softBody.Attach(AzPhysics::InvalidSceneHandle));
        EXPECT_FALSE(m_softBody.IsAttached());
        EXPECT_EQ(m_softBody.GetVertexCount(), 0u);
    }

    TEST_F(JoltSoftBodyTests, UnpinnedClothFalls)
    {
        m_softBody.SetSettings(ClothSettings(JoltSoftBodyPinning::None));
        m_softBody.SetTransform(AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 5.0f)));
        ASSERT_TRUE(Attach());

        SimulateSeconds(1.0f);

        // Roughly a second of free fall is about -4.9 m, so it should be well below where it
        // started with nothing holding it up.
        const AZ::Aabb bounds = m_softBody.GetWorldBounds();
        ASSERT_TRUE(bounds.IsValid());
        EXPECT_LT(bounds.GetMax().GetZ(), 3.0f);
    }

    TEST_F(JoltSoftBodyTests, PinnedCornersHoldTheClothUp)
    {
        m_softBody.SetSettings(ClothSettings(JoltSoftBodyPinning::Corners));
        m_softBody.SetTransform(AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 5.0f)));
        ASSERT_TRUE(Attach());

        SimulateSeconds(2.0f);

        // The pinned corners cannot move, so the sheet sags between them but its top stays at
        // the height it was created at - the contrast with UnpinnedClothFalls is the point.
        const AZ::Aabb bounds = m_softBody.GetWorldBounds();
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

        m_softBody.SetSettings(ClothSettings(JoltSoftBodyPinning::None));
        m_softBody.SetTransform(AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 2.0f)));
        ASSERT_TRUE(Attach());

        SimulateSeconds(3.0f);

        // Collision against a rigid body created through this gem's own configuration path is
        // what makes a soft body useful rather than a curiosity.
        const AZ::Aabb bounds = m_softBody.GetWorldBounds();
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
        m_softBody.SetSettings(settings);
        ASSERT_TRUE(Attach());

        SimulateSeconds(0.5f);
        const AZ::Aabb deflated = m_softBody.GetWorldBounds();
        ASSERT_TRUE(deflated.IsValid());

        // Raising pressure on a live body must take effect without a rebuild. This also pins
        // the sphere's face winding: Jolt derives the enclosed volume from it and silently
        // skips pressure when that volume comes out negative.
        m_softBody.SetPressure(5000.0f);
        SimulateSeconds(1.0f);

        const AZ::Aabb inflated = m_softBody.GetWorldBounds();
        ASSERT_TRUE(inflated.IsValid());
        EXPECT_GT(inflated.GetExtents().GetMaxElement(), deflated.GetExtents().GetMaxElement());
    }

    TEST_F(JoltSoftBodyTests, GravityFactorStopsTheBodyFalling)
    {
        JoltSoftBodySettings settings = ClothSettings(JoltSoftBodyPinning::None);
        settings.m_gravityFactor = 0.0f;
        m_softBody.SetSettings(settings);
        m_softBody.SetTransform(AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 5.0f)));
        ASSERT_TRUE(Attach());

        SimulateSeconds(1.0f);

        const AZ::Aabb bounds = m_softBody.GetWorldBounds();
        ASSERT_TRUE(bounds.IsValid());
        EXPECT_NEAR(bounds.GetMax().GetZ(), 5.0f, 0.1f);
    }

    TEST_F(JoltSoftBodyTests, ChangingABakedSettingRebuildsInPlace)
    {
        m_softBody.SetSettings(ClothSettings(JoltSoftBodyPinning::None));
        ASSERT_TRUE(Attach());

        const AZ::u32 generationBefore = m_softBody.GetBuildGeneration();
        EXPECT_EQ(m_softBody.GetVertexCount(), 36u);

        JoltSoftBodySettings settings = ClothSettings(JoltSoftBodyPinning::None);
        settings.m_resolution = 4; // baked: needs a new particle layout
        m_softBody.SetSettings(settings);

        EXPECT_GT(m_softBody.GetBuildGeneration(), generationBefore);
        EXPECT_EQ(m_softBody.GetVertexCount(), 16u);
        EXPECT_TRUE(m_softBody.IsAttached());
    }

    TEST_F(JoltSoftBodyTests, ChangingALiveSettingDoesNotRebuild)
    {
        m_softBody.SetSettings(ClothSettings(JoltSoftBodyPinning::None));
        ASSERT_TRUE(Attach());

        const AZ::u32 generationBefore = m_softBody.GetBuildGeneration();

        JoltSoftBodySettings settings = ClothSettings(JoltSoftBodyPinning::None);
        settings.m_linearDamping = 0.9f; // live: forwarded to the existing body
        settings.m_numIterations = 12;
        m_softBody.SetSettings(settings);

        // Rebuilding here would throw away the simulated state every time gameplay nudged a
        // value, which is the whole reason the settings are split into baked and live.
        EXPECT_EQ(m_softBody.GetBuildGeneration(), generationBefore);
        EXPECT_EQ(m_softBody.GetSettings().m_numIterations, 12u);
    }

    TEST_F(JoltSoftBodyTests, CollisionLayerAndGroupReachTheBody)
    {
        JoltSoftBodySettings settings = ClothSettings(JoltSoftBodyPinning::None);
        settings.m_collisionLayer = AzPhysics::CollisionLayer(SoftBodyLayerIndex);
        settings.m_collisionGroupId = m_noSoftBodiesGroupId;
        m_softBody.SetSettings(settings);
        ASSERT_TRUE(Attach());

        // A distinct (layer, group) pair has to resolve to its own object layer, not the
        // catch-all Moving layer every unconfigured body lands on.
        const JPH::ObjectLayer configuredLayer = m_softBody.GetObjectLayer();
        EXPECT_NE(configuredLayer, ObjectLayers::Moving);
        EXPECT_NE(configuredLayer, ObjectLayers::NonMoving);

        JoltSoftBody defaultBody;
        defaultBody.SetSettings(ClothSettings(JoltSoftBodyPinning::None));
        ASSERT_TRUE(defaultBody.Attach(m_sceneHandle));
        EXPECT_NE(defaultBody.GetObjectLayer(), configuredLayer);
        defaultBody.Detach();
    }

    TEST_F(JoltSoftBodyTests, ClothFallsThroughAFloorThatExcludesItsLayer)
    {
        // The floor collides with everything except the soft body layer.
        CreateFloor(0.0f, m_noSoftBodiesGroupId);

        JoltSoftBodySettings settings = ClothSettings(JoltSoftBodyPinning::None);
        settings.m_collisionLayer = AzPhysics::CollisionLayer(SoftBodyLayerIndex);
        m_softBody.SetSettings(settings);
        m_softBody.SetTransform(AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 2.0f)));
        ASSERT_TRUE(Attach());

        SimulateSeconds(3.0f);

        // Well below the floor: the filtering is what is being tested, so the assertion has
        // to be one that a merely-slow landing could not satisfy. ClothLandsOnAFloorAndStays        // AboveIt is the same setup with an unfiltered floor and asserts the opposite.
        const AZ::Aabb bounds = m_softBody.GetWorldBounds();
        ASSERT_TRUE(bounds.IsValid());
        EXPECT_LT(bounds.GetMax().GetZ(), -2.0f);
    }

    TEST_F(JoltSoftBodyTests, ChangingTheCollisionLayerMovesTheBodyWithoutRebuilding)
    {
        m_softBody.SetSettings(ClothSettings(JoltSoftBodyPinning::None));
        ASSERT_TRUE(Attach());

        const AZ::u32 generationBefore = m_softBody.GetBuildGeneration();
        const JPH::ObjectLayer layerBefore = m_softBody.GetObjectLayer();

        m_softBody.SetCollisionLayer(AzPhysics::CollisionLayer(SoftBodyLayerIndex));

        // Jolt can move a live body between object layers, and rebuilding would discard
        // whatever deformation the body had settled into.
        EXPECT_EQ(m_softBody.GetBuildGeneration(), generationBefore);
        EXPECT_NE(m_softBody.GetObjectLayer(), layerBefore);
        EXPECT_EQ(m_softBody.GetSettings().m_collisionLayer.GetIndex(), SoftBodyLayerIndex);
    }

    TEST_F(JoltSoftBodyTests, DetachRemovesTheBody)
    {
        m_softBody.SetSettings(ClothSettings(JoltSoftBodyPinning::None));
        ASSERT_TRUE(Attach());
        ASSERT_EQ(m_softBody.GetVertexCount(), 36u);

        m_softBody.Detach();

        EXPECT_FALSE(m_softBody.IsAttached());
        EXPECT_EQ(m_softBody.GetVertexCount(), 0u);
        EXPECT_FALSE(m_softBody.GetWorldBounds().IsValid());

        // Stepping a scene the body was removed from must not touch freed memory.
        SimulateSeconds(0.5f);

        // And it can be brought back, which is how the component's enable toggle works.
        EXPECT_TRUE(Attach());
        EXPECT_EQ(m_softBody.GetVertexCount(), 36u);
    }

    TEST_F(JoltSoftBodyTests, CubeKeepsItsVolume)
    {
        JoltSoftBodySettings settings;
        settings.m_shape = JoltSoftBodyShape::Cube;
        settings.m_size = AZ::Vector3(1.0f, 1.0f, 1.0f);
        settings.m_resolution = 4;
        settings.m_mass = 10.0f;
        settings.m_allowSleeping = false;
        m_softBody.SetSettings(settings);
        m_softBody.SetTransform(AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 3.0f)));
        ASSERT_TRUE(Attach());

        CreateFloor(0.0f);
        SimulateSeconds(3.0f);

        // A 4 x 4 x 4 grid is 64 particles. Volume constraints mean it lands and wobbles
        // rather than flattening into a sheet on the floor.
        EXPECT_EQ(m_softBody.GetVertexCount(), 64u);
        const AZ::Aabb bounds = m_softBody.GetWorldBounds();
        ASSERT_TRUE(bounds.IsValid());
        EXPECT_GT(bounds.GetExtents().GetZ(), 0.3f);
    }

    TEST_F(JoltSoftBodyTests, VertexPositionOutOfRangeReadsZero)
    {
        m_softBody.SetSettings(ClothSettings(JoltSoftBodyPinning::None));
        ASSERT_TRUE(Attach());

        // A caller polling by index while the body is rebuilt or disabled must not read out
        // of bounds.
        EXPECT_TRUE(m_softBody.GetVertexPosition(9999u).IsZero());
        EXPECT_FALSE(m_softBody.GetVertexPosition(0u).IsZero());
    }
} // namespace JoltPhysics
