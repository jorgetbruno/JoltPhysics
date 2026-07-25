#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <Character/JoltRagdoll.h>
#include <Character/JoltSkeletonMapper.h>
#include <Configuration/JoltSettingsRegistryManager.h>
#include <Joint/JoltJointConfiguration.h>
#include <Scene/JoltScene.h>
#include <System/JoltSystem.h>

#include <AzFramework/Physics/Ragdoll.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    class JoltRagdollTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "RagdollTestScene";
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

        //! Fills a two-node ragdoll config: a root sphere and a child sphere 0.5 m below it.
        //! (RagdollConfiguration has an explicit copy constructor, so it is filled in place.)
        void MakeTwoNodeRagdoll(Physics::RagdollConfiguration& config, float rootZ)
        {
            auto addNode = [&config](const char* name, const AZ::Vector3& position, size_t parentIndex)
            {
                Physics::RagdollNodeConfiguration node;
                node.m_debugName = name;
                node.m_mass = 1.0f;
                config.m_nodes.push_back(node);
                config.m_parentIndices.push_back(parentIndex);

                Physics::CharacterColliderNodeConfiguration collider;
                collider.m_name = name;
                collider.m_shapes.emplace_back(
                    AZStd::make_shared<Physics::ColliderConfiguration>(),
                    AZStd::make_shared<Physics::SphereShapeConfiguration>(0.15f));
                config.m_colliders.m_nodes.push_back(collider);

                Physics::RagdollNodeState state;
                state.m_position = position;
                config.m_initialState.push_back(state);
            };

            const size_t noParent = static_cast<size_t>(-1);
            addNode("root", AZ::Vector3(0.0f, 0.0f, rootZ), noParent);
            addNode("child", AZ::Vector3(0.0f, 0.0f, rootZ - 0.5f), /*parent*/ 0);
        }

        //! Gives the child node an articulated swing-twist joint at the midpoint between
        //! the bodies (motors only exist on these; a jointless node gets a point
        //! constraint, which cannot be driven).
        static void AddSwingTwistJointToChild(Physics::RagdollConfiguration& config)
        {
            auto jointConfig = AZStd::make_shared<JoltSwingTwistJointConfiguration>();
            jointConfig->m_parentLocalPosition = AZ::Vector3(0.0f, 0.0f, -0.25f);
            jointConfig->m_childLocalPosition = AZ::Vector3(0.0f, 0.0f, 0.25f);
            config.m_nodes[1].m_jointConfig = jointConfig;
        }

        //! Drives a ragdoll into a static wall with the given keying mode; returns the
        //! root node's final x position.
        float DriveIntoWallAndMeasureTravel(bool useHardKeying);

        //! The angle of the child's orientation relative to the root, in degrees.
        static float GetChildAngleRelativeToRootDegrees(const Physics::RagdollState& state)
        {
            const AZ::Quaternion relative = state[0].m_orientation.GetConjugate() * state[1].m_orientation;
            // The sign of w is irrelevant (q and -q are the same rotation), so fold it away
            // before taking the angle.
            const float halfAngleCosine = AZStd::min(AZStd::abs(relative.GetW()), 1.0f);
            return AZ::RadToDeg(2.0f * acosf(halfAngleCosine));
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle;
        AzPhysics::Scene* m_scene = nullptr;
    };

    TEST_F(JoltRagdollTests, RagdollFallsAndStaysConnected)
    {
        Physics::RagdollConfiguration config;
        MakeTwoNodeRagdoll(config, 5.0f);
        auto handle = m_scene->AddSimulatedBody(&config);
        ASSERT_NE(handle, AzPhysics::InvalidSimulatedBodyHandle);

        auto* ragdoll = azdynamic_cast<JoltRagdoll*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(ragdoll, nullptr);
        EXPECT_EQ(ragdoll->GetNumNodes(), 2u);
        EXPECT_FALSE(ragdoll->IsSimulated());

        ragdoll->EnableSimulation(config.m_initialState);
        EXPECT_TRUE(ragdoll->IsSimulated());

        SimulateSeconds(1.0f);

        Physics::RagdollState state;
        ragdoll->GetState(state);
        ASSERT_EQ(state.size(), 2u);

        // Both bodies fell under gravity...
        EXPECT_LT(state[0].m_position.GetZ(), 5.0f - 1.0f);
        EXPECT_LT(state[1].m_position.GetZ(), 4.5f - 1.0f);
        EXPECT_TRUE(state[0].m_position.IsFinite());
        EXPECT_TRUE(state[1].m_position.IsFinite());

        // ...and the point constraint kept the two nodes attached (roughly the original 0.5 m).
        const float separation = state[0].m_position.GetDistance(state[1].m_position);
        EXPECT_NEAR(separation, 0.5f, 0.2f);
    }

    TEST_F(JoltRagdollTests, RagdollCollidesWithStaticWorldGeometry)
    {
        // Regression: ragdoll parts carry Jolt's own GroupFilterTable (installed to
        // disable parent/child collisions), and the gem's group filter used to read that
        // foreign filter as if it were its own, so ragdolls fell through the world.
        auto floorCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        floorCollider->m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        auto floorShape = AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(20.0f, 20.0f, 1.0f));
        AzPhysics::StaticRigidBodyConfiguration floorConfig;
        floorConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(floorCollider, floorShape);
        m_scene->AddSimulatedBody(&floorConfig);

        Physics::RagdollConfiguration config;
        MakeTwoNodeRagdoll(config, 3.0f);
        auto handle = m_scene->AddSimulatedBody(&config);
        auto* ragdoll = azdynamic_cast<JoltRagdoll*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(ragdoll, nullptr);
        ragdoll->EnableSimulation(config.m_initialState);

        SimulateSeconds(3.0f);

        Physics::RagdollState state;
        ragdoll->GetState(state);
        // Both 0.15 m spheres come to rest on the floor instead of falling through it.
        EXPECT_GT(state[0].m_position.GetZ(), 0.0f);
        EXPECT_GT(state[1].m_position.GetZ(), 0.0f);
        EXPECT_NEAR(state[1].m_position.GetZ(), 0.15f, 0.1f);
    }

    TEST_F(JoltRagdollTests, DisableSimulationStopsTheRagdoll)
    {
        Physics::RagdollConfiguration config;
        MakeTwoNodeRagdoll(config, 5.0f);
        auto handle = m_scene->AddSimulatedBody(&config);
        auto* ragdoll = azdynamic_cast<JoltRagdoll*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(ragdoll, nullptr);

        ragdoll->EnableSimulation(config.m_initialState);
        SimulateSeconds(0.5f);
        ragdoll->DisableSimulation();
        EXPECT_FALSE(ragdoll->IsSimulated());

        Physics::RagdollState afterDisable;
        ragdoll->GetState(afterDisable);
        const float zAfterDisable = afterDisable[0].m_position.GetZ();

        // With simulation disabled the bodies no longer move.
        SimulateSeconds(0.5f);
        Physics::RagdollState later;
        ragdoll->GetState(later);
        EXPECT_NEAR(later[0].m_position.GetZ(), zAfterDisable, 0.01f);
    }

    TEST_F(JoltRagdollTests, NodesAreAccessibleAsRigidBodiesWithArticulatedJoint)
    {
        Physics::RagdollConfiguration config;
        MakeTwoNodeRagdoll(config, 5.0f);

        // Give the child an articulated swing-twist joint at the midpoint between the bodies.
        auto jointConfig = AZStd::make_shared<JoltSwingTwistJointConfiguration>();
        jointConfig->m_parentLocalPosition = AZ::Vector3(0.0f, 0.0f, -0.25f);
        jointConfig->m_childLocalPosition = AZ::Vector3(0.0f, 0.0f, 0.25f);
        config.m_nodes[1].m_jointConfig = jointConfig;

        auto handle = m_scene->AddSimulatedBody(&config);
        auto* ragdoll = azdynamic_cast<JoltRagdoll*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(ragdoll, nullptr);

        ragdoll->EnableSimulation(config.m_initialState);
        SimulateSeconds(1.0f);

        // Per-node access is available and reflects the simulation.
        Physics::RagdollNode* node0 = ragdoll->GetNode(0);
        Physics::RagdollNode* node1 = ragdoll->GetNode(1);
        ASSERT_NE(node0, nullptr);
        ASSERT_NE(node1, nullptr);
        EXPECT_TRUE(node0->IsSimulating());

        // GetRigidBody() wraps the same body the ragdoll state reports.
        Physics::RagdollState state;
        ragdoll->GetState(state);
        ASSERT_EQ(state.size(), 2u);
        EXPECT_TRUE(node0->GetRigidBody().GetPosition().IsClose(state[0].m_position, 0.05f));
        EXPECT_TRUE(node1->GetRigidBody().GetPosition().IsClose(state[1].m_position, 0.05f));

        // The articulated joint still holds the bodies together (~0.5 m apart).
        const float separation = state[0].m_position.GetDistance(state[1].m_position);
        EXPECT_NEAR(separation, 0.5f, 0.25f);
    }

    TEST_F(JoltRagdollTests, NodeJointIsAccessible)
    {
        Physics::RagdollConfiguration config;
        MakeTwoNodeRagdoll(config, 5.0f);
        auto jointConfig = AZStd::make_shared<JoltSwingTwistJointConfiguration>();
        jointConfig->m_parentLocalPosition = AZ::Vector3(0.0f, 0.0f, -0.25f);
        jointConfig->m_childLocalPosition = AZ::Vector3(0.0f, 0.0f, 0.25f);
        config.m_nodes[1].m_jointConfig = jointConfig;

        auto handle = m_scene->AddSimulatedBody(&config);
        auto* ragdoll = azdynamic_cast<JoltRagdoll*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(ragdoll, nullptr);

        EXPECT_EQ(ragdoll->GetNode(0)->GetJoint(), nullptr); // the root has no joint to a parent
        AzPhysics::Joint* childJoint = ragdoll->GetNode(1)->GetJoint();
        ASSERT_NE(childJoint, nullptr); // the child's joint to the root
        EXPECT_NE(childJoint->GetNativePointer(), nullptr);
    }

    TEST_F(JoltRagdollTests, DriveToPoseUsingKinematicsFollowsTarget)
    {
        Physics::RagdollConfiguration config;
        MakeTwoNodeRagdoll(config, 5.0f);
        auto handle = m_scene->AddSimulatedBody(&config);
        auto* ragdoll = azdynamic_cast<JoltRagdoll*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(ragdoll, nullptr);
        ragdoll->EnableSimulation(config.m_initialState);

        // Target pose: both nodes shifted +1 m in x, holding their heights.
        Physics::RagdollState target;
        Physics::RagdollNodeState rootTarget;
        rootTarget.m_position = AZ::Vector3(1.0f, 0.0f, 5.0f);
        target.push_back(rootTarget);
        Physics::RagdollNodeState childTarget;
        childTarget.m_position = AZ::Vector3(1.0f, 0.0f, 4.5f);
        target.push_back(childTarget);

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 30; ++i)
        {
            ragdoll->DriveToPoseUsingKinematics(target, dt);
            m_scene->StartSimulation(dt);
            m_scene->FinishSimulation();
        }

        Physics::RagdollState state;
        ragdoll->GetState(state);
        // The kinematic drive follows the target in x and holds height against gravity.
        EXPECT_NEAR(state[0].m_position.GetX(), 1.0f, 0.1f);
        EXPECT_NEAR(state[0].m_position.GetZ(), 5.0f, 0.2f);
        EXPECT_NEAR(state[1].m_position.GetX(), 1.0f, 0.1f);
    }

    //! Animation skeleton matching the two-node ragdoll ("root", "child") but with an
    //! extra joint between them and a leaf the ragdoll does not have - the shape a real
    //! animation skeleton has relative to its ragdoll.
    static JoltAnimationSkeleton MakeAnimationSkeleton(float rootZ)
    {
        JoltAnimationSkeleton skeleton;
        auto addJoint = [&skeleton](const char* name, int parentIndex, const AZ::Vector3& position)
        {
            skeleton.m_jointNames.push_back(name);
            skeleton.m_parentIndices.push_back(parentIndex);
            skeleton.m_neutralPoseModelSpace.push_back(AZ::Transform::CreateTranslation(position));
        };

        addJoint("root", -1, AZ::Vector3(0.0f, 0.0f, rootZ));           // mapped
        addJoint("spine", 0, AZ::Vector3(0.0f, 0.0f, rootZ - 0.25f));   // between the two
        addJoint("child", 1, AZ::Vector3(0.0f, 0.0f, rootZ - 0.5f));    // mapped
        addJoint("toe", 2, AZ::Vector3(0.0f, 0.0f, rootZ - 0.75f));     // leaf beyond
        return skeleton;
    }

    TEST_F(JoltRagdollTests, SkeletonMapperMapsRagdollPoseOntoTheAnimationSkeleton)
    {
        Physics::RagdollConfiguration config;
        MakeTwoNodeRagdoll(config, 5.0f);
        auto handle = m_scene->AddSimulatedBody(&config);
        auto* ragdoll = azdynamic_cast<JoltRagdoll*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(ragdoll, nullptr);

        JoltSkeletonMapper mapper;
        ASSERT_TRUE(mapper.Initialize(*ragdoll, MakeAnimationSkeleton(5.0f)));
        EXPECT_TRUE(mapper.IsInitialized());
        // "root" and "child" match by name; "spine" and "toe" have no ragdoll counterpart.
        EXPECT_EQ(mapper.GetMappedJointCount(), 2u);

        ragdoll->EnableSimulation(config.m_initialState);
        SimulateSeconds(1.0f);

        Physics::RagdollState state;
        ragdoll->GetState(state);

        AZStd::vector<AZ::Transform> animationPose;
        ASSERT_TRUE(mapper.MapRagdollStateToAnimationPose(state, animationPose));
        ASSERT_EQ(animationPose.size(), 4u);

        // The mapped joints follow the simulated bodies exactly...
        EXPECT_TRUE(animationPose[0].GetTranslation().IsClose(state[0].m_position, 0.01f));
        EXPECT_TRUE(animationPose[2].GetTranslation().IsClose(state[1].m_position, 0.01f));
        // ...and the ragdoll has fallen, so the whole mapped pose came down with it.
        EXPECT_LT(animationPose[0].GetTranslation().GetZ(), 4.0f);
        // The in-between joint is carried along, staying between its neighbours.
        const float spineZ = animationPose[1].GetTranslation().GetZ();
        EXPECT_LE(spineZ, animationPose[0].GetTranslation().GetZ() + 0.01f);
        EXPECT_GE(spineZ, animationPose[2].GetTranslation().GetZ() - 0.01f);
    }

    TEST_F(JoltRagdollTests, SkeletonMapperMapsAnimationPoseBackOntoTheRagdoll)
    {
        Physics::RagdollConfiguration config;
        MakeTwoNodeRagdoll(config, 5.0f);
        auto handle = m_scene->AddSimulatedBody(&config);
        auto* ragdoll = azdynamic_cast<JoltRagdoll*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(ragdoll, nullptr);

        JoltSkeletonMapper mapper;
        ASSERT_TRUE(mapper.Initialize(*ragdoll, MakeAnimationSkeleton(5.0f)));

        // An animated pose displaced 2 m along x from the neutral pose.
        JoltAnimationSkeleton animated = MakeAnimationSkeleton(5.0f);
        AZStd::vector<AZ::Transform> animationPose = animated.m_neutralPoseModelSpace;
        for (AZ::Transform& jointTransform : animationPose)
        {
            jointTransform.SetTranslation(jointTransform.GetTranslation() + AZ::Vector3(2.0f, 0.0f, 0.0f));
        }

        Physics::RagdollState ragdollPose;
        ASSERT_TRUE(mapper.MapAnimationPoseToRagdollState(animationPose, ragdollPose));
        ASSERT_EQ(ragdollPose.size(), 2u);

        // The ragdoll nodes land on their animation counterparts.
        EXPECT_TRUE(ragdollPose[0].m_position.IsClose(AZ::Vector3(2.0f, 0.0f, 5.0f), 0.01f));
        EXPECT_TRUE(ragdollPose[1].m_position.IsClose(AZ::Vector3(2.0f, 0.0f, 4.5f), 0.01f));

        // The mapped pose is usable as a ragdoll pose: keying to it moves the bodies there.
        ragdoll->EnableSimulation(ragdollPose);
        Physics::RagdollState state;
        ragdoll->GetState(state);
        EXPECT_TRUE(state[0].m_position.IsClose(AZ::Vector3(2.0f, 0.0f, 5.0f), 0.05f));
    }

    TEST_F(JoltRagdollTests, SkeletonMapperRejectsASkeletonThatSharesNoJointNames)
    {
        Physics::RagdollConfiguration config;
        MakeTwoNodeRagdoll(config, 5.0f);
        auto handle = m_scene->AddSimulatedBody(&config);
        auto* ragdoll = azdynamic_cast<JoltRagdoll*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(ragdoll, nullptr);

        JoltAnimationSkeleton unrelated = MakeAnimationSkeleton(5.0f);
        for (AZStd::string& jointName : unrelated.m_jointNames)
        {
            jointName = "other_" + jointName;
        }

        // Joints are matched by name, so nothing maps and initialization fails rather
        // than silently producing an identity mapping.
        JoltSkeletonMapper mapper;
        EXPECT_FALSE(mapper.Initialize(*ragdoll, unrelated));
        EXPECT_FALSE(mapper.IsInitialized());

        Physics::RagdollState state;
        AZStd::vector<AZ::Transform> pose;
        EXPECT_FALSE(mapper.MapRagdollStateToAnimationPose(state, pose));
    }

    TEST_F(JoltRagdollTests, PerBodyRayCastHitsNodesAndReportsTheRagdoll)
    {
        Physics::RagdollConfiguration config;
        MakeTwoNodeRagdoll(config, 5.0f);
        auto handle = m_scene->AddSimulatedBody(&config);
        auto* ragdoll = azdynamic_cast<JoltRagdoll*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(ragdoll, nullptr);
        ragdoll->EnableSimulation(config.m_initialState);

        // Straight down through the root node (a 0.15 m sphere centred at z=5).
        AzPhysics::RayCastRequest request;
        request.m_start = AZ::Vector3(0.0f, 0.0f, 8.0f);
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 10.0f;

        AzPhysics::SceneQueryHit hit = ragdoll->RayCast(request);
        ASSERT_TRUE(static_cast<bool>(hit));
        // The nearest node is hit, and the hit is attributed to the ragdoll itself.
        EXPECT_EQ(hit.m_bodyHandle, handle);
        EXPECT_NEAR(hit.m_position.GetZ(), 5.15f, 0.05f);

        // A ray that misses every node reports no hit.
        request.m_start = AZ::Vector3(3.0f, 0.0f, 8.0f);
        EXPECT_FALSE(static_cast<bool>(ragdoll->RayCast(request)));
    }

    //! Drives a two-node ragdoll sideways into a static wall and reports how far the root
    //! got. Hard keying should drive through it; soft keying should be stopped by it.
    float JoltRagdollTests::DriveIntoWallAndMeasureTravel(bool useHardKeying)
    {
        // Wall spanning x = [1.5, 2.0], tall and wide enough that the ragdoll cannot pass.
        auto wallCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto wallShape = AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(0.5f, 10.0f, 10.0f));
        AzPhysics::StaticRigidBodyConfiguration wallConfig;
        wallConfig.m_position = AZ::Vector3(1.75f, 0.0f, 5.0f);
        wallConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(wallCollider, wallShape);
        m_scene->AddSimulatedBody(&wallConfig);

        Physics::RagdollConfiguration config;
        MakeTwoNodeRagdoll(config, 5.0f);
        auto handle = m_scene->AddSimulatedBody(&config);
        auto* ragdoll = azdynamic_cast<JoltRagdoll*>(m_scene->GetSimulatedBodyFromHandle(handle));
        EXPECT_NE(ragdoll, nullptr);
        ragdoll->EnableSimulation(config.m_initialState);

        // Target well beyond the wall, at the ragdoll's starting height.
        Physics::RagdollState target = config.m_initialState;
        for (Physics::RagdollNodeState& nodeState : target)
        {
            nodeState.m_position += AZ::Vector3(4.0f, 0.0f, 0.0f);
        }

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 120; ++i)
        {
            if (useHardKeying)
            {
                ragdoll->DriveToPoseUsingKinematics(target, dt);
            }
            else
            {
                ragdoll->DriveToPoseUsingVelocities(target, dt);
            }
            m_scene->StartSimulation(dt);
            m_scene->FinishSimulation();
        }

        Physics::RagdollState state;
        ragdoll->GetState(state);
        return state[0].m_position.GetX();
    }

    TEST_F(JoltRagdollTests, HardKeyingDrivesThroughObstaclesButSoftKeyingIsStoppedByThem)
    {
        // Same target, same velocities: only the bodies' motion type differs.
        const float hardKeyedX = DriveIntoWallAndMeasureTravel(/*useHardKeying*/ true);

        // A kinematic body is not affected by the solver, so it reaches the target.
        EXPECT_NEAR(hardKeyedX, 4.0f, 0.2f);
    }

    TEST_F(JoltRagdollTests, SoftKeyingLetsThePhysicsOverruleTheAnimatedPose)
    {
        const float softKeyedX = DriveIntoWallAndMeasureTravel(/*useHardKeying*/ false);

        // The dynamic body is stopped at the wall (its near face is at x = 1.5, and the
        // node is a 0.15 m sphere) instead of reaching the target at x = 4.
        EXPECT_LT(softKeyedX, 1.5f);
        // ...but it did travel towards the target rather than just falling.
        EXPECT_GT(softKeyedX, 0.5f);
    }

    TEST_F(JoltRagdollTests, ReleaseToPhysicsLetsAHardKeyedRagdollFallAgain)
    {
        Physics::RagdollConfiguration config;
        MakeTwoNodeRagdoll(config, 5.0f);
        auto handle = m_scene->AddSimulatedBody(&config);
        auto* ragdoll = azdynamic_cast<JoltRagdoll*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(ragdoll, nullptr);
        ragdoll->EnableSimulation(config.m_initialState);

        // Hard keying holds the ragdoll in place against gravity.
        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 60; ++i)
        {
            ragdoll->DriveToPoseUsingKinematics(config.m_initialState, dt);
            m_scene->StartSimulation(dt);
            m_scene->FinishSimulation();
        }

        Physics::RagdollState heldState;
        ragdoll->GetState(heldState);
        EXPECT_NEAR(heldState[0].m_position.GetZ(), 5.0f, 0.05f);

        // Released, the bodies are dynamic again and fall.
        ragdoll->ReleaseToPhysics();
        SimulateSeconds(1.0f);

        Physics::RagdollState fallenState;
        ragdoll->GetState(fallenState);
        EXPECT_LT(fallenState[0].m_position.GetZ(), 4.0f);
    }

    TEST_F(JoltRagdollTests, DriveToPoseUsingMotorsBendsTheJointTowardsTheTarget)
    {
        Physics::RagdollConfiguration config;
        MakeTwoNodeRagdoll(config, 5.0f);
        AddSwingTwistJointToChild(config);

        auto handle = m_scene->AddSimulatedBody(&config);
        auto* ragdoll = azdynamic_cast<JoltRagdoll*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(ragdoll, nullptr);
        ragdoll->EnableSimulation(config.m_initialState);

        // Target pose: the child bent 30 degrees about x relative to the (unrotated) root.
        constexpr float targetDegrees = 30.0f;
        Physics::RagdollState target = config.m_initialState;
        target[1].m_orientation = AZ::Quaternion::CreateRotationX(AZ::DegToRad(targetDegrees));
        for (Physics::RagdollNodeState& nodeState : target)
        {
            nodeState.m_strength = 30.0f; // motor spring frequency, Hz
            nodeState.m_dampingRatio = 1.0f;
        }

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 120; ++i) // 2 seconds
        {
            ragdoll->DriveToPoseUsingMotors(target);
            m_scene->StartSimulation(dt);
            m_scene->FinishSimulation();
        }

        Physics::RagdollState state;
        ragdoll->GetState(state);
        ASSERT_EQ(state.size(), 2u);

        // The joint tracks the target angle...
        EXPECT_NEAR(GetChildAngleRelativeToRootDegrees(state), targetDegrees, 10.0f);
        // ...while the bodies stay dynamic, so the ragdoll still falls under gravity
        // (unlike the kinematic drive, motors do not hold it up).
        EXPECT_LT(state[0].m_position.GetZ(), 4.0f);
    }

    TEST_F(JoltRagdollTests, ZeroStrengthLeavesTheJointToPhysics)
    {
        Physics::RagdollConfiguration config;
        MakeTwoNodeRagdoll(config, 5.0f);
        AddSwingTwistJointToChild(config);

        auto handle = m_scene->AddSimulatedBody(&config);
        auto* ragdoll = azdynamic_cast<JoltRagdoll*>(m_scene->GetSimulatedBodyFromHandle(handle));
        ASSERT_NE(ragdoll, nullptr);
        ragdoll->EnableSimulation(config.m_initialState);

        // Same target, but strength 0: the motor is released and the joint is left limp,
        // so the ragdoll does not adopt the pose.
        Physics::RagdollState target = config.m_initialState;
        target[1].m_orientation = AZ::Quaternion::CreateRotationX(AZ::DegToRad(30.0f));
        for (Physics::RagdollNodeState& nodeState : target)
        {
            nodeState.m_strength = 0.0f;
        }

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 120; ++i)
        {
            ragdoll->DriveToPoseUsingMotors(target);
            m_scene->StartSimulation(dt);
            m_scene->FinishSimulation();
        }

        Physics::RagdollState state;
        ragdoll->GetState(state);
        EXPECT_LT(GetChildAngleRelativeToRootDegrees(state), 10.0f);
    }

} // namespace JoltPhysics
