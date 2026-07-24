#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <Character/JoltRagdoll.h>
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
