#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <Configuration/JoltSettingsRegistryManager.h>
#include <Joint/JoltJoint.h>
#include <Joint/JoltJointConfiguration.h>
#include <RigidBody/JoltRigidBody.h>
#include <Scene/JoltScene.h>
#include <System/JoltSystem.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>

namespace JoltPhysics
{
    class JoltJointTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "JointTestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);
        }

        void TearDown() override
        {
            m_system->RemoveScene(m_sceneHandle);
            m_system->Shutdown();
            m_system.reset();
        }

        AzPhysics::SimulatedBodyHandle CreateStaticBox(const AZ::Vector3& position, const AZ::Vector3& dimensions)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            boxShape->m_dimensions = dimensions;

            AzPhysics::StaticRigidBodyConfiguration staticConfig;
            staticConfig.m_position = position;
            staticConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);
            return m_scene->AddSimulatedBody(&staticConfig);
        }

        AzPhysics::SimulatedBodyHandle CreateDynamicBox(
            const AZ::Vector3& position,
            const AZ::Vector3& dimensions = AZ::Vector3(0.5f, 0.5f, 0.5f),
            const AZ::Vector3& initialAngularVelocity = AZ::Vector3::CreateZero())
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            boxShape->m_dimensions = dimensions;

            AzPhysics::RigidBodyConfiguration boxConfig;
            boxConfig.m_position = position;
            boxConfig.m_initialAngularVelocity = initialAngularVelocity;
            boxConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);
            return m_scene->AddSimulatedBody(&boxConfig);
        }

        AzPhysics::RigidBody* GetBody(AzPhysics::SimulatedBodyHandle handle)
        {
            return azdynamic_cast<AzPhysics::RigidBody*>(m_scene->GetSimulatedBodyFromHandle(handle));
        }

        JPH::Constraint* GetConstraint(AzPhysics::JointHandle jointHandle)
        {
            if (AzPhysics::Joint* joint = m_scene->GetJointFromHandle(jointHandle))
            {
                return static_cast<JPH::Constraint*>(joint->GetNativePointer());
            }
            return nullptr;
        }

        void SimulateSteps(int steps)
        {
            const float fixedDeltaTime = 1.0f / 60.0f;
            for (int i = 0; i < steps; ++i)
            {
                m_scene->StartSimulation(fixedDeltaTime);
                m_scene->FinishSimulation();
            }
        }

        //! Sets joint frames so the pivot is at pivotWorld: parent/child local positions
        //! are the pivot relative to each body's position; both frames share frameRotation.
        static void SetFrames(
            AzPhysics::JointConfiguration& config,
            const AZ::Vector3& parentBodyPosition,
            const AZ::Vector3& childBodyPosition,
            const AZ::Vector3& pivotWorld,
            const AZ::Quaternion& frameRotation = AZ::Quaternion::CreateIdentity())
        {
            config.m_parentLocalPosition = pivotWorld - parentBodyPosition;
            config.m_parentLocalRotation = frameRotation;
            config.m_childLocalPosition = pivotWorld - childBodyPosition;
            config.m_childLocalRotation = frameRotation;
        }

        //! Angle of the arm (child - pivot) away from straight down, in degrees.
        static float AngleFromDownDegrees(const AZ::Vector3& pivotWorld, const AZ::Vector3& childPosition)
        {
            const AZ::Vector3 arm = childPosition - pivotWorld;
            const float horizontal = AZStd::sqrt(arm.GetX() * arm.GetX() + arm.GetY() * arm.GetY());
            return AZ::RadToDeg(AZStd::atan2(horizontal, -arm.GetZ()));
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle;
        AzPhysics::Scene* m_scene = nullptr;
    };

    TEST_F(JoltJointTests, FixedJointKeepsRelativeTransform)
    {
        auto anchorHandle = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        auto childHandle = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 3.0f));

        JoltFixedJointConfiguration config;
        SetFrames(config, AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.0f, 0.0f, 3.0f), AZ::Vector3(0.0f, 0.0f, 5.0f));
        auto jointHandle = m_scene->AddJoint(&config, anchorHandle, childHandle);
        ASSERT_NE(jointHandle, AzPhysics::InvalidJointHandle);

        SimulateSteps(120);

        // The child must stay welded 2 m below the anchor with no rotation.
        auto* child = GetBody(childHandle);
        ASSERT_NE(child, nullptr);
        EXPECT_NEAR(child->GetPosition().GetZ(), 3.0f, 0.02f);
        EXPECT_NEAR(child->GetPosition().GetX(), 0.0f, 0.02f);
        EXPECT_TRUE(child->GetOrientation().IsClose(AZ::Quaternion::CreateIdentity(), 0.01f));
    }

    TEST_F(JoltJointTests, HingeJointPendulumSwings)
    {
        auto anchorHandle = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        auto childHandle = CreateDynamicBox(AZ::Vector3(2.0f, 0.0f, 5.0f));

        // Hinge axis = world Y (frame X rotated +90 degrees about Z): pendulum swings in XZ.
        const AZ::Quaternion frameRotation = AZ::Quaternion::CreateFromAxisAngle(AZ::Vector3::CreateAxisZ(), AZ::DegToRad(90.0f));

        JoltHingeJointConfiguration config;
        config.m_limitProperties.m_isLimited = false;
        SetFrames(config, AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(2.0f, 0.0f, 5.0f), AZ::Vector3(0.0f, 0.0f, 5.0f), frameRotation);
        auto jointHandle = m_scene->AddJoint(&config, anchorHandle, childHandle);
        ASSERT_NE(jointHandle, AzPhysics::InvalidJointHandle);

        SimulateSteps(60);
        // Released horizontal, the pendulum must have swung down.
        auto* child = GetBody(childHandle);
        ASSERT_NE(child, nullptr);
        const float angleAfter60 = AngleFromDownDegrees(AZ::Vector3(0.0f, 0.0f, 5.0f), child->GetPosition());
        EXPECT_LT(angleAfter60, 60.0f);

        SimulateSteps(240);
        // Arm length is preserved and the pendulum keeps swinging near the bottom.
        const float armLength = (child->GetPosition() - AZ::Vector3(0.0f, 0.0f, 5.0f)).GetLength();
        EXPECT_NEAR(armLength, 2.0f, 0.1f);
        EXPECT_LT(child->GetPosition().GetZ(), 5.0f);
    }

    TEST_F(JoltJointTests, HingeJointLimitsHold)
    {
        auto anchorHandle = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        auto childHandle = CreateDynamicBox(AZ::Vector3(2.0f, 0.0f, 5.0f));

        const AZ::Quaternion frameRotation = AZ::Quaternion::CreateFromAxisAngle(AZ::Vector3::CreateAxisZ(), AZ::DegToRad(90.0f));

        JoltHingeJointConfiguration config;
        config.m_limitProperties.m_isLimited = true;
        config.m_limitProperties.m_limitFirst = -10.0f; // lower limit (degrees)
        config.m_limitProperties.m_limitSecond = 95.0f; // upper limit (degrees)
        SetFrames(config, AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(2.0f, 0.0f, 5.0f), AZ::Vector3(0.0f, 0.0f, 5.0f), frameRotation);
        auto jointHandle = m_scene->AddJoint(&config, anchorHandle, childHandle);
        ASSERT_NE(jointHandle, AzPhysics::InvalidJointHandle);

        auto* child = GetBody(childHandle);
        ASSERT_NE(child, nullptr);

        // Track the maximum excursion over several seconds of swinging.
        float maxAngle = 0.0f;
        const float fixedDeltaTime = 1.0f / 60.0f;
        for (int i = 0; i < 300; ++i)
        {
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();
            maxAngle = AZStd::max(maxAngle, AngleFromDownDegrees(AZ::Vector3(0.0f, 0.0f, 5.0f), child->GetPosition()));
        }
        // The release angle is 90 degrees; the 95 degree limit must hold (with slop).
        EXPECT_LT(maxAngle, 105.0f);
    }

    TEST_F(JoltJointTests, HingeJointMotorDrivesRotation)
    {
        auto anchorHandle = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        auto childHandle = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 3.0f)); // hanging straight down

        const AZ::Quaternion frameRotation = AZ::Quaternion::CreateFromAxisAngle(AZ::Vector3::CreateAxisZ(), AZ::DegToRad(90.0f));

        JoltHingeJointConfiguration config;
        config.m_limitProperties.m_isLimited = false;
        config.m_motorProperties.m_useMotor = true;
        config.m_motorProperties.m_driveForceLimit = 1000.0f;
        SetFrames(config, AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.0f, 0.0f, 3.0f), AZ::Vector3(0.0f, 0.0f, 5.0f), frameRotation);
        auto jointHandle = m_scene->AddJoint(&config, anchorHandle, childHandle);
        ASSERT_NE(jointHandle, AzPhysics::InvalidJointHandle);

        auto* hinge = static_cast<JPH::HingeConstraint*>(GetConstraint(jointHandle));
        ASSERT_NE(hinge, nullptr);
        hinge->SetMotorState(JPH::EMotorState::Velocity);
        hinge->SetTargetAngularVelocity(3.0f);

        auto* child = GetBody(childHandle);
        ASSERT_NE(child, nullptr);

        SimulateSteps(60);
        // The motor must have lifted the pendulum away from straight down.
        const float angle = AngleFromDownDegrees(AZ::Vector3(0.0f, 0.0f, 5.0f), child->GetPosition());
        EXPECT_GT(angle, 30.0f);
    }

    TEST_F(JoltJointTests, BallJointChainStaysConnected)
    {
        auto anchorHandle = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 6.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        auto link1Handle = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 5.0f));
        auto link2Handle = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 4.0f));

        JoltBallJointConfiguration joint1Config;
        joint1Config.m_limitProperties.m_isLimited = true;
        joint1Config.m_limitProperties.m_limitFirst = 60.0f; // swing cone half-angle (deg)
        joint1Config.m_limitProperties.m_limitSecond = 60.0f;
        SetFrames(joint1Config, AZ::Vector3(0.0f, 0.0f, 6.0f), AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.0f, 0.0f, 5.5f));
        ASSERT_NE(m_scene->AddJoint(&joint1Config, anchorHandle, link1Handle), AzPhysics::InvalidJointHandle);

        JoltBallJointConfiguration joint2Config;
        joint2Config.m_limitProperties.m_isLimited = true;
        joint2Config.m_limitProperties.m_limitFirst = 60.0f;
        joint2Config.m_limitProperties.m_limitSecond = 60.0f;
        SetFrames(joint2Config, AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.0f, 0.0f, 4.0f), AZ::Vector3(0.0f, 0.0f, 4.5f));
        ASSERT_NE(m_scene->AddJoint(&joint2Config, link1Handle, link2Handle), AzPhysics::InvalidJointHandle);

        // Knock the chain sideways and let it settle.
        if (auto* link1 = GetBody(link1Handle))
        {
            link1->SetLinearVelocity(AZ::Vector3(2.0f, 0.0f, 0.0f));
        }
        SimulateSteps(300);

        auto* link1 = GetBody(link1Handle);
        auto* link2 = GetBody(link2Handle);
        ASSERT_NE(link1, nullptr);
        ASSERT_NE(link2, nullptr);

        // Chain links stay near their anchors (1 m reach each) and dangle downwards.
        EXPECT_LT((link1->GetPosition() - AZ::Vector3(0.0f, 0.0f, 5.5f)).GetLength(), 1.2f);
        EXPECT_LT((link2->GetPosition() - link1->GetPosition()).GetLength(), 1.2f);
        EXPECT_LT(link1->GetPosition().GetZ(), 5.5f);
        EXPECT_LT(link2->GetPosition().GetZ(), link1->GetPosition().GetZ() + 0.5f);
    }

    TEST_F(JoltJointTests, PrismaticJointSlidesWithMotorToLimit)
    {
        auto frameHandle = CreateStaticBox(AZ::Vector3(0.0f, 10.0f, 2.0f), AZ::Vector3(2.0f, 2.0f, 0.2f));
        auto doorHandle = CreateDynamicBox(AZ::Vector3(0.0f, 10.0f, 2.0f), AZ::Vector3(0.5f, 1.0f, 1.0f));

        JoltPrismaticJointConfiguration config;
        config.m_limitProperties.m_isLimited = true;
        config.m_limitProperties.m_limitFirst = 0.0f; // min slide (meters)
        config.m_limitProperties.m_limitSecond = 1.5f; // max slide (meters)
        config.m_motorProperties.m_useMotor = true;
        config.m_motorProperties.m_driveForceLimit = 1000.0f;
        SetFrames(config, AZ::Vector3(0.0f, 10.0f, 2.0f), AZ::Vector3(0.0f, 10.0f, 2.0f), AZ::Vector3(0.0f, 10.0f, 2.0f));
        auto jointHandle = m_scene->AddJoint(&config, frameHandle, doorHandle);
        ASSERT_NE(jointHandle, AzPhysics::InvalidJointHandle);

        auto* slider = static_cast<JPH::SliderConstraint*>(GetConstraint(jointHandle));
        ASSERT_NE(slider, nullptr);
        slider->SetMotorState(JPH::EMotorState::Velocity);
        slider->SetTargetVelocity(0.5f);

        auto* door = GetBody(doorHandle);
        ASSERT_NE(door, nullptr);

        SimulateSteps(300);

        // The door slides open to the 1.5 m limit, does not rotate and does not fall.
        EXPECT_NEAR(door->GetPosition().GetX(), 1.5f, 0.15f);
        EXPECT_NEAR(door->GetPosition().GetZ(), 2.0f, 0.05f);
        EXPECT_TRUE(door->GetOrientation().IsClose(AZ::Quaternion::CreateIdentity(), 0.01f));
    }

    TEST_F(JoltJointTests, D6JointAngularLimitsHold)
    {
        auto anchorHandle = CreateStaticBox(AZ::Vector3(0.0f, 20.0f, 5.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        auto childHandle = CreateDynamicBox(
            AZ::Vector3(0.0f, 20.0f, 4.0f),
            AZ::Vector3(0.5f, 0.5f, 0.5f),
            AZ::Vector3(3.0f, 2.0f, 1.0f) // initial spin to perturb the joint
        );

        JoltD6JointLimitConfiguration config;
        config.m_swingLimitY = 20.0f;
        config.m_swingLimitZ = 20.0f;
        config.m_twistLimitLower = -15.0f;
        config.m_twistLimitUpper = 15.0f;
        SetFrames(config, AZ::Vector3(0.0f, 20.0f, 5.0f), AZ::Vector3(0.0f, 20.0f, 4.0f), AZ::Vector3(0.0f, 20.0f, 5.0f));
        auto jointHandle = m_scene->AddJoint(&config, anchorHandle, childHandle);
        ASSERT_NE(jointHandle, AzPhysics::InvalidJointHandle);

        auto* child = GetBody(childHandle);
        ASSERT_NE(child, nullptr);

        SimulateSteps(180);

        // Linear DOF is locked: the child hangs exactly 1 m below the anchor.
        EXPECT_NEAR((child->GetPosition() - AZ::Vector3(0.0f, 20.0f, 5.0f)).GetLength(), 1.0f, 0.05f);

        // Angular limits hold: the child's up axis stays within ~35 degrees of world up
        // (20 degree swing limit plus transient slop).
        const AZ::Vector3 childUp = child->GetOrientation().TransformVector(AZ::Vector3::CreateAxisZ());
        EXPECT_GT(childUp.GetZ(), AZStd::cos(AZ::DegToRad(35.0f)));
    }

    TEST_F(JoltJointTests, JointsStableForSixtySeconds)
    {
        auto anchorHandle = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        auto pendulumHandle = CreateDynamicBox(AZ::Vector3(2.0f, 0.0f, 5.0f));
        auto link1Handle = CreateDynamicBox(AZ::Vector3(0.0f, 5.0f, 5.0f));
        auto link2Handle = CreateDynamicBox(AZ::Vector3(0.0f, 5.0f, 4.0f));

        const AZ::Quaternion frameRotation = AZ::Quaternion::CreateFromAxisAngle(AZ::Vector3::CreateAxisZ(), AZ::DegToRad(90.0f));

        JoltHingeJointConfiguration hingeConfig;
        hingeConfig.m_limitProperties.m_isLimited = true;
        hingeConfig.m_limitProperties.m_limitFirst = -95.0f;
        hingeConfig.m_limitProperties.m_limitSecond = 95.0f;
        SetFrames(hingeConfig, AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(2.0f, 0.0f, 5.0f), AZ::Vector3(0.0f, 0.0f, 5.0f), frameRotation);
        ASSERT_NE(m_scene->AddJoint(&hingeConfig, anchorHandle, pendulumHandle), AzPhysics::InvalidJointHandle);

        JoltBallJointConfiguration ballConfig1;
        ballConfig1.m_limitProperties.m_limitFirst = 60.0f;
        ballConfig1.m_limitProperties.m_limitSecond = 60.0f;
        SetFrames(ballConfig1, AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.0f, 5.0f, 5.0f), AZ::Vector3(0.0f, 5.0f, 5.5f));
        ASSERT_NE(m_scene->AddJoint(&ballConfig1, anchorHandle, link1Handle), AzPhysics::InvalidJointHandle);

        JoltBallJointConfiguration ballConfig2;
        ballConfig2.m_limitProperties.m_limitFirst = 60.0f;
        ballConfig2.m_limitProperties.m_limitSecond = 60.0f;
        SetFrames(ballConfig2, AZ::Vector3(0.0f, 5.0f, 5.0f), AZ::Vector3(0.0f, 5.0f, 4.0f), AZ::Vector3(0.0f, 5.0f, 4.5f));
        ASSERT_NE(m_scene->AddJoint(&ballConfig2, link1Handle, link2Handle), AzPhysics::InvalidJointHandle);

        SimulateSteps(3600); // 60 simulated seconds

        auto* pendulum = GetBody(pendulumHandle);
        auto* link1 = GetBody(link1Handle);
        auto* link2 = GetBody(link2Handle);
        ASSERT_NE(pendulum, nullptr);
        ASSERT_NE(link1, nullptr);
        ASSERT_NE(link2, nullptr);

        // Everything is still finite and near its anchor — no explosion or NaN.
        const AZ::Vector3 pendulumPosition = pendulum->GetPosition();
        EXPECT_TRUE(pendulumPosition.IsFinite());
        EXPECT_NEAR((pendulumPosition - AZ::Vector3(0.0f, 0.0f, 5.0f)).GetLength(), 2.0f, 0.2f);

        EXPECT_TRUE(link1->GetPosition().IsFinite());
        EXPECT_LT((link1->GetPosition() - AZ::Vector3(0.0f, 5.0f, 5.5f)).GetLength(), 1.2f);
        EXPECT_TRUE(link2->GetPosition().IsFinite());
        EXPECT_LT((link2->GetPosition() - link1->GetPosition()).GetLength(), 1.2f);
    }

} // namespace JoltPhysics
