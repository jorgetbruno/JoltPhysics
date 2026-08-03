#include <JoltPhysics/JoltPhysicsBus.h>
#include <Clients/Components/JoltJointComponents.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <Configuration/JoltSettingsRegistryManager.h>
#include <Joint/JoltJoint.h>
#include <Joint/JoltJointConfiguration.h>
#include <RigidBody/JoltRigidBody.h>
#include <Scene/JoltScene.h>
#include <System/JoltSystem.h>

#include "JoltTestWarningCatcher.h"

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
            // An authored mass, not a computed one: these thresholds are calibrated
            // against the body's weight, so the geometry must not decide it.
            boxConfig.m_computeMass = false;
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

        //! A dynamic box with gravity switched off, so a test measuring a constraint's
        //! coupling is not also measuring the body falling.
        AzPhysics::SimulatedBodyHandle CreateWeightlessBox(
            const AZ::Vector3& position, const AZ::Vector3& initialAngularVelocity = AZ::Vector3::CreateZero())
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            boxShape->m_dimensions = AZ::Vector3(0.5f, 0.5f, 0.5f);

            AzPhysics::RigidBodyConfiguration boxConfig;
            boxConfig.m_position = position;
            boxConfig.m_initialAngularVelocity = initialAngularVelocity;
            boxConfig.m_gravityEnabled = false;
            // An authored mass, not a computed one: these thresholds are calibrated
            // against the body's weight, so the geometry must not decide it.
            boxConfig.m_computeMass = false;
            boxConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);
            return m_scene->AddSimulatedBody(&boxConfig);
        }

        //! Hinges a body to a static anchor about the world X axis, pivoting in place.
        AzPhysics::JointHandle HingeInPlaceAboutX(
            AzPhysics::SimulatedBodyHandle anchor,
            const AZ::Vector3& anchorPosition,
            AzPhysics::SimulatedBodyHandle body,
            const AZ::Vector3& bodyPosition)
        {
            JoltHingeJointConfiguration hinge;
            hinge.m_limitProperties.m_isLimited = false;
            SetFrames(hinge, anchorPosition, bodyPosition, bodyPosition);
            return m_scene->AddJoint(&hinge, anchor, body);
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

    TEST_F(JoltJointTests, GearJointCouplesRotationAtTheToothRatio)
    {
        const AZ::Vector3 anchorPosition(0.0f, 0.0f, 0.0f);
        const AZ::Vector3 driverPosition(-1.0f, 0.0f, 0.0f);
        const AZ::Vector3 drivenPosition(1.0f, 0.0f, 0.0f);

        auto anchor = CreateStaticBox(anchorPosition, AZ::Vector3(0.2f, 0.2f, 0.2f));
        auto driver = CreateWeightlessBox(driverPosition, AZ::Vector3(10.0f, 0.0f, 0.0f));
        auto driven = CreateWeightlessBox(drivenPosition);

        // The gear only couples rotation; without these hinges holding each body on its
        // axis the constraint has nothing meaningful to couple.
        auto driverHinge = HingeInPlaceAboutX(anchor, anchorPosition, driver, driverPosition);
        auto drivenHinge = HingeInPlaceAboutX(anchor, anchorPosition, driven, drivenPosition);
        ASSERT_NE(driverHinge, AzPhysics::InvalidJointHandle);
        ASSERT_NE(drivenHinge, AzPhysics::InvalidJointHandle);

        JoltGearJointConfiguration gear;
        gear.m_parentNumTeeth = 1;
        gear.m_childNumTeeth = 2;
        gear.m_parentHingeJoint = driverHinge;
        gear.m_childHingeJoint = drivenHinge;
        SetFrames(gear, driverPosition, drivenPosition, AZ::Vector3::CreateZero());
        ASSERT_NE(m_scene->AddJoint(&gear, driver, driven), AzPhysics::InvalidJointHandle);

        SimulateSteps(30);

        // Jolt's ratio is driverRotation = -(childTeeth / parentTeeth) * drivenRotation, so a
        // 1-tooth driver turns a 2-tooth wheel at half speed, the opposite way.
        const float driverSpin = GetBody(driver)->GetAngularVelocity().GetX();
        const float drivenSpin = GetBody(driven)->GetAngularVelocity().GetX();
        EXPECT_GT(driverSpin, 1.0f);
        EXPECT_NEAR(drivenSpin, -driverSpin * 0.5f, AZStd::abs(driverSpin) * 0.1f);
    }

    TEST_F(JoltJointTests, GearJointToothRatioChangesTheCouplingSpeed)
    {
        // Same rig at 1:1, to show the ratio is doing the work rather than the constraint
        // simply mirroring whatever the driver does.
        const AZ::Vector3 anchorPosition(0.0f, 0.0f, 0.0f);
        const AZ::Vector3 driverPosition(-1.0f, 0.0f, 0.0f);
        const AZ::Vector3 drivenPosition(1.0f, 0.0f, 0.0f);

        auto anchor = CreateStaticBox(anchorPosition, AZ::Vector3(0.2f, 0.2f, 0.2f));
        auto driver = CreateWeightlessBox(driverPosition, AZ::Vector3(10.0f, 0.0f, 0.0f));
        auto driven = CreateWeightlessBox(drivenPosition);

        HingeInPlaceAboutX(anchor, anchorPosition, driver, driverPosition);
        HingeInPlaceAboutX(anchor, anchorPosition, driven, drivenPosition);

        JoltGearJointConfiguration gear;
        gear.m_parentNumTeeth = 1;
        gear.m_childNumTeeth = 1;
        SetFrames(gear, driverPosition, drivenPosition, AZ::Vector3::CreateZero());
        ASSERT_NE(m_scene->AddJoint(&gear, driver, driven), AzPhysics::InvalidJointHandle);

        SimulateSteps(30);

        const float driverSpin = GetBody(driver)->GetAngularVelocity().GetX();
        const float drivenSpin = GetBody(driven)->GetAngularVelocity().GetX();
        EXPECT_GT(driverSpin, 1.0f);
        EXPECT_NEAR(drivenSpin, -driverSpin, AZStd::abs(driverSpin) * 0.1f);
    }

    TEST_F(JoltJointTests, GearJointWithZeroTeethIsRejected)
    {
        auto driver = CreateWeightlessBox(AZ::Vector3(-1.0f, 0.0f, 0.0f));
        auto driven = CreateWeightlessBox(AZ::Vector3(1.0f, 0.0f, 0.0f));

        JoltGearJointConfiguration gear;
        gear.m_parentNumTeeth = 0;
        gear.m_debugName = "ZeroTeethGear";

        JoltWarningCatcher warnings;
        EXPECT_EQ(m_scene->AddJoint(&gear, driver, driven), AzPhysics::InvalidJointHandle);
        EXPECT_TRUE(warnings.ContainsWarningWith("ZeroTeethGear"));
    }

    TEST_F(JoltJointTests, RackAndPinionJointTurnsRotationIntoTranslation)
    {
        const AZ::Vector3 anchorPosition(0.0f, -2.0f, 0.0f);
        const AZ::Vector3 pinionPosition(0.0f, 0.0f, 0.0f);
        const AZ::Vector3 rackPosition(0.0f, 2.0f, 0.0f);

        auto anchor = CreateStaticBox(anchorPosition, AZ::Vector3(0.2f, 0.2f, 0.2f));
        auto pinion = CreateWeightlessBox(pinionPosition, AZ::Vector3(10.0f, 0.0f, 0.0f));
        auto rack = CreateWeightlessBox(rackPosition);

        // Pinion spins about X; the rack slides along X on a prismatic joint.
        auto pinionHinge = HingeInPlaceAboutX(anchor, anchorPosition, pinion, pinionPosition);
        ASSERT_NE(pinionHinge, AzPhysics::InvalidJointHandle);

        JoltPrismaticJointConfiguration slider;
        slider.m_limitProperties.m_isLimited = false;
        SetFrames(slider, anchorPosition, rackPosition, rackPosition);
        auto rackSlider = m_scene->AddJoint(&slider, anchor, rack);
        ASSERT_NE(rackSlider, AzPhysics::InvalidJointHandle);

        JoltRackAndPinionJointConfiguration rackAndPinion;
        rackAndPinion.m_pinionNumTeeth = 8;
        rackAndPinion.m_rackNumTeeth = 8;
        rackAndPinion.m_rackLength = 1.0f;
        rackAndPinion.m_pinionHingeJoint = pinionHinge;
        rackAndPinion.m_rackSliderJoint = rackSlider;
        SetFrames(rackAndPinion, pinionPosition, rackPosition, AZ::Vector3::CreateZero());
        ASSERT_NE(m_scene->AddJoint(&rackAndPinion, pinion, rack), AzPhysics::InvalidJointHandle);

        SimulateSteps(30);

        // A spinning pinion has to drag the rack along its slider axis. Without the
        // constraint the rack has no reason to move at all - see the control below.
        EXPECT_GT(AZStd::abs(GetBody(rack)->GetLinearVelocity().GetX()), 0.5f);
        EXPECT_GT(AZStd::abs(GetBody(rack)->GetPosition().GetX()), 0.05f);
    }

    TEST_F(JoltJointTests, RackOnItsOwnSliderDoesNotMoveWithoutTheRackAndPinionJoint)
    {
        // The control for the test above: same rig, no rack-and-pinion joint.
        const AZ::Vector3 anchorPosition(0.0f, -2.0f, 0.0f);
        const AZ::Vector3 rackPosition(0.0f, 2.0f, 0.0f);

        auto anchor = CreateStaticBox(anchorPosition, AZ::Vector3(0.2f, 0.2f, 0.2f));
        auto pinion = CreateWeightlessBox(AZ::Vector3::CreateZero(), AZ::Vector3(10.0f, 0.0f, 0.0f));
        auto rack = CreateWeightlessBox(rackPosition);

        HingeInPlaceAboutX(anchor, anchorPosition, pinion, AZ::Vector3::CreateZero());

        JoltPrismaticJointConfiguration slider;
        slider.m_limitProperties.m_isLimited = false;
        SetFrames(slider, anchorPosition, rackPosition, rackPosition);
        ASSERT_NE(m_scene->AddJoint(&slider, anchor, rack), AzPhysics::InvalidJointHandle);

        SimulateSteps(30);

        EXPECT_NEAR(GetBody(rack)->GetPosition().GetX(), 0.0f, 0.01f);
    }

    TEST_F(JoltJointTests, DistanceJointCapsSeparation)
    {
        auto anchorHandle = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        auto childHandle = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 3.0f));

        JoltDistanceJointConfiguration config;
        config.m_parentLocalPosition = AZ::Vector3::CreateZero(); // attach at the anchor centre
        config.m_childLocalPosition = AZ::Vector3::CreateZero();  // attach at the child centre
        config.m_minDistance = 0.0f;
        config.m_maxDistance = 2.0f;
        auto jointHandle = m_scene->AddJoint(&config, anchorHandle, childHandle);
        ASSERT_NE(jointHandle, AzPhysics::InvalidJointHandle);

        SimulateSteps(180);

        auto* child = GetBody(childHandle);
        ASSERT_NE(child, nullptr);
        const float distance = child->GetPosition().GetDistance(AZ::Vector3(0.0f, 0.0f, 5.0f));
        EXPECT_LE(distance, 2.05f);           // never separates past the max distance
        EXPECT_NEAR(distance, 2.0f, 0.1f);    // gravity pulls the tether taut
    }

    TEST_F(JoltJointTests, JointedBodiesDoNotCollideWithEachOther)
    {
        // Two weightless boxes spawned 0.1 m interpenetrating, loosely tethered: PhysX parity
        // means the joint suppresses their contact response, so nothing pushes them
        // apart. (A distance joint is used on purpose - a rigid joint would hold the
        // pair together even without the collision filter and make this test vacuous.)
        auto boxA = CreateWeightlessBox(AZ::Vector3(-0.2f, 0.0f, 0.0f));
        auto boxB = CreateWeightlessBox(AZ::Vector3(0.2f, 0.0f, 0.0f));

        JoltDistanceJointConfiguration config;
        config.m_parentLocalPosition = AZ::Vector3::CreateZero();
        config.m_childLocalPosition = AZ::Vector3::CreateZero();
        config.m_minDistance = 0.0f;
        config.m_maxDistance = 10.0f; // loose: free to separate, but nothing should push
        const AzPhysics::JointHandle jointHandle = m_scene->AddJoint(&config, boxA, boxB);
        ASSERT_NE(jointHandle, AzPhysics::InvalidJointHandle);

        SimulateSteps(120);

        // The overlap never resolves: the centers stay ~0.4 m apart instead of
        // separating past the 1.0 m the two half-meter boxes need to stop touching.
        const float distance = (GetBody(boxB)->GetPosition() - GetBody(boxA)->GetPosition()).GetLength();
        EXPECT_NEAR(distance, 0.4f, 0.1f);
    }

    TEST_F(JoltJointTests, OverlappingUnjointedBodiesStillPushApart)
    {
        // Same spawn without the joint: contact response separates them. Keeps the
        // jointed test above honest (proves contacts would otherwise generate).
        auto boxA = CreateWeightlessBox(AZ::Vector3(-0.2f, 0.0f, 0.0f));
        auto boxB = CreateWeightlessBox(AZ::Vector3(0.2f, 0.0f, 0.0f));

        SimulateSteps(120);

        const float distance = (GetBody(boxB)->GetPosition() - GetBody(boxA)->GetPosition()).GetLength();
        EXPECT_GT(distance, 0.45f);
    }

    TEST_F(JoltJointTests, CollisionResumesAfterJointRemoved)
    {
        auto boxA = CreateWeightlessBox(AZ::Vector3(-0.2f, 0.0f, 0.0f));
        auto boxB = CreateWeightlessBox(AZ::Vector3(0.2f, 0.0f, 0.0f));

        JoltDistanceJointConfiguration config;
        config.m_parentLocalPosition = AZ::Vector3::CreateZero();
        config.m_childLocalPosition = AZ::Vector3::CreateZero();
        config.m_minDistance = 0.0f;
        config.m_maxDistance = 10.0f;
        const AzPhysics::JointHandle jointHandle = m_scene->AddJoint(&config, boxA, boxB);
        ASSERT_NE(jointHandle, AzPhysics::InvalidJointHandle);
        m_scene->RemoveJoint(jointHandle);

        SimulateSteps(120);

        // With the joint gone the filter is gone too, and the overlap resolves.
        const float distance = (GetBody(boxB)->GetPosition() - GetBody(boxA)->GetPosition()).GetLength();
        EXPECT_GT(distance, 0.45f);
    }

    TEST_F(JoltJointTests, ThirdBodyStillCollidesWithJointedPair)
    {
        // The filter is pair-specific: a falling third body shares no immunity and
        // lands on a jointed box as normal. The stack is static so the outcome is
        // deterministic; the joint registers the pair all the same.
        auto floorHandle = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(20.0f, 20.0f, 1.0f));
        auto boxA = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 0.25f), AZ::Vector3(0.5f, 0.5f, 0.5f));

        JoltDistanceJointConfiguration config;
        config.m_parentLocalPosition = AZ::Vector3::CreateZero();
        config.m_childLocalPosition = AZ::Vector3::CreateZero();
        config.m_minDistance = 0.0f;
        config.m_maxDistance = 10.0f;
        ASSERT_NE(m_scene->AddJoint(&config, floorHandle, boxA), AzPhysics::InvalidJointHandle);

        auto falling = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 3.0f));
        SimulateSteps(240);

        // Floor top z=0, the static box's center is at z=0.25, so the falling box's
        // center lands at z=0.75 instead of passing through the jointed stack.
        EXPECT_NEAR(GetBody(falling)->GetPosition().GetZ(), 0.75f, 0.15f);
    }

    TEST_F(JoltJointTests, ConeJointPinsBodyAtPivot)
    {
        auto anchorHandle = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        auto childHandle = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 3.0f));

        JoltConeJointConfiguration config;
        SetFrames(config, AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.0f, 0.0f, 3.0f), AZ::Vector3(0.0f, 0.0f, 4.0f));
        config.m_halfConeAngle = 45.0f;
        auto jointHandle = m_scene->AddJoint(&config, anchorHandle, childHandle);
        ASSERT_NE(jointHandle, AzPhysics::InvalidJointHandle);

        SimulateSteps(180);

        auto* child = GetBody(childHandle);
        ASSERT_NE(child, nullptr);
        // The cone joint locks position: the child centre stays ~1 m from the pivot at (0,0,4).
        const float radius = child->GetPosition().GetDistance(AZ::Vector3(0.0f, 0.0f, 4.0f));
        EXPECT_NEAR(radius, 1.0f, 0.15f);
        EXPECT_LT(child->GetPosition().GetZ(), 3.1f); // hangs below the pivot
    }

    TEST_F(JoltJointTests, SwingTwistJointPinsBodyAtPivot)
    {
        auto anchorHandle = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        auto childHandle = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 3.0f));

        JoltSwingTwistJointConfiguration config;
        SetFrames(config, AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.0f, 0.0f, 3.0f), AZ::Vector3(0.0f, 0.0f, 4.0f));
        config.m_normalHalfConeAngle = 45.0f;
        config.m_planeHalfConeAngle = 45.0f;
        config.m_twistLower = -45.0f;
        config.m_twistUpper = 45.0f;
        auto jointHandle = m_scene->AddJoint(&config, anchorHandle, childHandle);
        ASSERT_NE(jointHandle, AzPhysics::InvalidJointHandle);

        SimulateSteps(180);

        auto* child = GetBody(childHandle);
        ASSERT_NE(child, nullptr);
        // Swing-twist also locks position: the child centre stays ~1 m from the pivot.
        const float radius = child->GetPosition().GetDistance(AZ::Vector3(0.0f, 0.0f, 4.0f));
        EXPECT_NEAR(radius, 1.0f, 0.15f);
        EXPECT_TRUE(child->GetPosition().IsFinite());
    }

    TEST_F(JoltJointTests, HingeSoftLimitBecomesALimitSpring)
    {
        auto anchorHandle = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        auto childHandle = CreateDynamicBox(AZ::Vector3(2.0f, 0.0f, 5.0f));

        JoltHingeJointConfiguration config;
        config.m_limitProperties.m_isLimited = true;
        config.m_limitProperties.m_isSoftLimit = true;
        config.m_limitProperties.m_limitFirst = -45.0f;
        config.m_limitProperties.m_limitSecond = 45.0f;
        config.m_limitProperties.m_stiffness = 77.0f;
        config.m_limitProperties.m_damping = 7.0f;
        SetFrames(config, AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(2.0f, 0.0f, 5.0f), AZ::Vector3(0.0f, 0.0f, 5.0f));
        auto jointHandle = m_scene->AddJoint(&config, anchorHandle, childHandle);
        ASSERT_NE(jointHandle, AzPhysics::InvalidJointHandle);

        // Stiffness/damping pass through unchanged: PhysX's soft-limit fields are the
        // k and c of Jolt's StiffnessAndDamping spring mode.
        auto* hinge = static_cast<JPH::HingeConstraint*>(GetConstraint(jointHandle));
        ASSERT_NE(hinge, nullptr);
        const JPH::SpringSettings& spring = hinge->GetLimitsSpringSettings();
        EXPECT_EQ(JPH::ESpringMode::StiffnessAndDamping, spring.mMode);
        EXPECT_FLOAT_EQ(77.0f, spring.mStiffness);
        EXPECT_FLOAT_EQ(7.0f, spring.mDamping);
    }

    TEST_F(JoltJointTests, HingeHardLimitLeavesTheLimitSpringInactive)
    {
        auto anchorHandle = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        auto childHandle = CreateDynamicBox(AZ::Vector3(2.0f, 0.0f, 5.0f));

        JoltHingeJointConfiguration config;
        config.m_limitProperties.m_isLimited = true;
        config.m_limitProperties.m_isSoftLimit = false;
        config.m_limitProperties.m_limitFirst = -45.0f;
        config.m_limitProperties.m_limitSecond = 45.0f;
        SetFrames(config, AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(2.0f, 0.0f, 5.0f), AZ::Vector3(0.0f, 0.0f, 5.0f));
        auto jointHandle = m_scene->AddJoint(&config, anchorHandle, childHandle);
        ASSERT_NE(jointHandle, AzPhysics::InvalidJointHandle);

        auto* hinge = static_cast<JPH::HingeConstraint*>(GetConstraint(jointHandle));
        ASSERT_NE(hinge, nullptr);
        EXPECT_FALSE(hinge->GetLimitsSpringSettings().HasStiffnessOrDamping());
    }

    TEST_F(JoltJointTests, PrismaticSoftLimitBecomesALimitSpring)
    {
        auto anchorHandle = CreateStaticBox(AZ::Vector3(0.0f, 10.0f, 5.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        auto childHandle = CreateDynamicBox(AZ::Vector3(2.0f, 10.0f, 5.0f));

        JoltPrismaticJointConfiguration config;
        config.m_limitProperties.m_isLimited = true;
        config.m_limitProperties.m_isSoftLimit = true;
        config.m_limitProperties.m_limitFirst = -0.5f;
        config.m_limitProperties.m_limitSecond = 0.5f;
        config.m_limitProperties.m_stiffness = 33.0f;
        config.m_limitProperties.m_damping = 3.0f;
        SetFrames(config, AZ::Vector3(0.0f, 10.0f, 5.0f), AZ::Vector3(2.0f, 10.0f, 5.0f), AZ::Vector3(2.0f, 10.0f, 5.0f));
        auto jointHandle = m_scene->AddJoint(&config, anchorHandle, childHandle);
        ASSERT_NE(jointHandle, AzPhysics::InvalidJointHandle);

        auto* slider = static_cast<JPH::SliderConstraint*>(GetConstraint(jointHandle));
        ASSERT_NE(slider, nullptr);
        const JPH::SpringSettings& spring = slider->GetLimitsSpringSettings();
        EXPECT_EQ(JPH::ESpringMode::StiffnessAndDamping, spring.mMode);
        EXPECT_FLOAT_EQ(33.0f, spring.mStiffness);
        EXPECT_FLOAT_EQ(3.0f, spring.mDamping);
    }

    TEST_F(JoltJointTests, PrismaticSoftLimitYieldsWhereAHardLimitStops)
    {
        // Two identical weightless boxes launched at 2 m/s along their slider, limits at
        // 0.5 m. The hard limit stops its box dead; the soft one (a weak 1 N/m spring
        // against a 1 kg box) must let its box glide well past before pulling it back.
        auto MakeSliderRig = [this](float y, bool softLimit, AzPhysics::SimulatedBodyHandle& outBox)
        {
            const AZ::Vector3 anchorPosition(0.0f, y, 5.0f);
            const AZ::Vector3 boxPosition(2.0f, y, 5.0f);
            auto anchor = CreateStaticBox(anchorPosition, AZ::Vector3(0.5f, 0.5f, 0.5f));
            outBox = CreateWeightlessBox(boxPosition);

            JoltPrismaticJointConfiguration config;
            config.m_limitProperties.m_isLimited = true;
            config.m_limitProperties.m_isSoftLimit = softLimit;
            config.m_limitProperties.m_limitFirst = -0.5f;
            config.m_limitProperties.m_limitSecond = 0.5f;
            config.m_limitProperties.m_stiffness = 1.0f;
            config.m_limitProperties.m_damping = 0.2f;
            SetFrames(config, anchorPosition, boxPosition, boxPosition);
            ASSERT_NE(m_scene->AddJoint(&config, anchor, outBox), AzPhysics::InvalidJointHandle);

            GetBody(outBox)->SetLinearVelocity(AZ::Vector3(2.0f, 0.0f, 0.0f));
        };

        AzPhysics::SimulatedBodyHandle hardBox = AzPhysics::InvalidSimulatedBodyHandle;
        AzPhysics::SimulatedBodyHandle softBox = AzPhysics::InvalidSimulatedBodyHandle;
        MakeSliderRig(0.0f, false, hardBox);
        MakeSliderRig(20.0f, true, softBox);

        float hardMaxSlide = 0.0f;
        float softMaxSlide = 0.0f;
        const float fixedDeltaTime = 1.0f / 60.0f;
        for (int i = 0; i < 300; ++i)
        {
            m_scene->StartSimulation(fixedDeltaTime);
            m_scene->FinishSimulation();
            hardMaxSlide = AZStd::max(hardMaxSlide, GetBody(hardBox)->GetPosition().GetX() - 2.0f);
            softMaxSlide = AZStd::max(softMaxSlide, GetBody(softBox)->GetPosition().GetX() - 2.0f);
        }

        EXPECT_LT(hardMaxSlide, 0.8f);
        EXPECT_GT(softMaxSlide, 1.2f);
        // And the spring does eventually turn it around rather than letting it escape.
        EXPECT_LT(GetBody(softBox)->GetPosition().GetX() - 2.0f, softMaxSlide - 0.05f);
    }

    TEST_F(JoltJointTests, BreakableJointBreaksUnderExcessForceAndSignalsTheBreak)
    {
        auto anchorHandle = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        auto childHandle = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 4.0f));

        // The hanging box weighs ~9.81 N; a 5 N break force cannot hold it.
        JoltFixedJointConfiguration config;
        config.m_genericProperties.m_flags = JointGenericProperties::GenericJointFlag::Breakable;
        config.m_genericProperties.m_forceMax = 5.0f;
        config.m_genericProperties.m_torqueMax = 0.0f; // never break on torque
        SetFrames(config, AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.0f, 0.0f, 4.0f), AZ::Vector3(0.0f, 0.0f, 5.0f));
        auto jointHandle = m_scene->AddJoint(&config, anchorHandle, childHandle);
        ASSERT_NE(jointHandle, AzPhysics::InvalidJointHandle);

        auto* joltScene = azrtti_cast<JoltScene*>(m_scene);
        ASSERT_NE(joltScene, nullptr);
        AzPhysics::JointHandle brokenHandle = AzPhysics::InvalidJointHandle;
        JoltScene::JointBreakEvent::Handler breakHandler(
            [&brokenHandle](AzPhysics::JointHandle handle)
            {
                brokenHandle = handle;
            });
        joltScene->RegisterJointBreakHandler(breakHandler);

        SimulateSteps(120);

        // The break was signalled with this joint's handle, the joint is gone from the
        // scene, and the box has been falling freely ever since.
        EXPECT_EQ(brokenHandle, jointHandle);
        EXPECT_EQ(m_scene->GetJointFromHandle(jointHandle), nullptr);
        auto* child = GetBody(childHandle);
        ASSERT_NE(child, nullptr);
        EXPECT_LT(child->GetPosition().GetZ(), 2.0f);
    }

    TEST_F(JoltJointTests, BreakableJointHoldsBelowItsThreshold)
    {
        auto anchorHandle = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        auto childHandle = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 4.0f));

        JoltFixedJointConfiguration config;
        config.m_genericProperties.m_flags = JointGenericProperties::GenericJointFlag::Breakable;
        config.m_genericProperties.m_forceMax = 50.0f; // well above the ~9.81 N load
        config.m_genericProperties.m_torqueMax = 0.0f;
        SetFrames(config, AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.0f, 0.0f, 4.0f), AZ::Vector3(0.0f, 0.0f, 5.0f));
        auto jointHandle = m_scene->AddJoint(&config, anchorHandle, childHandle);
        ASSERT_NE(jointHandle, AzPhysics::InvalidJointHandle);

        SimulateSteps(120);

        EXPECT_NE(m_scene->GetJointFromHandle(jointHandle), nullptr);
        auto* child = GetBody(childHandle);
        ASSERT_NE(child, nullptr);
        EXPECT_NEAR(child->GetPosition().GetZ(), 4.0f, 0.1f);
    }

    TEST_F(JoltJointTests, BreakableJointBreaksOnTorqueAlone)
    {
        // A box held 2 m out to the side: gravity applies ~9.81 N of force but
        // ~19.6 N m of torque about the attachment. Force testing is off in both
        // rigs, so only the torque threshold separates them.
        auto MakeCantileverRig = [this](float y, float torqueMax)
        {
            const AZ::Vector3 anchorPosition(0.0f, y, 5.0f);
            const AZ::Vector3 childPosition(2.0f, y, 5.0f);
            auto anchor = CreateStaticBox(anchorPosition, AZ::Vector3(0.5f, 0.5f, 0.5f));
            auto child = CreateDynamicBox(childPosition);

            JoltFixedJointConfiguration config;
            config.m_genericProperties.m_flags = JointGenericProperties::GenericJointFlag::Breakable;
            config.m_genericProperties.m_forceMax = 0.0f; // never break on force
            config.m_genericProperties.m_torqueMax = torqueMax;
            SetFrames(config, anchorPosition, childPosition, anchorPosition);
            return m_scene->AddJoint(&config, anchor, child);
        };

        auto weakJoint = MakeCantileverRig(0.0f, 10.0f);
        auto strongJoint = MakeCantileverRig(20.0f, 100.0f);
        ASSERT_NE(weakJoint, AzPhysics::InvalidJointHandle);
        ASSERT_NE(strongJoint, AzPhysics::InvalidJointHandle);

        SimulateSteps(120);

        EXPECT_EQ(m_scene->GetJointFromHandle(weakJoint), nullptr);
        EXPECT_NE(m_scene->GetJointFromHandle(strongJoint), nullptr);
    }

    TEST_F(JoltJointTests, NonBreakableJointIgnoresBreakThresholds)
    {
        auto anchorHandle = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        auto childHandle = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 4.0f));

        // Thresholds far below the load, but the Breakable flag is not set.
        JoltFixedJointConfiguration config;
        config.m_genericProperties.m_forceMax = 0.001f;
        config.m_genericProperties.m_torqueMax = 0.001f;
        SetFrames(config, AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.0f, 0.0f, 4.0f), AZ::Vector3(0.0f, 0.0f, 5.0f));
        auto jointHandle = m_scene->AddJoint(&config, anchorHandle, childHandle);
        ASSERT_NE(jointHandle, AzPhysics::InvalidJointHandle);

        SimulateSteps(120);

        EXPECT_NE(m_scene->GetJointFromHandle(jointHandle), nullptr);
    }

    TEST_F(JoltJointTests, BallJointSoftLimitWarnsAndFallsBackToHard)
    {
        auto anchorHandle = CreateStaticBox(AZ::Vector3(0.0f, 0.0f, 6.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        auto childHandle = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 5.0f));

        JoltBallJointConfiguration config;
        config.m_debugName = "SoftBall";
        config.m_limitProperties.m_isLimited = true;
        config.m_limitProperties.m_isSoftLimit = true;
        config.m_limitProperties.m_limitFirst = 60.0f;
        config.m_limitProperties.m_limitSecond = 60.0f;
        SetFrames(config, AZ::Vector3(0.0f, 0.0f, 6.0f), AZ::Vector3(0.0f, 0.0f, 5.0f), AZ::Vector3(0.0f, 0.0f, 5.5f));

        // Jolt's swing-twist constraint has no limit spring, so the request has to be
        // called out rather than silently ignored - but the joint is still created.
        JoltWarningCatcher warnings;
        EXPECT_NE(m_scene->AddJoint(&config, anchorHandle, childHandle), AzPhysics::InvalidJointHandle);
        EXPECT_TRUE(warnings.ContainsWarningWith("SoftBall"));
        EXPECT_TRUE(warnings.ContainsWarningWith("soft limits are not supported"));
    }

    TEST_F(JoltJointTests, AnEntityCanCarrySeveralJointsAndAddressEachOne)
    {
        // Joints used to declare their own service incompatible with itself, so an entity
        // could carry exactly one - and the bus was addressed by entity, so even lifting
        // that would have made every request ambiguous. Content that hangs one body from
        // two attachments (a plank on two chains, a door with a hinge and a damper) needed
        // proxy entities, and a script could never name "the second joint".
        auto entity = AZStd::make_unique<AZ::Entity>("TwoJoints");
        entity->CreateComponent<AzFramework::TransformComponent>();
        auto* first = entity->CreateComponent<JoltHingeJointComponent>();
        auto* second = entity->CreateComponent<JoltHingeJointComponent>();
        ASSERT_NE(first, nullptr);
        ASSERT_NE(second, nullptr) << "a second joint component could not even be created";
        ASSERT_NE(first->GetId(), second->GetId());

        entity->Init();
        entity->Activate();
        EXPECT_EQ(entity->GetState(), AZ::Entity::State::Active)
            << "an entity carrying two joints failed to activate";

        // Each joint answers on its own address rather than both on the entity's.
        const AZ::EntityComponentIdPair firstAddress(entity->GetId(), first->GetId());
        const AZ::EntityComponentIdPair secondAddress(entity->GetId(), second->GetId());

        int firstHandlerCalls = 0;
        int secondHandlerCalls = 0;
        JoltJointRequestBus::EventResult(firstHandlerCalls, firstAddress, [](JoltJointRequests*) { return 1; });
        JoltJointRequestBus::EventResult(secondHandlerCalls, secondAddress, [](JoltJointRequests*) { return 1; });
        EXPECT_EQ(firstHandlerCalls, 1) << "the first joint did not answer on its own address";
        EXPECT_EQ(secondHandlerCalls, 1) << "the second joint did not answer on its own address";

        entity->Deactivate();
    }

} // namespace JoltPhysics
