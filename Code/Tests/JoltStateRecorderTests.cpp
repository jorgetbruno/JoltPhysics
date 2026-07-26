#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include "JoltTestWarningCatcher.h"

#include <Configuration/JoltSettingsRegistryManager.h>
#include <Character/JoltCharacter.h>
#include <RigidBody/JoltRigidBody.h>
#include <Scene/JoltScene.h>
#include <System/JoltSystem.h>
#include <SoftBody/JoltSoftBody.h>
#include <Vehicle/JoltVehicle.h>
#include <Vehicle/JoltVehicleConfiguration.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/Character.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    //! Scene snapshots: SaveSimulationState / RestoreSimulationState.
    //!
    //! The replay assertions use exact float equality on purpose. Jolt is deterministic
    //! for the same binary on the same machine, and a snapshot covers bodies, contacts
    //! and constraints - so a resimulation from a restored state must retrace the
    //! original bit for bit. "Close" would pass while quietly leaving something out of
    //! the snapshot; only exactness proves the state is complete.
    class JoltStateRecorderTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            m_system = AZStd::make_unique<JoltSystem>(AZStd::make_unique<JoltSettingsRegistryManager>());
            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "StateRecorderTestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = azrtti_cast<JoltScene*>(m_system->GetScene(m_sceneHandle));
            ASSERT_NE(m_scene, nullptr);
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

            AzPhysics::StaticRigidBodyConfiguration config;
            config.m_position = position;
            config.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);
            return m_scene->AddSimulatedBody(&config);
        }

        AzPhysics::SimulatedBodyHandle CreateDynamicBox(
            const AZ::Vector3& position, const AZ::Vector3& initialLinearVelocity = AZ::Vector3::CreateZero())
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            boxShape->m_dimensions = AZ::Vector3(0.5f, 0.5f, 0.5f);

            AzPhysics::RigidBodyConfiguration config;
            config.m_position = position;
            config.m_initialLinearVelocity = initialLinearVelocity;
            config.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);
            return m_scene->AddSimulatedBody(&config);
        }

        AzPhysics::RigidBody* GetBody(AzPhysics::SimulatedBodyHandle handle)
        {
            return azdynamic_cast<AzPhysics::RigidBody*>(m_scene->GetSimulatedBodyFromHandle(handle));
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

        //! Ground plus a small unstable stack with sideways motion: enough contacts and
        //! impacts that an incomplete snapshot could not replay it exactly.
        AZStd::vector<AzPhysics::SimulatedBodyHandle> BuildTumblingStack()
        {
            CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(60.0f, 60.0f, 1.0f));
            AZStd::vector<AzPhysics::SimulatedBodyHandle> boxes;
            boxes.push_back(CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 0.25f)));
            boxes.push_back(CreateDynamicBox(AZ::Vector3(0.1f, 0.05f, 0.8f)));
            boxes.push_back(CreateDynamicBox(AZ::Vector3(-0.05f, 0.1f, 1.35f)));
            boxes.push_back(CreateDynamicBox(AZ::Vector3(2.0f, 0.0f, 2.0f), AZ::Vector3(-3.0f, 0.5f, 0.0f)));
            return boxes;
        }

        struct BodySnapshot
        {
            AZ::Vector3 m_position;
            AZ::Quaternion m_orientation;
            AZ::Vector3 m_linearVelocity;
            AZ::Vector3 m_angularVelocity;
        };

        AZStd::vector<BodySnapshot> Snapshot(const AZStd::vector<AzPhysics::SimulatedBodyHandle>& handles)
        {
            AZStd::vector<BodySnapshot> snapshots;
            for (const auto handle : handles)
            {
                AzPhysics::RigidBody* body = GetBody(handle);
                snapshots.push_back(
                    { body->GetPosition(), body->GetOrientation(), body->GetLinearVelocity(),
                      body->GetAngularVelocity() });
            }
            return snapshots;
        }

        static void ExpectIdentical(const AZStd::vector<BodySnapshot>& a, const AZStd::vector<BodySnapshot>& b)
        {
            ASSERT_EQ(a.size(), b.size());
            for (size_t i = 0; i < a.size(); ++i)
            {
                EXPECT_EQ(a[i].m_position, b[i].m_position) << "body " << i;
                EXPECT_EQ(a[i].m_orientation, b[i].m_orientation) << "body " << i;
                EXPECT_EQ(a[i].m_linearVelocity, b[i].m_linearVelocity) << "body " << i;
                EXPECT_EQ(a[i].m_angularVelocity, b[i].m_angularVelocity) << "body " << i;
            }
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle;
        JoltScene* m_scene = nullptr;
    };

    TEST_F(JoltStateRecorderTests, RestoreReturnsTheSceneToTheSavedInstant)
    {
        const auto boxes = BuildTumblingStack();
        SimulateSteps(30);

        AZStd::vector<AZ::u8> state;
        ASSERT_TRUE(m_scene->SaveSimulationState(state));
        EXPECT_FALSE(state.empty());
        const auto atSave = Snapshot(boxes);

        SimulateSteps(45);
        // The stack has genuinely moved on, or restoring proves nothing.
        EXPECT_NE(Snapshot(boxes)[3].m_position, atSave[3].m_position);

        ASSERT_TRUE(m_scene->RestoreSimulationState(state));
        ExpectIdentical(Snapshot(boxes), atSave);
    }

    TEST_F(JoltStateRecorderTests, AReplayFromARestoredStateRetracesTheOriginalExactly)
    {
        const auto boxes = BuildTumblingStack();
        SimulateSteps(30);

        AZStd::vector<AZ::u8> state;
        ASSERT_TRUE(m_scene->SaveSimulationState(state));

        // The original run: on through 60 more steps of tumbling and impacts.
        SimulateSteps(60);
        const auto original = Snapshot(boxes);

        // Rollback and replay - twice, so the second replay also starts from a
        // restored state rather than from a freshly saved one.
        ASSERT_TRUE(m_scene->RestoreSimulationState(state));
        SimulateSteps(60);
        ExpectIdentical(Snapshot(boxes), original);

        ASSERT_TRUE(m_scene->RestoreSimulationState(state));
        SimulateSteps(60);
        ExpectIdentical(Snapshot(boxes), original);
    }

    TEST_F(JoltStateRecorderTests, RestoreFailsWhenABodyWasAddedSinceTheSave)
    {
        // Jolt's own RestoreState would NOT catch this: it restores the bodies it
        // saved and silently leaves the newcomer as it is - a rollback that skips one
        // body. The gem compares body counts before applying anything, so this fails
        // atomically, and the body added after the save keeps its state to prove
        // nothing was half-applied.
        const auto boxes = BuildTumblingStack();
        SimulateSteps(10);

        AZStd::vector<AZ::u8> state;
        ASSERT_TRUE(m_scene->SaveSimulationState(state));

        const auto lateHandle = CreateDynamicBox(AZ::Vector3(10.0f, 0.0f, 1.0f));
        SimulateSteps(20);
        const AZ::Vector3 latePosition = GetBody(lateHandle)->GetPosition();
        const auto beforeFailedRestore = Snapshot(boxes);

        JoltWarningCatcher warnings;
        EXPECT_FALSE(m_scene->RestoreSimulationState(state));
        EXPECT_TRUE(warnings.ContainsWarningWith("does not match"));

        // Atomic: the failed restore touched nothing.
        ExpectIdentical(Snapshot(boxes), beforeFailedRestore);
        EXPECT_EQ(GetBody(lateHandle)->GetPosition(), latePosition);
    }

    TEST_F(JoltStateRecorderTests, RestoreFailsWhenABodyWasRemovedSinceTheSave)
    {
        auto boxes = BuildTumblingStack();
        SimulateSteps(10);

        AZStd::vector<AZ::u8> state;
        ASSERT_TRUE(m_scene->SaveSimulationState(state));

        m_scene->RemoveSimulatedBody(boxes.back());

        JoltWarningCatcher warnings;
        EXPECT_FALSE(m_scene->RestoreSimulationState(state));
        EXPECT_TRUE(warnings.ContainsWarningWith("does not match"));
    }

    TEST_F(JoltStateRecorderTests, RestoreRejectsABlobThatIsNotASnapshot)
    {
        BuildTumblingStack();

        JoltWarningCatcher warnings;

        const AZStd::vector<AZ::u8> garbage = { 0xDE, 0xAD, 0xBE, 0xEF, 0x42 };
        EXPECT_FALSE(m_scene->RestoreSimulationState(garbage));

        const AZStd::vector<AZ::u8> tooShort = { 0x01 };
        EXPECT_FALSE(m_scene->RestoreSimulationState(tooShort));

        EXPECT_FALSE(m_scene->RestoreSimulationState({}));
    }

    TEST_F(JoltStateRecorderTests, AVirtualCharacterComesBackWithTheScene)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(60.0f, 60.0f, 1.0f));

        Physics::CharacterConfiguration characterConfig;
        characterConfig.m_position = AZ::Vector3(0.0f, 0.0f, 0.0f);
        characterConfig.m_debugName = "StateRecorderCharacter";
        characterConfig.m_shapeConfig = AZStd::make_shared<Physics::CapsuleShapeConfiguration>(1.8f, 0.4f);
        const auto characterHandle = m_scene->AddSimulatedBody(&characterConfig);
        ASSERT_NE(characterHandle, AzPhysics::InvalidSimulatedBodyHandle);
        auto* character = azdynamic_cast<JoltCharacter*>(m_scene->GetSimulatedBodyFromHandle(characterHandle));
        ASSERT_NE(character, nullptr);
        ASSERT_FALSE(character->IsRigidBodyCharacter()); // the default backend is CharacterVirtual

        SimulateSteps(30); // settle onto the slab

        AZStd::vector<AZ::u8> state;
        ASSERT_TRUE(m_scene->SaveSimulationState(state));
        const AZ::Vector3 savedPosition = character->GetBasePosition();

        // Walk away: a CharacterVirtual's position lives outside the body system, so if
        // the snapshot missed it this movement would survive the restore.
        for (int i = 0; i < 60; ++i)
        {
            character->AddVelocityForTick(AZ::Vector3(3.0f, 0.0f, 0.0f));
            SimulateSteps(1);
        }
        ASSERT_GT(character->GetBasePosition().GetX(), savedPosition.GetX() + 1.0f);

        ASSERT_TRUE(m_scene->RestoreSimulationState(state));
        EXPECT_EQ(character->GetBasePosition(), savedPosition);
    }

    TEST_F(JoltStateRecorderTests, SoftBodyVerticesComeBackWithTheScene)
    {
        // Soft body state is per vertex, saved through the body's motion properties -
        // a whole different path from rigid positions, so it gets its own proof.
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(60.0f, 60.0f, 1.0f));

        JoltSoftBodySettings clothSettings;
        clothSettings.m_shape = JoltSoftBodyShape::Cloth;
        clothSettings.m_pinning = JoltSoftBodyPinning::None;
        clothSettings.m_size = AZ::Vector3(2.0f, 2.0f, 2.0f);
        clothSettings.m_resolution = 6;
        clothSettings.m_mass = 2.0f;
        clothSettings.m_allowSleeping = false;

        JoltSoftBodyConfiguration clothConfig;
        clothConfig.m_settings = clothSettings;
        clothConfig.m_position = AZ::Vector3(0.0f, 0.0f, 3.0f);
        clothConfig.m_debugName = "StateRecorderCloth";
        const auto clothHandle = m_scene->AddSimulatedBody(&clothConfig);
        ASSERT_NE(clothHandle, AzPhysics::InvalidSimulatedBodyHandle);
        auto* cloth = azdynamic_cast<JoltSoftBody*>(m_scene->GetSimulatedBodyFromHandle(clothHandle));
        ASSERT_NE(cloth, nullptr);

        SimulateSteps(20); // mid-fall, already deforming

        AZStd::vector<AZ::u8> state;
        ASSERT_TRUE(m_scene->SaveSimulationState(state));
        const AZ::Vector3 savedVertex = cloth->GetVertexPosition(0);
        const AZ::Vector3 savedFarVertex = cloth->GetVertexPosition(35); // 6x6 grid, opposite corner

        SimulateSteps(60); // lands and crumples
        ASSERT_NE(cloth->GetVertexPosition(0), savedVertex);

        ASSERT_TRUE(m_scene->RestoreSimulationState(state));
        EXPECT_EQ(cloth->GetVertexPosition(0), savedVertex);
        EXPECT_EQ(cloth->GetVertexPosition(35), savedFarVertex);
    }

    TEST_F(JoltStateRecorderTests, AVehicleReplaysItsDriveExactly)
    {
        CreateStaticBox(AZ::Vector3(0.0f, 0.0f, -0.5f), AZ::Vector3(400.0f, 200.0f, 1.0f));

        auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        boxShape->m_dimensions = AZ::Vector3(2.0f, 1.0f, 0.5f);
        AzPhysics::RigidBodyConfiguration chassisConfig;
        chassisConfig.m_position = AZ::Vector3(0.0f, 0.0f, 0.9f);
        chassisConfig.m_mass = 1200.0f;
        chassisConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);
        const auto chassisHandle = m_scene->AddSimulatedBody(&chassisConfig);
        ASSERT_NE(chassisHandle, AzPhysics::InvalidSimulatedBodyHandle);

        JoltVehicleConfiguration vehicleConfig; // default wheel layout
        vehicleConfig.m_chassisMass = 1200.0f;
        JoltVehicle vehicle(vehicleConfig, m_scene, m_scene->GetJoltBody(chassisHandle));
        ASSERT_TRUE(vehicle.IsValid());

        auto drive = [&](int steps)
        {
            for (int i = 0; i < steps; ++i)
            {
                vehicle.SetDriverInput(1.0f, 0.2f, 0.0f, 0.0f);
                SimulateSteps(1);
            }
        };

        SimulateSteps(60); // settle the suspension
        drive(60);         // under way, engine spun up, wheels loaded

        AZStd::vector<AZ::u8> state;
        ASSERT_TRUE(m_scene->SaveSimulationState(state));

        // The vehicle constraint holds wheel speeds, engine RPM and gearbox state; the
        // exact-replay assertion fails if any of it is missing from the snapshot.
        drive(90);
        auto* chassis = GetBody(chassisHandle);
        const AZ::Vector3 originalPosition = chassis->GetPosition();
        const AZ::Quaternion originalOrientation = chassis->GetOrientation();
        const float originalRpm = vehicle.GetEngineRpm();

        ASSERT_TRUE(m_scene->RestoreSimulationState(state));
        drive(90);
        EXPECT_EQ(chassis->GetPosition(), originalPosition);
        EXPECT_EQ(chassis->GetOrientation(), originalOrientation);
        EXPECT_EQ(vehicle.GetEngineRpm(), originalRpm);
    }

} // namespace JoltPhysics
