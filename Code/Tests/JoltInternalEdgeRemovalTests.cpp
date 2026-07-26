#include <AzTest/AzTest.h>

#include <AzFramework/Physics/Character.h>
#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

#include <Character/JoltCharacter.h>
#include <Configuration/JoltSettingsRegistryManager.h>
#include <Scene/JoltScene.h>
#include <System/JoltSystem.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Character/Character.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace JoltPhysics
{
    //! Enhanced internal edge removal suppresses ghost contacts against the seams
    //! between a mesh's triangles. Jolt decides per contact pair with an OR, so the gem
    //! sets the flag on the bodies that move; these tests check it reaches the native
    //! bodies, and that the vertex tolerance reaches JPH::PhysicsSettings squared.
    class JoltInternalEdgeRemovalTests : public ::testing::Test
    {
    protected:
        void TearDown() override
        {
            if (m_system)
            {
                m_system->Shutdown();
                m_system.reset();
            }
        }

        //! Brings up a system and one scene with the given configuration.
        void StartSystem(const JoltSystemConfiguration& config)
        {
            m_system = AZStd::make_unique<JoltSystem>(AZStd::make_unique<JoltSettingsRegistryManager>());
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "InternalEdgeRemovalScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);
            ASSERT_NE(m_scene, nullptr);
        }

        JPH::PhysicsSystem* NativeSystem() const
        {
            return static_cast<JPH::PhysicsSystem*>(m_scene->GetNativePointer());
        }

        AzPhysics::SimulatedBodyHandle AddDynamicBox()
        {
            AzPhysics::RigidBodyConfiguration config;
            config.m_position = AZ::Vector3(0.0f, 0.0f, 1.0f);
            config.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(
                AZStd::make_shared<Physics::ColliderConfiguration>(),
                AZStd::make_shared<Physics::BoxShapeConfiguration>());
            return m_scene->AddSimulatedBody(&config);
        }

        bool NativeBodyHasEdgeRemoval(JPH::BodyID bodyId) const
        {
            JPH::BodyLockRead lock(NativeSystem()->GetBodyLockInterface(), bodyId);
            EXPECT_TRUE(lock.Succeeded());
            return lock.Succeeded() && lock.GetBody().GetEnhancedInternalEdgeRemoval();
        }

        //! Reads the flag off the native body behind a rigid body handle. JoltRigidBody
        //! hands out its JPH::BodyID as the native pointer.
        bool RigidBodyHasEdgeRemoval(AzPhysics::SimulatedBodyHandle handle) const
        {
            AzPhysics::SimulatedBody* body = m_scene->GetSimulatedBodyFromHandle(handle);
            EXPECT_NE(body, nullptr);
            const auto* bodyId = static_cast<const JPH::BodyID*>(body->GetNativePointer());
            EXPECT_NE(bodyId, nullptr);
            return NativeBodyHasEdgeRemoval(*bodyId);
        }

        AzPhysics::SimulatedBodyHandle AddCharacter(bool rigidBodyBacked)
        {
            JoltCharacterConfiguration config;
            config.m_position = AZ::Vector3::CreateZero();
            config.m_debugName = "EdgeRemovalCharacter";
            config.m_shapeConfig = AZStd::make_shared<Physics::CapsuleShapeConfiguration>(1.8f, 0.3f);
            config.m_rigidBodyCharacter = rigidBodyBacked;
            return m_scene->AddSimulatedBody(&config);
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle = AzPhysics::InvalidSceneHandle;
        AzPhysics::Scene* m_scene = nullptr;
    };

    TEST_F(JoltInternalEdgeRemovalTests, TheToleranceReachesPhysicsSettingsSquared)
    {
        JoltSystemConfiguration config;
        config.m_internalEdgeRemovalTolerance = 0.01f;
        StartSystem(config);

        // Jolt stores the tolerance squared; the configuration exposes plain metres.
        EXPECT_FLOAT_EQ(NativeSystem()->GetPhysicsSettings().mInternalEdgeRemovalVertexToleranceSq, 0.0001f);
    }

    TEST_F(JoltInternalEdgeRemovalTests, TheDefaultToleranceMatchesJoltsOwn)
    {
        StartSystem(JoltSystemConfiguration());

        // Jolt's cDefaultInternalEdgeRemovalVertexToleranceSq is 1e-8, i.e. 1e-4 m.
        EXPECT_NEAR(NativeSystem()->GetPhysicsSettings().mInternalEdgeRemovalVertexToleranceSq, 1.0e-8f, 1.0e-12f);
    }

    TEST_F(JoltInternalEdgeRemovalTests, DynamicBodiesGetTheFlagByDefault)
    {
        StartSystem(JoltSystemConfiguration());

        // On by default: ghost contacts are hard to diagnose from content, so the gem
        // opts in rather than following Jolt's off-by-default.
        EXPECT_TRUE(RigidBodyHasEdgeRemoval(AddDynamicBox()));
    }

    TEST_F(JoltInternalEdgeRemovalTests, TheFlagCanBeTurnedOff)
    {
        JoltSystemConfiguration config;
        config.m_enhancedInternalEdgeRemoval = false;
        StartSystem(config);

        EXPECT_FALSE(RigidBodyHasEdgeRemoval(AddDynamicBox()));
    }

    TEST_F(JoltInternalEdgeRemovalTests, VirtualCharactersGetTheFlag)
    {
        StartSystem(JoltSystemConfiguration());

        auto* character = azdynamic_cast<JoltCharacter*>(
            m_scene->GetSimulatedBodyFromHandle(AddCharacter(/*rigidBodyBacked*/ false)));
        ASSERT_NE(character, nullptr);

        auto* nativeCharacter = static_cast<JPH::CharacterVirtual*>(character->GetNativePointer());
        ASSERT_NE(nativeCharacter, nullptr);
        EXPECT_TRUE(nativeCharacter->GetEnhancedInternalEdgeRemoval());
    }

    TEST_F(JoltInternalEdgeRemovalTests, VirtualCharactersRespectTheSettingBeingOff)
    {
        JoltSystemConfiguration config;
        config.m_enhancedInternalEdgeRemoval = false;
        StartSystem(config);

        auto* character = azdynamic_cast<JoltCharacter*>(
            m_scene->GetSimulatedBodyFromHandle(AddCharacter(/*rigidBodyBacked*/ false)));
        ASSERT_NE(character, nullptr);

        auto* nativeCharacter = static_cast<JPH::CharacterVirtual*>(character->GetNativePointer());
        ASSERT_NE(nativeCharacter, nullptr);
        EXPECT_FALSE(nativeCharacter->GetEnhancedInternalEdgeRemoval());
    }

    TEST_F(JoltInternalEdgeRemovalTests, RigidBodyCharactersGetTheFlag)
    {
        StartSystem(JoltSystemConfiguration());

        // The rigid backend hands out its JPH::Character, which passes the flag through
        // to the body it creates (Character.cpp applies it to the creation settings).
        auto* character = azdynamic_cast<JoltCharacter*>(
            m_scene->GetSimulatedBodyFromHandle(AddCharacter(/*rigidBodyBacked*/ true)));
        ASSERT_NE(character, nullptr);

        auto* nativeCharacter = static_cast<JPH::Character*>(character->GetNativePointer());
        ASSERT_NE(nativeCharacter, nullptr);
        EXPECT_TRUE(NativeBodyHasEdgeRemoval(nativeCharacter->GetBodyID()));
    }

} // namespace JoltPhysics
