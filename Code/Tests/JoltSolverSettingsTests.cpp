#include <AzTest/AzTest.h>

#include <Configuration/JoltSettingsRegistryManager.h>
#include <Scene/JoltScene.h>
#include <System/JoltSystem.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace JoltPhysics
{
    //! Verifies the solver portion of JoltSystemConfiguration actually reaches the
    //! native JPH::PhysicsSystem of each scene — at scene creation and again when the
    //! configuration of a live system is updated (the configuration window path).
    class JoltSolverSettingsTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            m_system = AZStd::make_unique<JoltSystem>(AZStd::make_unique<JoltSettingsRegistryManager>());
        }

        void TearDown() override
        {
            m_system->Shutdown();
            m_system.reset();
        }

        AzPhysics::Scene* AddScene(const AzPhysics::SceneConfiguration& sceneConfig)
        {
            m_sceneHandle = m_system->AddScene(sceneConfig);
            return m_system->GetScene(m_sceneHandle);
        }

        static const JPH::PhysicsSettings& NativeSettings(AzPhysics::Scene* scene)
        {
            return static_cast<JPH::PhysicsSystem*>(scene->GetNativePointer())->GetPhysicsSettings();
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle = AzPhysics::InvalidSceneHandle;
    };

    TEST_F(JoltSolverSettingsTests, SolverSettings_ReachTheNativePhysicsSystemAtSceneCreation)
    {
        JoltSystemConfiguration config;
        config.m_numVelocitySteps = 16;
        config.m_numPositionSteps = 4;
        config.m_penetrationSlop = 0.005f;
        config.m_timeBeforeSleep = 1.5f;
        config.m_allowSleeping = false;
        config.m_deterministicSimulation = false;
        m_system->Initialize(&config);

        AzPhysics::SceneConfiguration sceneConfig;
        sceneConfig.m_sceneName = "SolverSettingsScene";
        AzPhysics::Scene* scene = AddScene(sceneConfig);
        ASSERT_NE(nullptr, scene);

        const JPH::PhysicsSettings& settings = NativeSettings(scene);
        EXPECT_EQ(16u, settings.mNumVelocitySteps);
        EXPECT_EQ(4u, settings.mNumPositionSteps);
        EXPECT_FLOAT_EQ(0.005f, settings.mPenetrationSlop);
        EXPECT_FLOAT_EQ(1.5f, settings.mTimeBeforeSleep);
        EXPECT_FALSE(settings.mAllowSleeping);
        EXPECT_FALSE(settings.mDeterministicSimulation);

        // Fields the configuration does not cover keep Jolt's defaults.
        EXPECT_TRUE(settings.mConstraintWarmStart);
        EXPECT_TRUE(settings.mUseManifoldReduction);
    }

    TEST_F(JoltSolverSettingsTests, SolverSettings_ApplyToLiveScenesOnUpdateConfiguration)
    {
        JoltSystemConfiguration config;
        m_system->Initialize(&config);

        AzPhysics::SceneConfiguration sceneConfig;
        sceneConfig.m_sceneName = "LiveUpdateScene";
        AzPhysics::Scene* scene = AddScene(sceneConfig);
        ASSERT_NE(nullptr, scene);
        EXPECT_EQ(10u, NativeSettings(scene).mNumVelocitySteps);

        config.m_numVelocitySteps = 20;
        config.m_speculativeContactDistance = 0.1f;
        m_system->UpdateConfiguration(&config);

        EXPECT_EQ(20u, NativeSettings(scene).mNumVelocitySteps);
        EXPECT_FLOAT_EQ(0.1f, NativeSettings(scene).mSpeculativeContactDistance);
    }

    TEST_F(JoltSolverSettingsTests, CapacitySettings_ReachTheNativePhysicsSystem)
    {
        JoltSystemConfiguration config;
        config.m_maxBodies = 128;
        m_system->Initialize(&config);

        AzPhysics::SceneConfiguration sceneConfig;
        sceneConfig.m_sceneName = "CapacityScene";
        AzPhysics::Scene* scene = AddScene(sceneConfig);
        ASSERT_NE(nullptr, scene);

        EXPECT_EQ(128u, static_cast<JPH::PhysicsSystem*>(scene->GetNativePointer())->GetMaxBodies());
    }

    TEST_F(JoltSolverSettingsTests, CollisionSteps_ApplyAtCreationAndOnLiveUpdate)
    {
        JoltSystemConfiguration config;
        config.m_collisionSteps = 2;
        m_system->Initialize(&config);

        AzPhysics::SceneConfiguration sceneConfig;
        sceneConfig.m_sceneName = "CollisionStepsScene";
        auto* scene = azdynamic_cast<JoltScene*>(AddScene(sceneConfig));
        ASSERT_NE(nullptr, scene);
        EXPECT_EQ(2, scene->GetCollisionSteps());

        config.m_collisionSteps = 3;
        m_system->UpdateConfiguration(&config);
        EXPECT_EQ(3, scene->GetCollisionSteps());

        // Zero or negative would assert inside Jolt's Update; the scene clamps.
        config.m_collisionSteps = 0;
        m_system->UpdateConfiguration(&config);
        EXPECT_EQ(1, scene->GetCollisionSteps());
    }

} // namespace JoltPhysics
