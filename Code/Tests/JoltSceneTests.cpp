#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <System/JoltSystem.h>
#include <Scene/JoltScene.h>
#include <Configuration/JoltSettingsRegistryManager.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>

namespace JoltPhysics
{
    class JoltSceneTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "TestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);
        }

        void TearDown() override
        {
            m_system->RemoveScene(m_sceneHandle);
            m_system->Shutdown();
            m_system.reset();
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle;
        AzPhysics::Scene* m_scene = nullptr;
    };

    TEST_F(JoltSceneTests, SceneIsValid)
    {
        ASSERT_NE(m_scene, nullptr);
        EXPECT_TRUE(m_scene->IsEnabled());
    }

    TEST_F(JoltSceneTests, GravityCanBeSet)
    {
        AZ::Vector3 newGravity(0.0f, 0.0f, -20.0f);
        m_scene->SetGravity(newGravity);

        AZ::Vector3 gravity = m_scene->GetGravity();
        EXPECT_FLOAT_EQ(gravity.GetZ(), -20.0f);
    }

    TEST_F(JoltSceneTests, SceneCanBeDisabled)
    {
        m_scene->SetEnabled(false);
        EXPECT_FALSE(m_scene->IsEnabled());

        m_scene->SetEnabled(true);
        EXPECT_TRUE(m_scene->IsEnabled());
    }

    TEST_F(JoltSceneTests, RigidBodyCanBeAdded)
    {
        AzPhysics::RigidBodyConfiguration rigidBodyConfig;
        rigidBodyConfig.m_debugName = "TestBody";
        rigidBodyConfig.m_position = AZ::Vector3(0.0f, 0.0f, 10.0f);
        rigidBodyConfig.m_mass = 1.0f;

        AzPhysics::SimulatedBodyHandle handle = m_scene->AddSimulatedBody(&rigidBodyConfig);

        EXPECT_NE(handle, AzPhysics::InvalidSimulatedBodyHandle);

        AzPhysics::SimulatedBody* body = m_scene->GetSimulatedBodyFromHandle(handle);
        EXPECT_NE(body, nullptr);

        m_scene->RemoveSimulatedBody(handle);
    }

} // namespace JoltPhysics
