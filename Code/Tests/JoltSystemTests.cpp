#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <System/JoltSystem.h>
#include <Configuration/JoltSettingsRegistryManager.h>

namespace JoltPhysics
{
    class JoltSystemTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
        }

        void TearDown() override
        {
        }
    };

    TEST_F(JoltSystemTests, SystemCanBeCreated)
    {
        auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
        JoltSystem system(AZStd::move(registryManager));

        EXPECT_EQ(system.GetConfiguration(), nullptr);
    }

    TEST_F(JoltSystemTests, SystemCanBeInitialized)
    {
        auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
        JoltSystem system(AZStd::move(registryManager));

        JoltSystemConfiguration config;
        system.Initialize(&config);

        EXPECT_NE(system.GetConfiguration(), nullptr);

        system.Shutdown();
    }

    TEST_F(JoltSystemTests, SceneCanBeAdded)
    {
        auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
        JoltSystem system(AZStd::move(registryManager));

        JoltSystemConfiguration config;
        system.Initialize(&config);

        AzPhysics::SceneConfiguration sceneConfig;
        sceneConfig.m_sceneName = "TestScene";

        AzPhysics::SceneHandle handle = system.AddScene(sceneConfig);

        EXPECT_NE(handle, AzPhysics::InvalidSceneHandle);
        EXPECT_NE(system.GetScene(handle), nullptr);

        system.RemoveScene(handle);
        system.Shutdown();
    }

} // namespace JoltPhysics
