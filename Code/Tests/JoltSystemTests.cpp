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

    TEST_F(JoltSystemTests, DefaultConfigurationSeedsTheDefaultCollisionLayer)
    {
        // AzFramework leaves CollisionLayers as 64 blank names, so without seeding here
        // the editor's layer dropdown has nothing to list and every collider sits on an
        // unnamed layer.
        const JoltSystemConfiguration config;
        const auto& layers = config.m_collisionConfig.m_collisionLayers;

        EXPECT_STREQ(layers.GetName(AzPhysics::CollisionLayer::Default).c_str(), "Default");

        AzPhysics::CollisionLayer resolved;
        EXPECT_TRUE(layers.TryGetLayer("Default", resolved));
        EXPECT_EQ(resolved.GetIndex(), AzPhysics::CollisionLayer::Default.GetIndex());
    }

    TEST_F(JoltSystemTests, DefaultConfigurationSeedsTheAllAndNoneGroups)
    {
        const JoltSystemConfiguration config;
        const auto& groups = config.m_collisionConfig.m_collisionGroups;

        ASSERT_EQ(groups.GetPresets().size(), 2u);

        const AzPhysics::CollisionGroup all = groups.FindGroupById(JoltSystemConfiguration::AllGroupId);
        const AzPhysics::CollisionGroup none = groups.FindGroupById(JoltSystemConfiguration::NoneGroupId);

        // The names are what the dropdown shows; the masks are what they have to mean.
        EXPECT_STREQ(groups.FindGroupNameById(JoltSystemConfiguration::AllGroupId).c_str(), "All");
        EXPECT_STREQ(groups.FindGroupNameById(JoltSystemConfiguration::NoneGroupId).c_str(), "None");
        EXPECT_TRUE(all.IsSet(AzPhysics::CollisionLayer::Default));
        EXPECT_FALSE(none.IsSet(AzPhysics::CollisionLayer::Default));

        for (const auto& preset : groups.GetPresets())
        {
            EXPECT_TRUE(preset.m_readOnly) << "Default groups must not be deletable: " << preset.m_name.c_str();
        }
    }

    TEST_F(JoltSystemTests, DefaultGroupIdsAreStableAcrossConfigurations)
    {
        // A collider serializes the group *id* it was authored with. Generating a fresh
        // id per construction would orphan every collider referencing it the next run.
        const JoltSystemConfiguration first;
        const JoltSystemConfiguration second;

        const auto& firstPresets = first.m_collisionConfig.m_collisionGroups.GetPresets();
        const auto& secondPresets = second.m_collisionConfig.m_collisionGroups.GetPresets();
        ASSERT_EQ(firstPresets.size(), secondPresets.size());

        for (size_t i = 0; i < firstPresets.size(); ++i)
        {
            EXPECT_EQ(firstPresets[i].m_id, secondPresets[i].m_id);
        }
    }

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
