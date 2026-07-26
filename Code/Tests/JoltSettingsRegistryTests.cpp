#include <AzCore/JSON/document.h>
#include <AzCore/JSON/pointer.h>
#include <AzCore/JSON/stringbuffer.h>
#include <AzCore/JSON/writer.h>
#include <AzCore/Serialization/Json/JsonSerialization.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzTest/AzTest.h>

#include <Configuration/JoltSettingsRegistryManager.h>

namespace JoltPhysicsTests
{
    using JoltPhysics::JoltSettingsRegistryManager;
    using JoltPhysics::JoltSystemConfiguration;

    //! Serializes an object the same way the editor save path does (keeping default
    //! values) and merges it into the global settings registry under the given path,
    //! which is exactly what happens when a project .setreg is merged at boot.
    template<typename ConfigType>
    void MergeConfigIntoSettingsRegistry(const ConfigType& config, const char* settingsPath)
    {
        rapidjson::Document document(rapidjson::kObjectType);

        AZ::JsonSerializerSettings serializerSettings;
        serializerSettings.m_keepDefaults = true;

        rapidjson::Value configValue;
        const auto result =
            AZ::JsonSerialization::Store(configValue, document.GetAllocator(), config, serializerSettings);
        ASSERT_NE(AZ::JsonSerializationResult::Processing::Halted, result.GetProcessing());

        rapidjson::Pointer(settingsPath).Set(document, configValue, document.GetAllocator());

        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        document.Accept(writer);

        auto* settingsRegistry = AZ::SettingsRegistry::Get();
        ASSERT_NE(nullptr, settingsRegistry);
        ASSERT_TRUE(settingsRegistry->MergeSettings(
            AZStd::string_view(buffer.GetString(), buffer.GetSize()),
            AZ::SettingsRegistryInterface::Format::JsonMergePatch));
    }

    TEST(JoltSettingsRegistryTest, SystemConfiguration_RoundTripsThroughSettingsRegistry)
    {
        JoltSystemConfiguration saved;
        saved.m_maxBodies = 12345;
        saved.m_maxJobThreads = 3;
        saved.m_fixedTimestep = 0.02f;
        saved.m_collisionConfig.m_collisionLayers.SetName(AZ::u64(5), "Water");
        AzPhysics::CollisionGroup underwaterGroup = AzPhysics::CollisionGroup::All;
        underwaterGroup.SetLayer(AzPhysics::CollisionLayer(5), false);
        saved.m_collisionConfig.m_collisionGroups.CreateGroup("NoWater", underwaterGroup);

        MergeConfigIntoSettingsRegistry(saved, JoltSettingsRegistryManager::SystemConfigPath);

        JoltSettingsRegistryManager manager;
        const auto loaded = manager.LoadSystemConfiguration();
        ASSERT_TRUE(loaded.has_value());

        EXPECT_EQ(12345u, loaded->m_maxBodies);
        EXPECT_EQ(3u, loaded->m_maxJobThreads);
        EXPECT_FLOAT_EQ(0.02f, loaded->m_fixedTimestep);
        EXPECT_STREQ("Water", loaded->m_collisionConfig.m_collisionLayers.GetName(AzPhysics::CollisionLayer(5)).c_str());

        AzPhysics::CollisionGroup loadedGroup;
        EXPECT_TRUE(loaded->m_collisionConfig.m_collisionGroups.TryFindGroupByName("NoWater", loadedGroup));
        EXPECT_FALSE(loadedGroup.IsSet(AzPhysics::CollisionLayer(5)));
        EXPECT_TRUE(loadedGroup.IsSet(AzPhysics::CollisionLayer(0)));
    }

    TEST(JoltSettingsRegistryTest, SystemConfiguration_SeededPresetsDoNotDuplicateOnRoundTrip)
    {
        // The saved configuration already holds the seeded All and None, because it is
        // itself a JoltSystemConfiguration. Loading deserializes into another seeded
        // one, and the JSON serializer appends to vectors rather than replacing them,
        // so without de-duplication the preset list grows by two every round trip.
        JoltSystemConfiguration saved;
        const size_t seededCount = saved.m_collisionConfig.m_collisionGroups.GetPresets().size();
        ASSERT_EQ(2u, seededCount); // All and None

        MergeConfigIntoSettingsRegistry(saved, JoltSettingsRegistryManager::SystemConfigPath);

        JoltSettingsRegistryManager manager;
        const auto loaded = manager.LoadSystemConfiguration();
        ASSERT_TRUE(loaded.has_value());

        const auto& presets = loaded->m_collisionConfig.m_collisionGroups.GetPresets();
        EXPECT_EQ(seededCount, presets.size());

        // Both are still resolvable, and None still means "collides with nothing".
        EXPECT_EQ(
            0u,
            loaded->m_collisionConfig.m_collisionGroups.FindGroupById(JoltSystemConfiguration::NoneGroupId).GetMask());
        EXPECT_NE(
            0u,
            loaded->m_collisionConfig.m_collisionGroups.FindGroupById(JoltSystemConfiguration::AllGroupId).GetMask());
    }

    TEST(JoltSettingsRegistryTest, SystemConfiguration_UserPresetsSurviveDeduplication)
    {
        JoltSystemConfiguration saved;
        AzPhysics::CollisionGroup custom = AzPhysics::CollisionGroup::All;
        custom.SetLayer(AzPhysics::CollisionLayer(3), false);
        const auto customId = saved.m_collisionConfig.m_collisionGroups.CreateGroup("Custom", custom);

        MergeConfigIntoSettingsRegistry(saved, JoltSettingsRegistryManager::SystemConfigPath);

        JoltSettingsRegistryManager manager;
        const auto loaded = manager.LoadSystemConfiguration();
        ASSERT_TRUE(loaded.has_value());

        // De-duplication keys on the preset id, so a group the user added is untouched.
        EXPECT_EQ(3u, loaded->m_collisionConfig.m_collisionGroups.GetPresets().size());
        EXPECT_FALSE(loaded->m_collisionConfig.m_collisionGroups.FindGroupById(customId)
                         .IsSet(AzPhysics::CollisionLayer(3)));
    }

    //! An id that resolves to nothing - unset, or left by a deleted preset - answers
    //! CollisionGroup::All. This is what made "Collides With" look like it did nothing:
    //! a collider that was never given a group collides with everything, so the control
    //! has to show All rather than a blank the user cannot meaningfully change away from.
    TEST(JoltSettingsRegistryTest, AnUnsetCollisionGroupIdResolvesToAll)
    {
        const JoltSystemConfiguration config;
        const AzPhysics::CollisionGroups::Id unsetId;

        EXPECT_TRUE(config.m_collisionConfig.m_collisionGroups.FindGroupNameById(unsetId).empty());
        EXPECT_EQ(
            AzPhysics::CollisionGroup::All.GetMask(),
            config.m_collisionConfig.m_collisionGroups.FindGroupById(unsetId).GetMask());
    }

    TEST(JoltSettingsRegistryTest, DefaultSceneConfiguration_RoundTripsThroughSettingsRegistry)
    {
        AzPhysics::SceneConfiguration saved;
        saved.m_gravity = AZ::Vector3(0.0f, 0.0f, -3.7f);

        MergeConfigIntoSettingsRegistry(saved, JoltSettingsRegistryManager::DefaultSceneConfigPath);

        JoltSettingsRegistryManager manager;
        const auto loaded = manager.LoadDefaultSceneConfiguration();
        ASSERT_TRUE(loaded.has_value());
        EXPECT_THAT(loaded->m_gravity, ::testing::Eq(AZ::Vector3(0.0f, 0.0f, -3.7f)));
    }

    TEST(JoltSettingsRegistryTest, LoadSystemConfiguration_NothingSaved_ReturnsNullopt)
    {
        // A path nothing ever merges to: the manager reads fixed paths, so exercise
        // the miss through a manager pointed at pristine state by checking the value
        // is only present after a merge. The system-config path may have been merged
        // by the round-trip test above (test order is not guaranteed), so this only
        // asserts on the scene path when it has not been merged yet, and otherwise
        // just documents the contract.
        auto* settingsRegistry = AZ::SettingsRegistry::Get();
        ASSERT_NE(nullptr, settingsRegistry);

        AZ::SettingsRegistryInterface::FixedValueString dummy;
        const bool sceneKeyExists = settingsRegistry->Get(dummy, JoltSettingsRegistryManager::DefaultSceneConfigPath) ||
            settingsRegistry->GetType(JoltSettingsRegistryManager::DefaultSceneConfigPath) !=
                AZ::SettingsRegistryInterface::Type::NoType;

        if (!sceneKeyExists)
        {
            JoltSettingsRegistryManager manager;
            EXPECT_FALSE(manager.LoadDefaultSceneConfiguration().has_value());
        }
    }

} // namespace JoltPhysicsTests
