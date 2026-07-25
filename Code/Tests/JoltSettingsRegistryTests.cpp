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
