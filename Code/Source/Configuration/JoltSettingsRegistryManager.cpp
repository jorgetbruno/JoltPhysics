#include <Configuration/JoltSettingsRegistryManager.h>

#include <AzCore/Settings/SettingsRegistry.h>

namespace JoltPhysics
{
    AZStd::optional<JoltSystemConfiguration> JoltSettingsRegistryManager::LoadSystemConfiguration() const
    {
        if (auto* settingsRegistry = AZ::SettingsRegistry::Get())
        {
            JoltSystemConfiguration configuration;
            if (settingsRegistry->GetObject(configuration, SystemConfigPath))
            {
                return configuration;
            }
        }
        return AZStd::nullopt;
    }

    AZStd::optional<AzPhysics::SceneConfiguration> JoltSettingsRegistryManager::LoadDefaultSceneConfiguration() const
    {
        if (auto* settingsRegistry = AZ::SettingsRegistry::Get())
        {
            AzPhysics::SceneConfiguration configuration;
            if (settingsRegistry->GetObject(configuration, DefaultSceneConfigPath))
            {
                return configuration;
            }
        }
        return AZStd::nullopt;
    }

    bool JoltSettingsRegistryManager::SaveConfiguration(
        [[maybe_unused]] const JoltSystemConfiguration& systemConfiguration,
        [[maybe_unused]] const AzPhysics::SceneConfiguration& defaultSceneConfiguration) const
    {
        return false;
    }

} // namespace JoltPhysics
