#include <Configuration/JoltSettingsRegistryManager.h>

#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/Console/ILogger.h>

namespace JoltPhysics
{
    void JoltSettingsRegistryManager::LoadSettings()
    {
        if (auto* settingsRegistry = AZ::SettingsRegistry::Get())
        {
            AZLOG_INFO("JoltPhysics: Loading settings from registry");
        }
    }

    void JoltSettingsRegistryManager::SaveSettings()
    {
        if (auto* settingsRegistry = AZ::SettingsRegistry::Get())
        {
            AZLOG_INFO("JoltPhysics: Saving settings to registry");
        }
    }

} // namespace JoltPhysics
