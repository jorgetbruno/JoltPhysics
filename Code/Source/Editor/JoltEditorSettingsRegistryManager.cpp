#include <Editor/JoltEditorSettingsRegistryManager.h>

#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/Console/ILogger.h>

namespace JoltPhysics
{
    void JoltEditorSettingsRegistryManager::LoadSettings()
    {
        JoltSettingsRegistryManager::LoadSettings();
        AZLOG_INFO("JoltPhysics: Loading editor-specific settings");
    }

    void JoltEditorSettingsRegistryManager::SaveSettings()
    {
        JoltSettingsRegistryManager::SaveSettings();
        AZLOG_INFO("JoltPhysics: Saving editor-specific settings");
    }

} // namespace JoltPhysics
