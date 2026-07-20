#pragma once

#include <Configuration/JoltSettingsRegistryManager.h>

namespace JoltPhysics
{
    class JoltEditorSettingsRegistryManager : public JoltSettingsRegistryManager
    {
    public:
        JoltEditorSettingsRegistryManager() = default;
        ~JoltEditorSettingsRegistryManager() override = default;

        void LoadSettings() override;
        void SaveSettings() override;
    };

} // namespace JoltPhysics
