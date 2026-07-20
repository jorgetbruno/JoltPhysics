#pragma once

#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/std/string/string.h>

namespace JoltPhysics
{
    class JoltSettingsRegistryManager
    {
    public:
        JoltSettingsRegistryManager() = default;
        virtual ~JoltSettingsRegistryManager() = default;

        virtual void LoadSettings();
        virtual void SaveSettings();

        static constexpr const char* SettingsRegistryPath = "/O3DE/Physics/JoltPhysics";
        static constexpr const char* DefaultConfigPath = "/O3DE/Physics/JoltPhysics/DefaultConfig";
    };

} // namespace JoltPhysics
