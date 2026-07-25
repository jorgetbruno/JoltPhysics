#pragma once

#include <AzCore/std/optional.h>

#include <AzFramework/Physics/Configuration/SceneConfiguration.h>

#include <JoltPhysics/Configuration/JoltConfiguration.h>

namespace JoltPhysics
{
    //! Loads the gem's configuration from the settings registry and, in editor builds,
    //! persists it back to the project.
    //!
    //! Project .setreg files are merged into the global settings registry during
    //! application boot, before system components activate, so the Load functions see
    //! the project's saved configuration in the Editor, launchers and servers alike.
    //! Saving is editor-only: the runtime base class has nowhere to persist to, so
    //! SaveConfiguration reports failure and JoltEditorSettingsRegistryManager
    //! overrides it to write the project .setreg file.
    class JoltSettingsRegistryManager
    {
    public:
        JoltSettingsRegistryManager() = default;
        virtual ~JoltSettingsRegistryManager() = default;

        //! Returns the saved system configuration (Jolt tuning values plus collision
        //! layers and groups), or nullopt when the project has never saved one.
        AZStd::optional<JoltSystemConfiguration> LoadSystemConfiguration() const;

        //! Returns the saved default scene configuration (gravity etc.), or nullopt
        //! when the project has never saved one.
        AZStd::optional<AzPhysics::SceneConfiguration> LoadDefaultSceneConfiguration() const;

        //! Persists both configurations. Returns false where persistence is
        //! unavailable (runtime builds).
        virtual bool SaveConfiguration(
            const JoltSystemConfiguration& systemConfiguration,
            const AzPhysics::SceneConfiguration& defaultSceneConfiguration) const;

        static constexpr const char* SettingsRegistryPath = "/O3DE/Physics/JoltPhysics";
        static constexpr const char* SystemConfigPath = "/O3DE/Physics/JoltPhysics/SystemConfiguration";
        static constexpr const char* DefaultSceneConfigPath = "/O3DE/Physics/JoltPhysics/DefaultSceneConfiguration";
    };

} // namespace JoltPhysics
