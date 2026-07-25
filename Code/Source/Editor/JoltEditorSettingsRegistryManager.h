#pragma once

#include <Configuration/JoltSettingsRegistryManager.h>

namespace JoltPhysics
{
    //! Editor-side persistence for the gem's configuration: writes
    //! <project>/Registry/joltphysicsconfiguration.setreg, which the settings registry
    //! merges at boot in every build flavor (Editor, launchers, servers).
    class JoltEditorSettingsRegistryManager : public JoltSettingsRegistryManager
    {
    public:
        JoltEditorSettingsRegistryManager() = default;
        ~JoltEditorSettingsRegistryManager() override = default;

        //! Writes the .setreg file and merges the same document into the live
        //! settings registry, so loads within this session see the new values.
        bool SaveConfiguration(
            const JoltSystemConfiguration& systemConfiguration,
            const AzPhysics::SceneConfiguration& defaultSceneConfiguration) const override;
    };

} // namespace JoltPhysics
