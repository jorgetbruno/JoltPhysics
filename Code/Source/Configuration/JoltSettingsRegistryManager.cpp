#include <Configuration/JoltSettingsRegistryManager.h>

#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/std/containers/unordered_map.h>

namespace JoltPhysics
{
    namespace
    {
        //! Drops presets that share an id, keeping the last of each.
        //!
        //! GetObject deserializes into an existing object, and the JSON serializer
        //! appends to a vector rather than replacing it. The constructor seeds All and
        //! None, so a registry that also holds them (which it will, since they are
        //! saved back out) arrives appended to the seeded pair and the list grows by two
        //! on every round trip. Keeping the last occurrence lets a saved preset win over
        //! the seeded one; order follows first appearance so the list stays stable.
        void RemoveDuplicatePresets(AzPhysics::CollisionGroups& groups)
        {
            // Held by reference throughout: copying the preset vector would need
            // Preset::operator delete, which AzFramework declares but does not export.
            const AZStd::vector<AzPhysics::CollisionGroups::Preset>& presets = groups.GetPresets();

            AZStd::unordered_map<AZ::Uuid, size_t> lastIndexById;
            AZStd::vector<AZ::Uuid> order;
            for (size_t i = 0; i < presets.size(); ++i)
            {
                const AZ::Uuid& id = presets[i].m_id.m_id;
                if (lastIndexById.find(id) == lastIndexById.end())
                {
                    order.push_back(id);
                }
                lastIndexById[id] = i;
            }

            if (order.size() == presets.size())
            {
                return;
            }

            AzPhysics::CollisionGroups deduplicated;
            for (const AZ::Uuid& id : order)
            {
                const AzPhysics::CollisionGroups::Preset& preset = presets[lastIndexById[id]];
                deduplicated.CreateGroup(preset.m_name, preset.m_group, preset.m_id, preset.m_readOnly);
            }
            groups = deduplicated;
        }
    } // namespace

    AZStd::optional<JoltSystemConfiguration> JoltSettingsRegistryManager::LoadSystemConfiguration() const
    {
        if (auto* settingsRegistry = AZ::SettingsRegistry::Get())
        {
            JoltSystemConfiguration configuration;
            if (settingsRegistry->GetObject(configuration, SystemConfigPath))
            {
                RemoveDuplicatePresets(configuration.m_collisionConfig.m_collisionGroups);
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
