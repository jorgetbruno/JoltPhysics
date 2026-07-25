#include <Editor/JoltEditorSettingsRegistryManager.h>

#include <AzCore/IO/SystemFile.h>
#include <AzCore/JSON/document.h>
#include <AzCore/JSON/pointer.h>
#include <AzCore/JSON/prettywriter.h>
#include <AzCore/JSON/stringbuffer.h>
#include <AzCore/Serialization/Json/JsonSerialization.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/StringFunc/StringFunc.h>
#include <AzCore/Utils/Utils.h>

namespace JoltPhysics
{
    bool JoltEditorSettingsRegistryManager::SaveConfiguration(
        const JoltSystemConfiguration& systemConfiguration,
        const AzPhysics::SceneConfiguration& defaultSceneConfiguration) const
    {
        rapidjson::Document document(rapidjson::kObjectType);

        // Keep default values in the output. The document is merged into the live
        // registry as a patch, so an omitted field would not overwrite the previously
        // merged value and a setting changed back to its default would appear not to
        // stick until the next Editor restart.
        AZ::JsonSerializerSettings serializerSettings;
        serializerSettings.m_keepDefaults = true;

        rapidjson::Value systemValue;
        AZ::JsonSerializationResult::ResultCode result = AZ::JsonSerialization::Store(
            systemValue, document.GetAllocator(), systemConfiguration, serializerSettings);
        if (result.GetProcessing() == AZ::JsonSerializationResult::Processing::Halted)
        {
            AZ_Error("JoltPhysics", false, "Failed to serialize the Jolt system configuration: %s",
                result.ToString("").c_str());
            return false;
        }

        rapidjson::Value sceneValue;
        result = AZ::JsonSerialization::Store(
            sceneValue, document.GetAllocator(), defaultSceneConfiguration, serializerSettings);
        if (result.GetProcessing() == AZ::JsonSerializationResult::Processing::Halted)
        {
            AZ_Error("JoltPhysics", false, "Failed to serialize the default scene configuration: %s",
                result.ToString("").c_str());
            return false;
        }

        rapidjson::Pointer(SystemConfigPath).Set(document, systemValue, document.GetAllocator());
        rapidjson::Pointer(DefaultSceneConfigPath).Set(document, sceneValue, document.GetAllocator());

        rapidjson::StringBuffer buffer;
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
        document.Accept(writer);

        AZ::IO::FixedMaxPath setregPath = AZ::Utils::GetProjectPath();
        setregPath /= "Registry";
        AZ::IO::SystemFile::CreateDir(setregPath.c_str());
        setregPath /= "joltphysicsconfiguration.setreg";

        AZ::IO::SystemFile file;
        if (!file.Open(setregPath.c_str(),
                AZ::IO::SystemFile::SF_OPEN_CREATE |
                AZ::IO::SystemFile::SF_OPEN_CREATE_PATH |
                AZ::IO::SystemFile::SF_OPEN_WRITE_ONLY))
        {
            AZ_Error("JoltPhysics", false, "Failed to open '%s' for writing", setregPath.c_str());
            return false;
        }

        const AZ::IO::SystemFile::SizeType bytesWritten = file.Write(buffer.GetString(), buffer.GetSize());
        file.Close();
        if (bytesWritten != buffer.GetSize())
        {
            AZ_Error("JoltPhysics", false, "Failed to write '%s'", setregPath.c_str());
            return false;
        }

        // Merge into the live registry so LoadSystemConfiguration reflects the save
        // immediately; the file alone would only be seen on the next boot.
        if (auto* settingsRegistry = AZ::SettingsRegistry::Get())
        {
            settingsRegistry->MergeSettings(
                AZStd::string_view(buffer.GetString(), buffer.GetSize()),
                AZ::SettingsRegistryInterface::Format::JsonMergePatch);
        }

        return true;
    }

} // namespace JoltPhysics
