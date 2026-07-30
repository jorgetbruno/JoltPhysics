#pragma once

#include <SceneAPI/SceneCore/Components/BehaviorComponent.h>
#include <SceneAPI/SceneCore/Events/AssetImportRequest.h>
#include <SceneAPI/SceneCore/Events/ManifestMetaInfoBus.h>

namespace JoltPhysics::Pipeline
{
    //! Scene Builder behavior that owns the manifest side of the Jolt mesh
    //! pipeline: it puts the "Jolt Physics" category on the Scene Settings tab,
    //! offers the modifiers a JoltMeshGroup accepts, and creates/updates the
    //! default group whenever a scene contains mesh data. The actual cooking
    //! happens in JoltMeshExporter; this component only shapes the manifest.
    class JoltMeshBehavior
        : public AZ::SceneAPI::SceneCore::BehaviorComponent
        , private AZ::SceneAPI::Events::ManifestMetaInfoBus::Handler
        , private AZ::SceneAPI::Events::AssetImportRequestBus::Handler
    {
    public:
        AZ_COMPONENT(JoltMeshBehavior, "{3CAE04C1-06C8-4789-AAB7-999BF5556261}", AZ::SceneAPI::SceneCore::BehaviorComponent);
        static void Reflect(AZ::ReflectContext* context);

        ~JoltMeshBehavior() override = default;

        // BehaviorComponent overrides ...
        void Activate() override;
        void Deactivate() override;

        // ManifestMetaInfoBus overrides ...
        void GetCategoryAssignments(CategoryRegistrationList& categories, const AZ::SceneAPI::Containers::Scene& scene) override;
        void GetAvailableModifiers(
            AZ::SceneAPI::Events::ManifestMetaInfo::ModifiersList& modifiers,
            const AZ::SceneAPI::Containers::Scene& scene,
            const AZ::SceneAPI::DataTypes::IManifestObject& target) override;
        void InitializeObject(const AZ::SceneAPI::Containers::Scene& scene, AZ::SceneAPI::DataTypes::IManifestObject& target) override;

        // AssetImportRequestBus overrides ...
        AZ::SceneAPI::Events::ProcessingResult UpdateManifest(AZ::SceneAPI::Containers::Scene& scene, ManifestAction action,
            RequestingApplication requester) override;
        void GetPolicyName(AZStd::string& result) const override
        {
            result = "JoltPhysics::Pipeline::JoltMeshBehavior";
        }

    private:
        AZ::SceneAPI::Events::ProcessingResult BuildDefault(AZ::SceneAPI::Containers::Scene& scene) const;
        AZ::SceneAPI::Events::ProcessingResult UpdateJoltMeshGroups(AZ::SceneAPI::Containers::Scene& scene) const;

        static constexpr int s_meshBehaviorPreferredTabOrder{ 5 };
    };

} // namespace JoltPhysics::Pipeline
