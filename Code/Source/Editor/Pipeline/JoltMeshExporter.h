#pragma once

#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/optional.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Physics/Material/PhysicsMaterialSlots.h>
#include <SceneAPI/SceneCore/Components/ExportingComponent.h>

namespace AZ
{
    namespace SceneAPI
    {
        namespace Events
        {
            class ExportEventContext;
        }

        namespace Containers
        {
            class Scene;
            class SceneGraph;
        }
    }
}

namespace JoltPhysics::Pipeline
{
    class JoltMeshGroup;

    //! Scene Builder exporting component that cooks every JoltMeshGroup in a scene
    //! manifest into a .joltmesh product. It is discovered purely through reflection:
    //! the Scene Builder instantiates every reflected ExportingComponent and calls it
    //! during export, which is why no explicit registration with the builder is needed.
    class JoltMeshExporter
        : public AZ::SceneAPI::SceneCore::ExportingComponent
    {
    public:
        AZ_COMPONENT(JoltMeshExporter, "{9A3F3974-B132-410B-AED2-FC8AFE76366A}", AZ::SceneAPI::SceneCore::ExportingComponent);

        JoltMeshExporter();
        ~JoltMeshExporter() override = default;

        static void Reflect(AZ::ReflectContext* context);

        AZ::SceneAPI::Events::ProcessingResult ProcessContext(AZ::SceneAPI::Events::ExportEventContext& context) const;
    };

    namespace Utils
    {
        //! A struct to store the materials of the mesh nodes selected in a mesh group.
        struct AssetMaterialsData
        {
            //! Material names coming from the source scene file.
            AZStd::vector<AZStd::string> m_sourceSceneMaterialNames;

            //! Look-up table for sourceSceneMaterialNames.
            AZStd::unordered_map<AZStd::string, size_t> m_materialIndexByName;

            //! Map of mesh nodes to their list of material indices associated to each face.
            AZStd::unordered_map<AZStd::string, AZStd::vector<AZ::u16>> m_nodesToPerFaceMaterialIndices;
        };

        //! Returns the list of materials assigned to the triangles
        //! of the mesh nodes selected in a mesh group.
        AZStd::optional<AssetMaterialsData> GatherMaterialsFromMeshGroup(
            const JoltMeshGroup& meshGroup,
            const AZ::SceneAPI::Containers::SceneGraph& sceneGraph);

        //! Function to update a list of physics material slots from a new list.
        //! All those new materials not found in the previous list will fallback to default physics material.
        void UpdateAssetPhysicsMaterials(
            const AZStd::vector<AZStd::string>& newMaterials,
            Physics::MaterialSlots& physicsMaterialSlots);
    } // namespace Utils

} // namespace JoltPhysics::Pipeline
