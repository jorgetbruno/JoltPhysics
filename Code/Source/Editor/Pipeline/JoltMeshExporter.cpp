#include <Editor/Pipeline/JoltMeshExporter.h>

#include <Editor/EditorJoltConvexDecomposition.h>
#include <Editor/Pipeline/JoltMeshGroup.h>
#include <Editor/Pipeline/JoltPrimitiveShapeFitter.h>
#include <Pipeline/JoltMeshAsset.h>
#include <Shape/JoltMeshUtils.h>

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/Utils.h>
#include <AzCore/std/algorithm.h>
#include <AzCore/std/containers/set.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/StringFunc/StringFunc.h>
#include <AzToolsFramework/Debug/TraceContext.h>
#include <SceneAPI/SceneCore/Containers/Scene.h>
#include <SceneAPI/SceneCore/Containers/SceneGraph.h>
#include <SceneAPI/SceneCore/Containers/Utilities/Filters.h>
#include <SceneAPI/SceneCore/Containers/Utilities/SceneUtilities.h>
#include <SceneAPI/SceneCore/Containers/Views/SceneGraphChildIterator.h>
#include <SceneAPI/SceneCore/DataTypes/GraphData/IMaterialData.h>
#include <SceneAPI/SceneCore/DataTypes/GraphData/IMeshData.h>
#include <SceneAPI/SceneCore/Events/ExportEventContext.h>
#include <SceneAPI/SceneCore/Events/ExportProductList.h>
#include <SceneAPI/SceneCore/Events/ProcessingResult.h>
#include <SceneAPI/SceneCore/Utilities/CoordinateSystemConverter.h>
#include <SceneAPI/SceneCore/Utilities/FileUtilities.h>
#include <SceneAPI/SceneCore/Utilities/Reporting.h>
#include <SceneAPI/SceneData/Rules/CoordinateSystemRule.h>

namespace JoltPhysics::Pipeline
{
    namespace SceneContainers = AZ::SceneAPI::Containers;
    namespace SceneEvents = AZ::SceneAPI::Events;
    namespace SceneUtil = AZ::SceneAPI::Utilities;

    namespace
    {
        constexpr const char* const DefaultMaterialName = "default";
        constexpr const char* const JoltMeshAssetFileExtension = "joltmesh";

        // Geometry of a single exported part. One of these maps to one entry in
        // JoltMeshAssetData::m_colliderShapes. With decomposition enabled a single
        // scene node produces several of them (one per hull), each named "<node>_<i>".
        struct NodeCollisionGeomExportData
        {
            AZStd::vector<AZ::Vector3> m_vertices;
            AZStd::vector<AZ::u32> m_indices;
            AZStd::vector<AZ::u16> m_perFaceMaterialIndices;
            AZStd::string m_nodeName;
        };
    } // namespace

    JoltMeshExporter::JoltMeshExporter()
    {
        BindToCall(&JoltMeshExporter::ProcessContext);
    }

    void JoltMeshExporter::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            // The Scene Builder folds every exporter's reflected class name and version
            // into the builder job's fingerprint, so bumping this version is the
            // mechanism that re-cooks .joltmesh products when the export logic changes.
            serializeContext->Class<JoltMeshExporter, AZ::SceneAPI::SceneCore::ExportingComponent>()
                ->Version(2); // v2: per-face trimesh materials (blob v2) + Primitive export mode
        }
    }

    namespace Utils
    {
        // Utility function doing look-up in sourceSceneMaterialNames and inserting the name if it's not found.
        AZ::u16 InsertMaterialIndexByName(const AZStd::string& materialName, AssetMaterialsData& materials)
        {
            AZStd::vector<AZStd::string>& sourceSceneMaterialNames = materials.m_sourceSceneMaterialNames;
            AZStd::unordered_map<AZStd::string, size_t>& materialIndexByName = materials.m_materialIndexByName;

            // Check if we have this material in the list.
            auto materialIndexIter = materialIndexByName.find(materialName);

            if (materialIndexIter != materialIndexByName.end())
            {
                return static_cast<AZ::u16>(materialIndexIter->second);
            }

            // Add it to the list otherwise.
            sourceSceneMaterialNames.push_back(materialName);

            AZ::u16 newIndex = static_cast<AZ::u16>(sourceSceneMaterialNames.size() - 1);
            materialIndexByName[materialName] = newIndex;

            return newIndex;
        }

        void UpdateAssetPhysicsMaterials(
            const AZStd::vector<AZStd::string>& newMaterials,
            Physics::MaterialSlots& physicsMaterialSlots)
        {
            Physics::MaterialSlots newSlots;
            newSlots.SetSlots(newMaterials);

            // The new material list could have different names or be in a different order,
            // because they are obtained from the current mesh nodes selected.
            // Go through the previous slots and keep the same physics material
            // association if the slot name is the same.
            // Example:
            //
            //     Previous Material Slots from MeshGroup:
            //        Material_A: glass.physicsmaterial
            //        Material_B: sand.physicsmaterial
            //        Material_C: gold.physicsmaterial
            //
            //     Materials now extracted from mesh nodes selected:
            //        Material_C
            //        Material_A
            //
            //     New Material Slots have to keep the same physics materials association:
            //        Material_C: gold.physicsmaterial
            //        Material_A: glass.physicsmaterial
            //
            for (size_t newSlotId = 0; newSlotId < newSlots.GetSlotsCount(); ++newSlotId)
            {
                for (size_t prevSlotId = 0; prevSlotId < physicsMaterialSlots.GetSlotsCount(); ++prevSlotId)
                {
                    if (AZ::StringFunc::Equal(physicsMaterialSlots.GetSlotName(prevSlotId), newSlots.GetSlotName(newSlotId), false/*bCaseSensitive*/))
                    {
                        const auto materialAsset = physicsMaterialSlots.GetMaterialAsset(prevSlotId);
                        // Note: Material asset is also valid if it's got a path hint (which will be resolved
                        // by the JoltMeshAssetHandler after loading the JoltMeshAsset).
                        if (materialAsset.GetId().IsValid() || !materialAsset.GetHint().empty())
                        {
                            newSlots.SetMaterialAsset(newSlotId, materialAsset);
                        }
                        break;
                    }
                }
            }

            // The material slots data come from JoltMeshGroup. A JoltMeshGroup created from the
            // Scene Settings UI will always generate Material Slots with a valid name in them
            // (extracted from the mesh). But when JoltMeshGroup data is generated procedurally
            // it's not possible to know what's the material name from the mesh nodes. In order
            // to cover this case, when the slot name is default or empty the physics materials
            // assigned will still be used in the new slots.
            // Example:
            //
            //     Previous Material Slots from MeshGroup:
            //        "": glass.physicsmaterial
            //
            //     Materials now extracted from mesh nodes selected:
            //        Material_C
            //        Material_A
            //
            //     New Material Slots will keep physics materials from empty slot names
            //        Material_C: glass.physicsmaterial
            //        Material_A:
            //
            for (size_t slotId = 0; slotId < physicsMaterialSlots.GetSlotsCount(); ++slotId)
            {
                if (physicsMaterialSlots.GetSlotName(slotId).empty() ||
                    physicsMaterialSlots.GetSlotName(slotId) == Physics::MaterialSlots::EntireObjectSlotName)
                {
                    const auto materialAsset = physicsMaterialSlots.GetMaterialAsset(slotId);
                    // Note: Material asset is also valid if it's got a path hint (which will be resolved
                    // by the JoltMeshAssetHandler after loading the JoltMeshAsset).
                    if (materialAsset.GetId().IsValid() || !materialAsset.GetHint().empty())
                    {
                        newSlots.SetMaterialAsset(slotId, materialAsset);
                    }
                }
            }

            physicsMaterialSlots = AZStd::move(newSlots);
        }

        AZStd::vector<AZStd::string> GenerateLocalNodeMaterialMap(const AZ::SceneAPI::Containers::SceneGraph& graph, const AZ::SceneAPI::Containers::SceneGraph::NodeIndex& nodeIndex)
        {
            AZStd::vector<AZStd::string> materialNames;

            auto view = AZ::SceneAPI::Containers::Views::MakeSceneGraphChildView<AZ::SceneAPI::Containers::Views::AcceptEndPointsOnly>(
                graph,
                nodeIndex,
                graph.GetContentStorage().begin(),
                true
            );

            for (auto it = view.begin(), itEnd = view.end(); it != itEnd; ++it)
            {
                if ((*it) && (*it)->RTTI_IsTypeOf(AZ::SceneAPI::DataTypes::IMaterialData::TYPEINFO_Uuid()))
                {
                    AZStd::string nodeName = graph.GetNodeName(graph.ConvertToNodeIndex(it.GetHierarchyIterator())).GetName();
                    materialNames.push_back(nodeName);
                }
            }

            return materialNames;
        }

        AZStd::optional<AssetMaterialsData> GatherMaterialsFromMeshGroup(
            const JoltMeshGroup& meshGroup,
            const AZ::SceneAPI::Containers::SceneGraph& sceneGraph)
        {
            AssetMaterialsData assetMaterialData;
            bool errorFound = false;

            const auto& sceneNodeSelectionList = meshGroup.GetSceneNodeSelectionList();
            sceneNodeSelectionList.EnumerateSelectedNodes(
                [&](const AZStd::string& name)
                {
                    AZ::SceneAPI::Containers::SceneGraph::NodeIndex nodeIndex = sceneGraph.Find(name);
                    if (!nodeIndex.IsValid())
                    {
                        AZ_TracePrintf(
                            AZ::SceneAPI::Utilities::WarningWindow,
                            "Node '%s' was not found in the scene graph.",
                            name.c_str());
                        return true;
                    }
                    auto nodeMesh =
                        azrtti_cast<const AZ::SceneAPI::DataTypes::IMeshData*>(*sceneGraph.ConvertToStorageIterator(nodeIndex));
                    if (!nodeMesh)
                    {
                        return true;
                    }

                    AZStd::string_view nodeName = sceneGraph.GetNodeName(nodeIndex).GetName();

                    const AZStd::vector<AZStd::string> localSourceSceneMaterialsList =
                        GenerateLocalNodeMaterialMap(sceneGraph, nodeIndex);
                    if (localSourceSceneMaterialsList.empty())
                    {
                        AZ_TracePrintf(
                            AZ::SceneAPI::Utilities::WarningWindow,
                            "Node '%.*s' does not have any material assigned to it. Material '%s' will be used.",
                            AZ_STRING_ARG(nodeName),
                            DefaultMaterialName);
                    }

                    const AZ::u32 faceCount = nodeMesh->GetFaceCount();

                    assetMaterialData.m_nodesToPerFaceMaterialIndices.emplace(nodeName, AZStd::vector<AZ::u16>(faceCount));

                    // Convex shapes can only carry one material per node: a Jolt convex
                    // hull is cooked from a point cloud without face data, so there is
                    // nothing per-face left to map. Triangle meshes keep the per-face
                    // table (mirroring PhysX) so the material slot list reflects every
                    // material in use, even though the exporter currently only consumes
                    // the first one - per-face materials are a follow-up.
                    const bool limitToOneMaterial = meshGroup.GetExportAsConvex();
                    AZStd::string firstMaterial;
                    AZStd::set<AZStd::string> nodeMaterials;

                    for (AZ::u32 faceIndex = 0; faceIndex < faceCount; ++faceIndex)
                    {
                        AZStd::string materialName = DefaultMaterialName;
                        if (!localSourceSceneMaterialsList.empty())
                        {
                            const size_t materialId = nodeMesh->GetFaceMaterialId(faceIndex);
                            if (materialId >= localSourceSceneMaterialsList.size())
                            {
                                AZ_TracePrintf(
                                    AZ::SceneAPI::Utilities::ErrorWindow,
                                    "materialId %zu for face %d is out of bound for localSourceSceneMaterialsList (size %zu).",
                                    materialId,
                                    faceIndex,
                                    localSourceSceneMaterialsList.size());

                                errorFound = true;
                                return false;
                            }

                            materialName = localSourceSceneMaterialsList[materialId];

                            // Use the first material found in the mesh when it has to be limited to one.
                            if (limitToOneMaterial)
                            {
                                nodeMaterials.insert(materialName);
                                if (firstMaterial.empty())
                                {
                                    firstMaterial = materialName;
                                }
                                materialName = firstMaterial;
                            }
                        }

                        const AZ::u16 materialIndex = InsertMaterialIndexByName(materialName, assetMaterialData);
                        assetMaterialData.m_nodesToPerFaceMaterialIndices[nodeName][faceIndex] = materialIndex;
                    }

                    if (limitToOneMaterial && nodeMaterials.size() > 1)
                    {
                        AZ_TracePrintf(
                            AZ::SceneAPI::Utilities::WarningWindow,
                            "Node '%s' has %zu materials, but cooking method Convex supports one material per node. The "
                            "first material '%s' will be used.",
                            name.c_str(),
                            nodeMaterials.size(),
                            firstMaterial.c_str());
                    }

                    return true;
                });

            if (errorFound)
            {
                return AZStd::nullopt;
            }

            return assetMaterialData;
        }
    } // namespace Utils

    namespace
    {
        // Packs one part's geometry into the blob format that
        // Physics::CookedMeshShapeConfiguration expects. Jolt needs no offline cooking
        // pass (the runtime builds the mesh BVH / convex hull when the shape is
        // created), so "cooking" here is only packing; an empty blob signals failure.
        AZStd::vector<AZ::u8> CookJoltMesh(const NodeCollisionGeomExportData& geometry, const JoltMeshGroup& meshGroup)
        {
            if (meshGroup.GetExportAsTriMesh())
            {
                // The per-face table feeds the blob's material indices (u8; Jolt packs
                // 5 bits per triangle, so anything past 31 falls to the last slot).
                AZStd::vector<AZ::u8> materialIndices;
                materialIndices.reserve(geometry.m_perFaceMaterialIndices.size());
                for (const AZ::u16 materialIndex : geometry.m_perFaceMaterialIndices)
                {
                    materialIndices.push_back(static_cast<AZ::u8>(AZStd::min<AZ::u16>(materialIndex, 31)));
                }
                return JoltMeshUtils::PackTriangleMesh(
                    geometry.m_vertices.data(), static_cast<AZ::u32>(geometry.m_vertices.size()),
                    geometry.m_indices.data(), static_cast<AZ::u32>(geometry.m_indices.size()),
                    materialIndices.data(), static_cast<AZ::u32>(materialIndices.size()));
            }

            if (meshGroup.GetDecomposeMeshes())
            {
                // Each decomposed part already holds one convex hull point cloud, but it
                // is wrapped in the hull-group blob so the runtime decodes decomposed and
                // plain convex data through the same format.
                return JoltMeshUtils::PackConvexHulls({ geometry.m_vertices });
            }

            return JoltMeshUtils::PackConvexMesh(geometry.m_vertices.data(), static_cast<AZ::u32>(geometry.m_vertices.size()));
        }

        // Runs V-HACD over one node's geometry and appends one export entry per hull.
        // One entry per hull (instead of one blob holding all hulls) mirrors how PhysX
        // exports decomposition parts: every part becomes its own shape with its own
        // name and material slot, which keeps the asset shape list aligned with what
        // the collider components display.
        SceneEvents::ProcessingResult DecomposeAndAppendMeshes(
            const JoltConvexDecompositionParams& decompositionParams,
            AZStd::vector<NodeCollisionGeomExportData>& totalExportData,
            const NodeCollisionGeomExportData& nodeExportData)
        {
            AZ_Assert(
                !nodeExportData.m_perFaceMaterialIndices.empty(),
                "DecomposeAndAppendMeshes: Empty per-face material vector. Node: %s",
                nodeExportData.m_nodeName.c_str());

            EditorConvexDecomposition::DecompositionParams params;
            params.m_maxHulls = decompositionParams.GetMaxConvexHulls();
            params.m_voxelResolution = decompositionParams.GetResolution();
            params.m_maxVerticesPerHull = decompositionParams.GetMaxNumVerticesPerConvexHull();
            params.m_concavity = decompositionParams.GetConcavity();

            const EditorConvexDecomposition::DecompositionResult decomposition =
                EditorConvexDecomposition::DecomposeToHullPointClouds(nodeExportData.m_vertices, nodeExportData.m_indices, params);

            if (!decomposition.Succeeded())
            {
                AZ_TracePrintf(
                    AZ::SceneAPI::Utilities::ErrorWindow,
                    "Convex decomposition of node '%s' produced no hulls.",
                    nodeExportData.m_nodeName.c_str());
                return SceneEvents::ProcessingResult::Failure;
            }

            AZ_TracePrintf(
                AZ::SceneAPI::Utilities::LogWindow,
                "Convex decomposition of node '%s' returned %zu hulls",
                nodeExportData.m_nodeName.c_str(),
                decomposition.m_hulls.size());

            for (size_t hullIndex = 0; hullIndex < decomposition.m_hulls.size(); ++hullIndex)
            {
                NodeCollisionGeomExportData hullExportData;
                hullExportData.m_vertices = decomposition.m_hulls[hullIndex];

                // A hull has no faces of its own, but it inherits the node's material
                // so its shape entry maps to a material slot like any other.
                if (!nodeExportData.m_perFaceMaterialIndices.empty())
                {
                    hullExportData.m_perFaceMaterialIndices.push_back(nodeExportData.m_perFaceMaterialIndices[0]);
                }

                hullExportData.m_nodeName = nodeExportData.m_nodeName + "_" + AZStd::to_string(hullIndex);
                totalExportData.emplace_back(AZStd::move(hullExportData));
            }

            return SceneEvents::ProcessingResult::Success;
        }

        // Processes the collected geometry and writes it into a .joltmesh file.
        SceneEvents::ProcessingResult WriteJoltMeshAsset(
            SceneEvents::ExportEventContext& context,
            const AZStd::vector<NodeCollisionGeomExportData>& totalExportData,
            const Utils::AssetMaterialsData& assetMaterialsData,
            const JoltMeshGroup& meshGroup,
            bool mergedTriangleMesh [[maybe_unused]])
        {
            const AZStd::string& assetName = meshGroup.GetName();
            AZStd::string filename = SceneUtil::FileUtilities::CreateOutputFileName(
                assetName, context.GetOutputDirectory(), JoltMeshAssetFileExtension, AZStd::string(context.GetScene().GetSourceExtension()));

            // Jolt packs material indices as 5 bits per triangle, so a mesh can
            // address at most 32 material slots; excess faces clamp to slot 31.
            if (meshGroup.GetMaterialSlots().GetSlotsCount() > 32)
            {
                AZ_Warning("JoltPhysics", false,
                    "WriteJoltMeshAsset: group '%s' has more than 32 material slots; faces above slot 31 will use slot 31.",
                    assetName.c_str());
            }

            JoltMeshAssetData assetData;

            // Start from the material slots authored on the group, then remap them onto
            // the materials gathered from the source scene right now: this exporter runs
            // while the scene is being processed, and its content may have changed since
            // the group was last edited in the Scene Settings UI.
            assetData.m_materialSlots = meshGroup.GetMaterialSlots();
            Utils::UpdateAssetPhysicsMaterials(assetMaterialsData.m_sourceSceneMaterialNames, assetData.m_materialSlots);

            const Physics::CookedMeshShapeConfiguration::MeshType meshType = meshGroup.GetExportAsConvex()
                ? Physics::CookedMeshShapeConfiguration::MeshType::Convex
                : Physics::CookedMeshShapeConfiguration::MeshType::TriangleMesh;

            for (const NodeCollisionGeomExportData& subMesh : totalExportData)
            {
                AZStd::shared_ptr<Physics::ShapeConfiguration> shapeConfig;
                AZStd::shared_ptr<JoltAssetColliderConfiguration> colliderConfig =
                    AZStd::make_shared<JoltAssetColliderConfiguration>();

                if (meshGroup.GetExportAsPrimitive())
                {
                    // Primitives are not packed geometry: the fit yields a shape
                    // configuration directly, with its transform stored on the entry's
                    // collider configuration (the collider components compose it).
                    const AZStd::optional<PrimitiveFitResult> fit =
                        FitPrimitiveToPoints(subMesh.m_vertices, meshGroup.GetPrimitiveTarget());
                    if (!fit)
                    {
                        AZ_TracePrintf(
                            AZ::SceneAPI::Utilities::ErrorWindow,
                            "WriteJoltMeshAsset: Failed to fit a primitive to mesh data. Node: %s",
                            subMesh.m_nodeName.c_str());
                        return SceneEvents::ProcessingResult::Failure;
                    }
                    shapeConfig = fit->m_shapeConfig;
                    colliderConfig->m_transform = fit->m_transform;
                }
                else
                {
                    AZStd::vector<AZ::u8> cookedData = CookJoltMesh(subMesh, meshGroup);
                    if (cookedData.empty())
                    {
                        AZ_TracePrintf(
                            AZ::SceneAPI::Utilities::ErrorWindow,
                            "WriteJoltMeshAsset: Failed to cook mesh data. Node: %s",
                            subMesh.m_nodeName.c_str());
                        return SceneEvents::ProcessingResult::Failure;
                    }

                    AZStd::shared_ptr<Physics::CookedMeshShapeConfiguration> cookedShapeConfig =
                        AZStd::make_shared<Physics::CookedMeshShapeConfiguration>();
                    cookedShapeConfig->SetCookedMeshData(cookedData.data(), cookedData.size(), meshType);
                    shapeConfig = AZStd::move(cookedShapeConfig);
                }

                // Default collider configuration with no overrides: the collider
                // components fill in their own settings when they instantiate the asset.
                assetData.m_colliderShapes.emplace_back(AZStd::move(colliderConfig), AZStd::move(shapeConfig));

                if (meshGroup.GetExportAsTriMesh())
                {
                    AZ_Assert(
                        !subMesh.m_perFaceMaterialIndices.empty(),
                        "WriteJoltMeshAsset: m_perFaceMaterialIndices must not be empty! Please make sure you have a material assigned to the geometry. Node: %s",
                        subMesh.m_nodeName.c_str());

                    // Trimesh entries carry the whole slot list and resolve each face's
                    // material from the table baked into the mesh itself; the sentinel
                    // tells the collider components to leave the slot list untouched.
                    assetData.m_materialIndexPerShape.push_back(JoltMeshAssetData::TriangleMeshMaterialIndex);
                }
                else
                {
                    AZ_Assert(
                        !subMesh.m_perFaceMaterialIndices.empty(),
                        "WriteJoltMeshAsset: m_perFaceMaterialIndices must not be empty! Please make sure you have a material assigned to the geometry. Node: %s",
                        subMesh.m_nodeName.c_str());

                    // Convex entries (hulls, decomposed parts) are single-material shapes:
                    // the node's first material becomes the shape's one slot.
                    const AZ::u16 materialIndex = !subMesh.m_perFaceMaterialIndices.empty()
                        ? subMesh.m_perFaceMaterialIndices[0]
                        : 0;
                    assetData.m_materialIndexPerShape.push_back(materialIndex);
                }
            }

            // Like PhysX, the on-disk format is the bare asset-data struct:
            // JoltMeshAssetHandler loads it back into JoltMeshAsset::m_assetData.
            AZ::SerializeContext* serializeContext = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(serializeContext, &AZ::ComponentApplicationRequests::GetSerializeContext);

            if (!AZ::Utils::SaveObjectToFile(filename, AZ::DataStream::ST_BINARY, &assetData, serializeContext))
            {
                AZ_TracePrintf(
                    AZ::SceneAPI::Utilities::ErrorWindow,
                    "Unable to write to a file for a Jolt mesh asset. AssetName: %s, filename: %s",
                    assetName.c_str(),
                    filename.c_str());
                return SceneEvents::ProcessingResult::Failure;
            }

            AZStd::string productUuidString = meshGroup.GetId().ToString<AZStd::string>();
            AZ::Uuid productUuid = AZ::Uuid::CreateName(productUuidString);

            auto& meshProduct = context.GetProductList().AddProduct(
                AZStd::move(filename), productUuid, azrtti_typeid<JoltMeshAsset>(), AZStd::nullopt, AZStd::nullopt);

            // Add product dependencies for every valid physics material used by this physics mesh.
            for (size_t materialIndex = 0; materialIndex < assetData.m_materialSlots.GetSlotsCount(); ++materialIndex)
            {
                const auto material = assetData.m_materialSlots.GetMaterialAsset(materialIndex);

                if (material.GetId().IsValid())
                {
                    AZ::SceneAPI::Events::ExportProduct materialProduct;
                    materialProduct.m_filename = material.GetHint();
                    materialProduct.m_id = material.GetId().m_guid;
                    materialProduct.m_subId = material.GetId().m_subId;
                    materialProduct.m_assetType = material.GetType();

                    meshProduct.m_productDependencies.push_back(materialProduct);
                }
            }

            return SceneEvents::ProcessingResult::Success;
        }
    } // namespace

    SceneEvents::ProcessingResult JoltMeshExporter::ProcessContext(SceneEvents::ExportEventContext& context) const
    {
        AZ_TraceContext("Exporter", "JoltPhysics");

        SceneEvents::ProcessingResultCombiner result;

        const AZ::SceneAPI::Containers::Scene& scene = context.GetScene();
        const AZ::SceneAPI::Containers::SceneGraph& graph = scene.GetGraph();

        const SceneContainers::SceneManifest& manifest = scene.GetManifest();

        SceneContainers::SceneManifest::ValueStorageConstData valueStorage = manifest.GetValueStorage();
        auto view = SceneContainers::MakeExactFilterView<JoltMeshGroup>(valueStorage);

        for (const JoltMeshGroup& joltMeshGroup : view)
        {
            // Gather material data from asset for the mesh group.
            AZStd::optional<Utils::AssetMaterialsData> assetMaterialData = Utils::GatherMaterialsFromMeshGroup(joltMeshGroup, graph);
            if (!assetMaterialData.has_value())
            {
                return SceneEvents::ProcessingResult::Failure;
            }

            // Export data per node.
            AZStd::vector<NodeCollisionGeomExportData> totalExportData;

            const AZStd::string& groupName = joltMeshGroup.GetName();

            AZ_TraceContext("Group Name", groupName);

            const auto& sceneNodeSelectionList = joltMeshGroup.GetSceneNodeSelectionList();

            totalExportData.reserve(sceneNodeSelectionList.GetSelectedNodeCount());

            // Get the coordinate system conversion rule.
            AZ::SceneAPI::CoordinateSystemConverter coordSysConverter;
            AZStd::shared_ptr<AZ::SceneAPI::SceneData::CoordinateSystemRule> coordinateSystemRule =
                joltMeshGroup.GetRuleContainerConst().FindFirstByType<AZ::SceneAPI::SceneData::CoordinateSystemRule>();
            if (coordinateSystemRule)
            {
                coordinateSystemRule->UpdateCoordinateSystemConverter();
                coordSysConverter = coordinateSystemRule->GetCoordinateSystemConverter();
            }

            SceneEvents::ProcessingResult enumerationResult = SceneEvents::ProcessingResult::Success;

            sceneNodeSelectionList.EnumerateSelectedNodes(
                [&](const AZStd::string& name)
                {
                    AZ::SceneAPI::Containers::SceneGraph::NodeIndex nodeIndex = graph.Find(name);
                    auto nodeMesh = azrtti_cast<const AZ::SceneAPI::DataTypes::IMeshData*>(*graph.ConvertToStorageIterator(nodeIndex));

                    if (!nodeMesh)
                    {
                        return true;
                    }

                    const AZ::SceneAPI::Containers::SceneGraph::Name& nodeName = graph.GetNodeName(nodeIndex);

                    // The cooked collider must line up with the render mesh, so the node's
                    // world transform and the group's coordinate-system conversion are baked
                    // straight into the vertices; the runtime shape then sits at identity in
                    // entity space instead of carrying its own offset.
                    // CoordinateSystemConverter covers the simple transformations of
                    // CoordinateSystemRule and DetermineWorldTransform covers the advanced mode.
                    const AZ::SceneAPI::DataTypes::MatrixType worldTransform = coordSysConverter.ConvertMatrix3x4(
                        SceneUtil::DetermineWorldTransform(scene, nodeIndex, joltMeshGroup.GetRuleContainerConst()));

                    NodeCollisionGeomExportData nodeExportData;
                    nodeExportData.m_nodeName = nodeName.GetName();

                    const AZ::u32 vertexCount = nodeMesh->GetVertexCount();
                    const AZ::u32 faceCount = nodeMesh->GetFaceCount();

                    nodeExportData.m_vertices.resize(vertexCount);

                    for (AZ::u32 vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex)
                    {
                        nodeExportData.m_vertices[vertexIndex] = worldTransform * nodeMesh->GetPosition(vertexIndex);
                    }

                    nodeExportData.m_indices.resize(faceCount * 3);

                    for (AZ::u32 faceIndex = 0; faceIndex < faceCount; ++faceIndex)
                    {
                        const AZ::SceneAPI::DataTypes::IMeshData::Face& face = nodeMesh->GetFaceInfo(faceIndex);
                        nodeExportData.m_indices[faceIndex * 3] = face.vertexIndex[0];
                        nodeExportData.m_indices[faceIndex * 3 + 1] = face.vertexIndex[1];
                        nodeExportData.m_indices[faceIndex * 3 + 2] = face.vertexIndex[2];
                    }

                    nodeExportData.m_perFaceMaterialIndices =
                        assetMaterialData->m_nodesToPerFaceMaterialIndices[nodeExportData.m_nodeName];
                    if (nodeExportData.m_perFaceMaterialIndices.size() != faceCount)
                    {
                        AZ_TracePrintf(
                            AZ::SceneAPI::Utilities::WarningWindow,
                            "Node '%s' material information face count %zu does not match the node's %d.",
                            nodeExportData.m_nodeName.c_str(),
                            nodeExportData.m_perFaceMaterialIndices.size(),
                            faceCount);
                        enumerationResult = SceneEvents::ProcessingResult::Failure;
                        return false;
                    }

                    if (joltMeshGroup.GetDecomposeMeshes())
                    {
                        if (DecomposeAndAppendMeshes(joltMeshGroup.GetConvexDecompositionParams(), totalExportData, nodeExportData)
                            == SceneEvents::ProcessingResult::Failure)
                        {
                            enumerationResult = SceneEvents::ProcessingResult::Failure;
                            return false;
                        }
                    }
                    else
                    {
                        totalExportData.emplace_back(AZStd::move(nodeExportData));
                    }

                    return true;
                });

            if (enumerationResult == SceneEvents::ProcessingResult::Failure)
            {
                return enumerationResult;
            }

            // Merge the selected nodes into one soup if requested and there is more
            // than one: a single static triangle mesh (or one fitted primitive) is the
            // common case for level geometry.
            const bool mergedForExport = (joltMeshGroup.GetExportAsTriMesh() || joltMeshGroup.GetExportAsPrimitive())
                && joltMeshGroup.GetTriangleMeshAssetParams().GetMergeMeshes()
                && totalExportData.size() > 1;
            if (mergedForExport)
            {
                NodeCollisionGeomExportData mergedData;
                mergedData.m_nodeName = groupName;

                AZStd::vector<AZ::Vector3>& mergedVertices = mergedData.m_vertices;
                AZStd::vector<AZ::u32>& mergedIndices = mergedData.m_indices;
                AZStd::vector<AZ::u16>& mergedPerFaceMaterials = mergedData.m_perFaceMaterialIndices;

                // Here we add the geometry data for each node into a single merged one.
                // Vertices & materials can be added directly but indices need to be
                // incremented by the amount of vertices already added in the last iteration.
                for (const NodeCollisionGeomExportData& exportData : totalExportData)
                {
                    AZ::u32 startingIndex = static_cast<AZ::u32>(mergedVertices.size());

                    mergedVertices.insert(mergedVertices.end(), exportData.m_vertices.begin(), exportData.m_vertices.end());

                    mergedPerFaceMaterials.insert(mergedPerFaceMaterials.end(),
                        exportData.m_perFaceMaterialIndices.begin(), exportData.m_perFaceMaterialIndices.end());

                    mergedIndices.reserve(mergedIndices.size() + exportData.m_indices.size());

                    AZStd::transform(exportData.m_indices.begin(), exportData.m_indices.end(),
                        AZStd::back_inserter(mergedIndices), [startingIndex](AZ::u32 index)
                    {
                        return index + startingIndex;
                    });
                }

                // Clear the data per node and use only the merged one.
                totalExportData.clear();
                totalExportData.emplace_back(AZStd::move(mergedData));
            }

            if (!totalExportData.empty())
            {
                result += WriteJoltMeshAsset(context, totalExportData, *assetMaterialData, joltMeshGroup, mergedForExport);
            }
        }

        return result.GetResult();
    }

} // namespace JoltPhysics::Pipeline
