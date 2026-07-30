#include <Editor/Pipeline/JoltMeshGroup.h>
#include <Editor/Pipeline/JoltMeshExporter.h>

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzToolsFramework/UI/PropertyEditor/PropertyEditorAPI.h>
#include <SceneAPI/SceneCore/DataTypes/GraphData/IMeshData.h>

namespace JoltPhysics::Pipeline
{
    void JoltTriangleMeshAssetParams::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltTriangleMeshAssetParams>()->Version(1)
                ->Field("MergeMeshes", &JoltTriangleMeshAssetParams::m_mergeMeshes);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltTriangleMeshAssetParams>("Triangle Mesh Asset Parameters",
                    "Configure the parameters controlling the exported triangle mesh asset.")

                    ->DataElement(AZ_CRC_CE("MergeMeshes"), &JoltTriangleMeshAssetParams::m_mergeMeshes, "Merge Meshes",
                        "<span>When set, all selected nodes will be merged into a single collision mesh. Otherwise "
                        "they will be exported as separate shapes. Typically it is more efficient to have a single "
                        "mesh, however if you have game code handling specific shapes differently, you want to "
                        "avoid merging them together.</span>");
            }
        }
    }

    bool JoltTriangleMeshAssetParams::GetMergeMeshes() const
    {
        return m_mergeMeshes;
    }

    void JoltTriangleMeshAssetParams::SetMergeMeshes(bool mergeMeshes)
    {
        m_mergeMeshes = mergeMeshes;
    }


    void JoltConvexDecompositionParams::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltConvexDecompositionParams>()->Version(1)
                ->Field("MaxConvexHulls", &JoltConvexDecompositionParams::m_maxConvexHulls)
                ->Field("Resolution", &JoltConvexDecompositionParams::m_resolution)
                ->Field("MaxNumVerticesPerConvexHull", &JoltConvexDecompositionParams::m_maxNumVerticesPerConvexHull)
                ->Field("Concavity", &JoltConvexDecompositionParams::m_concavity);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltConvexDecompositionParams>("Decomposition Parameters",
                    "Configure the parameters controlling the approximate convex decomposition algorithm.")

                    ->DataElement(AZ_CRC_CE("MaxConvexHulls"), &JoltConvexDecompositionParams::m_maxConvexHulls, "Maximum Hulls",
                        "<span>Controls the maximum number of hulls to generate.</span>")
                        ->Attribute(AZ::Edit::Attributes::Min, 1)
                        ->Attribute(AZ::Edit::Attributes::Max, 1024)

                    ->DataElement(AZ_CRC_CE("MaxNumVerticesPerConvexHull"), &JoltConvexDecompositionParams::m_maxNumVerticesPerConvexHull, "Maximum Vertices Per Hull",
                        "<span>Controls the maximum number of vertices per convex hull. Jolt convex hulls support "
                        "at most 256 points.</span>")
                        ->Attribute(AZ::Edit::Attributes::Min, 4)
                        ->Attribute(AZ::Edit::Attributes::Max, 256)

                    ->DataElement(AZ_CRC_CE("Resolution"), &JoltConvexDecompositionParams::m_resolution, "Resolution",
                        "<span>Maximum number of voxels generated during the voxelization stage.</span>")
                        ->Attribute(AZ::Edit::Attributes::Min, 10000)
                        ->Attribute(AZ::Edit::Attributes::Max, 10000000)
                        ->Attribute(AZ::Edit::Attributes::Step, 10000)

                    ->DataElement(AZ_CRC_CE("Concavity"), &JoltConvexDecompositionParams::m_concavity, "Concavity",
                        "<span>Maximum concavity error accepted before a hull is split into smaller parts. "
                        "Lower values produce tighter-fitting but more numerous hulls.</span>")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0)
                        ->Attribute(AZ::Edit::Attributes::Max, 1.0)
                        ->Attribute(AZ::Edit::Attributes::Step, 0.001)
                        ->Attribute(AZ::Edit::Attributes::Decimals, 4)
                        ->Attribute(AZ::Edit::Attributes::DisplayDecimals, 4);
            }
        }
    }

    AZ::u32 JoltConvexDecompositionParams::GetMaxConvexHulls() const
    {
        return m_maxConvexHulls;
    }

    AZ::u32 JoltConvexDecompositionParams::GetResolution() const
    {
        return m_resolution;
    }

    AZ::u32 JoltConvexDecompositionParams::GetMaxNumVerticesPerConvexHull() const
    {
        return m_maxNumVerticesPerConvexHull;
    }

    double JoltConvexDecompositionParams::GetConcavity() const
    {
        return m_concavity;
    }


    AZ_CLASS_ALLOCATOR_IMPL(JoltMeshGroup, AZ::SystemAllocator)

    JoltMeshGroup::JoltMeshGroup()
        : m_id(AZ::Uuid::CreateRandom())
    {
    }

    JoltMeshGroup::~JoltMeshGroup()
    {
    }

    void JoltMeshGroup::Reflect(AZ::ReflectContext* context)
    {
        JoltTriangleMeshAssetParams::Reflect(context);
        JoltConvexDecompositionParams::Reflect(context);

        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltMeshGroup, AZ::SceneAPI::DataTypes::ISceneNodeGroup>()->Version(1)
                ->Field("id", &JoltMeshGroup::m_id)
                ->Field("name", &JoltMeshGroup::m_name)
                ->Field("NodeSelectionList", &JoltMeshGroup::m_nodeSelectionList)
                ->Field("export method", &JoltMeshGroup::m_exportMethod)
                ->Field("TriangleMeshAssetParams", &JoltMeshGroup::m_triangleMeshAssetParams)
                ->Field("DecomposeMeshes", &JoltMeshGroup::m_decomposeMeshes)
                ->Field("ConvexDecompositionParams", &JoltMeshGroup::m_convexDecompositionParams)
                ->Field("PhysicsMaterialSlots", &JoltMeshGroup::m_physicsMaterialSlots)
                ->Field("rules", &JoltMeshGroup::m_rules);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltMeshGroup>("Jolt Mesh group", "Configure Jolt mesh data exporting.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                        ->Attribute(AZ::Edit::Attributes::NameLabelOverride, "")
                        ->Attribute(AZ::Edit::Attributes::CategoryStyle, "display divider")

                    ->DataElement(AZ_CRC_CE("ManifestName"), &JoltMeshGroup::m_name, "Name Jolt Mesh",
                        "<span>Name for the group. This name will also be used as a part of the name for the "
                        "generated file.</span>")

                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltMeshGroup::m_nodeSelectionList, "Select meshes",
                        "<span>Select the meshes to be included in the mesh group.</span>")
                        ->Attribute("FilterName", "meshes")
                        ->Attribute("FilterType", AZ::SceneAPI::DataTypes::IMeshData::TYPEINFO_Uuid())
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &JoltMeshGroup::OnNodeSelectionChanged)

                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &JoltMeshGroup::m_exportMethod, "Export As",
                        "<span>The export method to be applied to this mesh group. For the asset to be usable as "
                        "a rigid body, select \"Convex\".</span>")
                        ->EnumAttribute(MeshExportMethod::TriMesh, "Triangle Mesh")
                        ->EnumAttribute(MeshExportMethod::Convex, "Convex")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &JoltMeshGroup::OnExportMethodChanged)

                    ->DataElement(AZ_CRC_CE("DecomposeMeshes"), &JoltMeshGroup::m_decomposeMeshes, "Decompose Meshes",
                        "<span>If enabled, this option will apply the V-HACD algorithm to split each node "
                        "into approximately convex parts. Each part will individually be exported as a convex "
                        "collider.</span>")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltMeshGroup::GetDecomposeMeshesVisibility)
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &JoltMeshGroup::OnDecomposeMeshesChanged)

                    ->DataElement(AZ_CRC_CE("TriangleMeshAssetParams"), &JoltMeshGroup::m_triangleMeshAssetParams, "Triangle Mesh Asset Parameters",
                        "<span>Configure the parameters controlling the exported triangle mesh asset.</span>")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltMeshGroup::GetExportAsTriMesh)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)

                    ->DataElement(AZ_CRC_CE("ConvexDecompositionParams"), &JoltMeshGroup::m_convexDecompositionParams, "Decomposition Parameters",
                        "<span>Configure the parameters controlling the approximate convex decomposition algorithm.</span>")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltMeshGroup::GetDecomposeMeshes)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)

                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltMeshGroup::m_physicsMaterialSlots, "", "")
                        ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)

                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltMeshGroup::m_rules, "",
                        "Add or remove rules to fine-tune the export process.")
                        ->Attribute(AZ::Edit::Attributes::Visibility, AZ_CRC_CE("PropertyVisibility_ShowChildrenOnly"));
            }
        }
    }

    const AZStd::string& JoltMeshGroup::GetName() const
    {
        return m_name;
    }

    void JoltMeshGroup::SetName(const AZStd::string& name)
    {
        m_name = name;
    }

    void JoltMeshGroup::SetName(AZStd::string&& name)
    {
        m_name = AZStd::move(name);
    }

    const AZ::Uuid& JoltMeshGroup::GetId() const
    {
        return m_id;
    }

    void JoltMeshGroup::OverrideId(const AZ::Uuid& id)
    {
        m_id = id;
    }

    bool JoltMeshGroup::GetExportAsConvex() const
    {
        return m_exportMethod == MeshExportMethod::Convex;
    }

    bool JoltMeshGroup::GetExportAsTriMesh() const
    {
        return m_exportMethod == MeshExportMethod::TriMesh;
    }

    bool JoltMeshGroup::GetDecomposeMeshes() const
    {
        // Decomposition only makes sense on top of convex export: it replaces each
        // node's single hull with several approximately convex parts.
        return GetExportAsConvex() && m_decomposeMeshes;
    }

    MeshExportMethod JoltMeshGroup::GetExportMethod() const
    {
        return m_exportMethod;
    }

    const Physics::MaterialSlots& JoltMeshGroup::GetMaterialSlots() const
    {
        return m_physicsMaterialSlots;
    }

    void JoltMeshGroup::SetSceneGraph(const AZ::SceneAPI::Containers::SceneGraph* graph)
    {
        m_graph = graph;
    }

    void JoltMeshGroup::UpdateMaterialSlots()
    {
        if (!m_graph)
        {
            return;
        }

        AZStd::optional<Utils::AssetMaterialsData> assetMaterialData = Utils::GatherMaterialsFromMeshGroup(*this, *m_graph);
        if (!assetMaterialData)
        {
            return;
        }

        Utils::UpdateAssetPhysicsMaterials(assetMaterialData->m_sourceSceneMaterialNames, m_physicsMaterialSlots);
    }

    AZ::SceneAPI::Containers::RuleContainer& JoltMeshGroup::GetRuleContainer()
    {
        return m_rules;
    }

    const AZ::SceneAPI::Containers::RuleContainer& JoltMeshGroup::GetRuleContainerConst() const
    {
        return m_rules;
    }

    AZ::SceneAPI::DataTypes::ISceneNodeSelectionList& JoltMeshGroup::GetSceneNodeSelectionList()
    {
        return m_nodeSelectionList;
    }

    const AZ::SceneAPI::DataTypes::ISceneNodeSelectionList& JoltMeshGroup::GetSceneNodeSelectionList() const
    {
        return m_nodeSelectionList;
    }

    JoltTriangleMeshAssetParams& JoltMeshGroup::GetTriangleMeshAssetParams()
    {
        return m_triangleMeshAssetParams;
    }

    const JoltTriangleMeshAssetParams& JoltMeshGroup::GetTriangleMeshAssetParams() const
    {
        return m_triangleMeshAssetParams;
    }

    JoltConvexDecompositionParams& JoltMeshGroup::GetConvexDecompositionParams()
    {
        return m_convexDecompositionParams;
    }

    const JoltConvexDecompositionParams& JoltMeshGroup::GetConvexDecompositionParams() const
    {
        return m_convexDecompositionParams;
    }

    AZ::u32 JoltMeshGroup::OnNodeSelectionChanged()
    {
        UpdateMaterialSlots();
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    AZ::u32 JoltMeshGroup::OnExportMethodChanged()
    {
        UpdateMaterialSlots();
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    AZ::u32 JoltMeshGroup::OnDecomposeMeshesChanged()
    {
        UpdateMaterialSlots();
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    bool JoltMeshGroup::GetDecomposeMeshesVisibility() const
    {
        return GetExportAsConvex();
    }

} // namespace JoltPhysics::Pipeline
