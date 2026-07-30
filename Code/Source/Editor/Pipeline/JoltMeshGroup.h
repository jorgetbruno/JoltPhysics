#pragma once

#include <AzCore/Memory/Memory.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzFramework/Physics/Material/PhysicsMaterialSlots.h>
#include <SceneAPI/SceneCore/Containers/RuleContainer.h>
#include <SceneAPI/SceneCore/DataTypes/Groups/ISceneNodeGroup.h>
#include <SceneAPI/SceneData/ManifestBase/SceneNodeSelectionList.h>

namespace AZ
{
    class ReflectContext;
    class SerializeContext;

    namespace SceneAPI::Containers
    {
        class SceneGraph;
    }
}

namespace JoltPhysics::Pipeline
{
    class JoltMeshGroup;

    //! How the selected render meshes are turned into collision geometry.
    //! There is intentionally no Primitive mode (PhysX has one): the gem already
    //! ships dedicated primitive collider components, so the asset pipeline only
    //! ever produces real mesh geometry.
    enum class MeshExportMethod : AZ::u8
    {
        TriMesh,
        Convex,
    };

    //! Knobs that only apply to triangle-mesh export. Jolt needs no cooking
    //! parameters of its own (MeshShape builds its BVH when the shape is created,
    //! not offline), so the only decision left is whether the selected nodes are
    //! merged into one shared triangle soup or stay separate shapes.
    class JoltTriangleMeshAssetParams
    {
    public:
        AZ_TYPE_INFO(JoltTriangleMeshAssetParams, "{2FAB077E-D1CC-4CB2-BF36-38E766AB4ACB}");

        static void Reflect(AZ::ReflectContext* context);

        bool GetMergeMeshes() const;
        void SetMergeMeshes(bool mergeMeshes);

    private:
        bool m_mergeMeshes = false;
    };

    //! Parameters for the optional approximate convex decomposition (V-HACD),
    //! mapping one-to-one onto EditorConvexDecomposition::DecompositionParams.
    //! Decomposition is opt-in (JoltMeshGroup::m_decomposeMeshes defaults to
    //! false) because it is lossy and significantly slower than a plain hull.
    class JoltConvexDecompositionParams
    {
    public:
        AZ_TYPE_INFO(JoltConvexDecompositionParams, "{6DADDA0A-2B72-4930-99A0-2EFA16CD2277}");

        static void Reflect(AZ::ReflectContext* context);

        AZ::u32 GetMaxConvexHulls() const;
        AZ::u32 GetResolution() const;
        AZ::u32 GetMaxNumVerticesPerConvexHull() const;
        double GetConcavity() const;

    private:
        AZ::u32 m_maxConvexHulls = 8; //!< 8 hulls cover most props (chairs, vases, pillars) without exploding cook times.
        AZ::u32 m_resolution = 400000; //!< Voxelization resolution, default from vhacd.h.
        AZ::u32 m_maxNumVerticesPerConvexHull = 64; //!< Per-hull vertex cap; Jolt hulls support at most 256 points.
        double m_concavity = 0.001; //!< Maximum concavity error accepted before a hull is split further.
    };

    //! SceneAPI manifest group describing one .joltmesh product: which render
    //! mesh nodes to export, how to cook them, and which physics materials the
    //! resulting shapes map to. Mirrors PhysX::Pipeline::MeshGroup, trimmed to
    //! the options Jolt actually consumes.
    class JoltMeshGroup
        : public AZ::SceneAPI::DataTypes::ISceneNodeGroup
    {
    public:
        AZ_RTTI(JoltMeshGroup, "{E6AEACB6-0A6C-4642-95C2-BC3E3A95C643}", AZ::SceneAPI::DataTypes::ISceneNodeGroup);
        AZ_CLASS_ALLOCATOR_DECL

        JoltMeshGroup();
        ~JoltMeshGroup() override;

        static void Reflect(AZ::ReflectContext* context);

        const AZStd::string& GetName() const override;
        void SetName(const AZStd::string& name);
        void SetName(AZStd::string&& name);
        const AZ::Uuid& GetId() const override;
        void OverrideId(const AZ::Uuid& id);
        bool GetExportAsConvex() const;
        bool GetExportAsTriMesh() const;
        bool GetDecomposeMeshes() const;
        MeshExportMethod GetExportMethod() const;
        const Physics::MaterialSlots& GetMaterialSlots() const;

        void SetSceneGraph(const AZ::SceneAPI::Containers::SceneGraph* graph);
        void UpdateMaterialSlots();

        AZ::SceneAPI::Containers::RuleContainer& GetRuleContainer() override;
        const AZ::SceneAPI::Containers::RuleContainer& GetRuleContainerConst() const override;

        AZ::SceneAPI::DataTypes::ISceneNodeSelectionList& GetSceneNodeSelectionList() override;
        const AZ::SceneAPI::DataTypes::ISceneNodeSelectionList& GetSceneNodeSelectionList() const override;

        JoltTriangleMeshAssetParams& GetTriangleMeshAssetParams();
        const JoltTriangleMeshAssetParams& GetTriangleMeshAssetParams() const;

        JoltConvexDecompositionParams& GetConvexDecompositionParams();
        const JoltConvexDecompositionParams& GetConvexDecompositionParams() const;

    protected:
        AZ::u32 OnNodeSelectionChanged();
        AZ::u32 OnExportMethodChanged();
        AZ::u32 OnDecomposeMeshesChanged();

        bool GetDecomposeMeshesVisibility() const;

        AZ::Uuid m_id{};
        AZStd::string m_name{};
        AZ::SceneAPI::SceneData::SceneNodeSelectionList m_nodeSelectionList{};
        MeshExportMethod m_exportMethod{ MeshExportMethod::Convex };
        bool m_decomposeMeshes{ false };
        JoltTriangleMeshAssetParams m_triangleMeshAssetParams{};
        JoltConvexDecompositionParams m_convexDecompositionParams{};
        AZ::SceneAPI::Containers::RuleContainer m_rules{};
        Physics::MaterialSlots m_physicsMaterialSlots;

        // Transient (not serialized): lets UpdateMaterialSlots re-read the materials
        // of the currently selected nodes after an edit-context change.
        const AZ::SceneAPI::Containers::SceneGraph* m_graph = nullptr;
    };

} // namespace JoltPhysics::Pipeline
