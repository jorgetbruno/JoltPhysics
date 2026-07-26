#pragma once

#include <Editor/Components/EditorJoltColliderComponentBase.h>

#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    //! Editor Mesh Collider: bakes the entity's render geometry (via
    //! AzFramework::VisibleGeometryRequestBus, implemented by the Mesh component) into
    //! cooked triangle-mesh or convex-hull data, draws it in the Edit viewport, and
    //! spawns the runtime JoltMeshColliderComponent via BuildGameEntity.
    //! Baking happens automatically on activation when no baked data exists yet, and
    //! on demand through the "Bake from render mesh" button (e.g. after the render
    //! mesh changes).
    class EditorJoltMeshColliderComponent
        : public EditorJoltColliderComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltMeshColliderComponent, "{F7A8B9C0-D1E2-4F30-B4C5-D6E7F8A9B0C1}", EditorJoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void Activate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    protected:
        // EditorJoltColliderComponentBase
        void DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const override;

    private:
        //! Cooks the entity's current render geometry into m_shapeConfiguration.
        //! Returns false (optionally warning) when no geometry is available yet.
        bool BakeFromRenderMesh(bool warnOnFailure);

        AZ::u32 OnBakeButtonPressed();
        AZ::u32 OnMeshTypeChanged();

        //! Decodes the baked data into a world-space-ready line list for DrawShape.
        void RebuildDebugLines() const;

        Physics::CookedMeshShapeConfiguration::MeshType m_meshType =
            Physics::CookedMeshShapeConfiguration::MeshType::TriangleMesh;
        Physics::CookedMeshShapeConfiguration m_shapeConfiguration;

        //! Edge list (point pairs, entity-local space) of the baked mesh, for viewport drawing.
        mutable AZStd::vector<AZ::Vector3> m_debugLines;
        mutable AZStd::vector<AZ::Vector3> m_debugLinesWorld; //!< Per-frame transformed scratch.
        mutable bool m_debugLinesDirty = true;
    };
} // namespace JoltPhysics
