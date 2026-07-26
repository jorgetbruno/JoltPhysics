#pragma once

#include <AzCore/Component/TickBus.h>

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
        , private AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(EditorJoltMeshColliderComponent, "{F7A8B9C0-D1E2-4F30-B4C5-D6E7F8A9B0C1}", EditorJoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    protected:
        // EditorJoltColliderComponentBase
        void DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const override;
        AZ::Aabb GetLocalShapeBounds() const override;

    private:
        // AZ::TickBus - connected only while there is no baked data yet. The render
        // mesh's asset loads asynchronously, so the bake this component attempts on
        // activation usually loses the race; retrying each editor tick until the mesh
        // can answer is what makes a freshly added (or freshly authored) collider bake
        // itself instead of waiting for someone to find the button.
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        //! Cooks the entity's current render geometry into m_shapeConfiguration.
        //! Returns false (optionally warning) when no geometry is available yet.
        bool BakeFromRenderMesh(bool warnOnFailure);

        //! A bake changed serialized component data, so the level has to know: without
        //! the dirty mark, saving skips the entity and the baked mesh dies with the
        //! session - the level keeps warning "no baked collision mesh" on every play.
        void MarkBakedDataDirty();

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
        mutable AZ::Aabb m_debugBounds = AZ::Aabb::CreateNull(); //!< Bounds of the same vertices.
        mutable bool m_debugLinesDirty = true;
    };
} // namespace JoltPhysics
