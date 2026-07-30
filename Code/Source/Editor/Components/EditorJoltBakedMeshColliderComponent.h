#pragma once

#include <AzCore/Component/TickBus.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>

#include <Editor/Components/EditorJoltColliderComponentBase.h>
#include <Editor/EditorJoltConvexDecomposition.h>

#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    //! Editor Mesh Collider: bakes the entity's render geometry (via
    //! AzFramework::VisibleGeometryRequestBus, implemented by the Mesh component) into
    //! cooked triangle-mesh or convex-hull data, draws it in the Edit viewport, and
    //! spawns the runtime JoltBakedMeshColliderComponent via BuildGameEntity.
    //! Baking happens automatically on activation when no baked data exists yet, and
    //! on demand through the "Bake from render mesh" button (e.g. after the render
    //! mesh changes).
    //! Convex bakes can produce a single hull, a hull group (one hull per render node,
    //! e.g. wheels separate from the wagon body), or an automatic convex decomposition
    //! (VHACD) of the merged geometry - the latter on a worker thread, collected on tick.
    class EditorJoltBakedMeshColliderComponent
        : public EditorJoltColliderComponentBase
        , private AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(EditorJoltBakedMeshColliderComponent, "{F7A8B9C0-D1E2-4F30-B4C5-D6E7F8A9B0C1}", EditorJoltColliderComponentBase);

        //! How a convex bake groups the render geometry into hulls.
        enum class ConvexMode : AZ::u8
        {
            SingleHull, //!< One hull around all render geometry.
            HullPerMeshNode, //!< One hull per render-mesh node (a "convex hull group").
            Decomposed, //!< Automatic convex decomposition (VHACD) of the merged geometry.
        };

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
        // AZ::TickBus - connected while there is no baked data yet or a decomposition
        // job is in flight. The render mesh's asset loads asynchronously, so the bake
        // this component attempts on activation usually loses the race; retrying each
        // editor tick until the mesh can answer is what makes a freshly added (or
        // freshly authored) collider bake itself instead of waiting for someone to
        // find the button.
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        //! Cooks the entity's current render geometry into m_shapeConfiguration.
        //! Returns false (optionally warning) when no geometry is available yet - and
        //! also in Decomposed mode, where it only *starts* the worker job and the
        //! completion lands on a later OnTick.
        bool BakeFromRenderMesh(bool warnOnFailure);

        //! Decomposed-mode bake: gathers the triangle soup and starts VHACD on its own
        //! thread. False while a run is already in flight.
        bool StartDecompositionBake(bool warnOnFailure);

        //! Collects a finished decomposition: packs the hull group blob and marks the
        //! data dirty. Gives up auto-retry when VHACD itself produced nothing.
        void FinishDecompositionBake();

        //! Reports the running decomposition's progress to the console, once per 10%.
        //! A decomposition of a dense mesh runs for tens of seconds, and in silence the
        //! editor looks like it ignored the button.
        void ReportDecompositionProgress();

        //! Stops an in-flight decomposition (settings changed, or the component is going
        //! away). VHACD is signalled and its thread joined, so the work really stops
        //! rather than running on to produce a result nobody reads.
        void CancelDecompositionJob();

        //! A bake changed serialized component data, so the level has to know: without
        //! the dirty mark, saving skips the entity and the baked mesh dies with the
        //! session - the level keeps warning "no baked collision mesh" on every play.
        void MarkBakedDataDirty();

        AZ::u32 OnBakeButtonPressed();
        AZ::u32 OnBakingSettingsChanged();

        //! The convex-mode setting only applies to convex bakes; hide it otherwise.
        bool IsConvexModeVisible() const
        {
            return m_meshType == Physics::CookedMeshShapeConfiguration::MeshType::Convex;
        }

        //! The decomposition parameters only apply to Decomposed bakes.
        bool IsDecomposedModeVisible() const
        {
            return IsConvexModeVisible() && m_convexMode == ConvexMode::Decomposed;
        }

        //! Decodes the baked data into a world-space-ready line list for DrawShape.
        void RebuildDebugLines() const;

        Physics::CookedMeshShapeConfiguration::MeshType m_meshType =
            Physics::CookedMeshShapeConfiguration::MeshType::TriangleMesh;
        ConvexMode m_convexMode = ConvexMode::SingleHull;
        Physics::CookedMeshShapeConfiguration m_shapeConfiguration;

        AZ::u32 m_decompositionMaxHulls = 16;
        AZ::u32 m_decompositionVoxelResolution = 100000;
        AZ::u32 m_decompositionMaxVerticesPerHull = 64;
        double m_decompositionConcavity = 0.001;

        //! Set only while VHACD is running; destroying it cancels the run.
        AZStd::unique_ptr<EditorConvexDecomposition::DecompositionSession> m_decompositionSession;
        //! Last progress decile already printed, so the console gets one line per 10%.
        int m_reportedProgressDecile = -1;

        //! Edge list (point pairs, entity-local space) of the baked mesh, for viewport drawing.
        mutable AZStd::vector<AZ::Vector3> m_debugLines;
        mutable AZStd::vector<AZ::Vector3> m_debugLinesWorld; //!< Per-frame transformed scratch.
        mutable AZ::Aabb m_debugBounds = AZ::Aabb::CreateNull(); //!< Bounds of the same vertices.
        mutable bool m_debugLinesDirty = true;
    };
} // namespace JoltPhysics
