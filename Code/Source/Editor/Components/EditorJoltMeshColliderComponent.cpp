#include <Editor/Components/EditorJoltMeshColliderComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Visibility/VisibleGeometryBus.h>

#include <Clients/Components/JoltMeshColliderComponent.h>
#include <Editor/Components/EditorJoltColliderGeometryUtils.h>
#include <Shape/JoltMeshUtils.h>
#include <Utils/ReflectionUtils.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace JoltPhysics
{
    void EditorJoltMeshColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltColliderComponentBase::Reflect(context);
        Internal::ReflectOnce<Physics::CookedMeshShapeConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {

            serializeContext->Class<EditorJoltMeshColliderComponent, EditorJoltColliderComponentBase>()
                ->Version(2)
                ->Field("MeshType", &EditorJoltMeshColliderComponent::m_meshType)
                ->Field("ConvexMode", &EditorJoltMeshColliderComponent::m_convexMode)
                ->Field("DecompositionMaxHulls", &EditorJoltMeshColliderComponent::m_decompositionMaxHulls)
                ->Field("DecompositionVoxelResolution", &EditorJoltMeshColliderComponent::m_decompositionVoxelResolution)
                ->Field("DecompositionMaxVerticesPerHull", &EditorJoltMeshColliderComponent::m_decompositionMaxVerticesPerHull)
                ->Field("DecompositionConcavity", &EditorJoltMeshColliderComponent::m_decompositionConcavity)
                ->Field("ShapeConfiguration", &EditorJoltMeshColliderComponent::m_shapeConfiguration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltMeshColliderComponent>(
                    "Jolt Mesh Collider", "Collider baked from the entity's render mesh for the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &EditorJoltMeshColliderComponent::m_meshType,
                        "Mesh Type",
                        "Triangle Mesh matches the render geometry exactly (static bodies only); "
                        "Convex Hull wraps it in convex shape(s) and also works on dynamic rigid bodies.")
                        ->EnumAttribute(Physics::CookedMeshShapeConfiguration::MeshType::TriangleMesh, "Triangle Mesh")
                        ->EnumAttribute(Physics::CookedMeshShapeConfiguration::MeshType::Convex, "Convex Hull")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltMeshColliderComponent::OnBakingSettingsChanged)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &EditorJoltMeshColliderComponent::m_convexMode,
                        "Convex Mode",
                        "Single Hull wraps all render geometry in one hull; Hull per Mesh Node bakes one hull "
                        "per render node (e.g. wheels separate from the body); Decomposed runs VHACD over the "
                        "merged geometry to approximate it with a set of hulls.")
                        ->EnumAttribute(ConvexMode::SingleHull, "Single Hull")
                        ->EnumAttribute(ConvexMode::HullPerMeshNode, "Hull per Mesh Node")
                        ->EnumAttribute(ConvexMode::Decomposed, "Decomposed (VHACD)")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &EditorJoltMeshColliderComponent::IsConvexModeVisible)
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltMeshColliderComponent::OnBakingSettingsChanged)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltMeshColliderComponent::m_decompositionMaxHulls,
                        "Max Hulls", "Maximum convex hulls the decomposition may produce.")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &EditorJoltMeshColliderComponent::IsDecomposedModeVisible)
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltMeshColliderComponent::OnBakingSettingsChanged)
                        ->Attribute(AZ::Edit::Attributes::Min, 1)
                        ->Attribute(AZ::Edit::Attributes::Max, 256)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltMeshColliderComponent::m_decompositionVoxelResolution,
                        "Voxel Resolution", "Voxelization resolution for the decomposition; higher is more faithful and slower.")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &EditorJoltMeshColliderComponent::IsDecomposedModeVisible)
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltMeshColliderComponent::OnBakingSettingsChanged)
                        ->Attribute(AZ::Edit::Attributes::Min, 10000)
                        ->Attribute(AZ::Edit::Attributes::Max, 1000000)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltMeshColliderComponent::m_decompositionMaxVerticesPerHull,
                        "Max Vertices per Hull", "Per-hull vertex cap for the decomposition (Jolt hulls cap at 256).")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &EditorJoltMeshColliderComponent::IsDecomposedModeVisible)
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltMeshColliderComponent::OnBakingSettingsChanged)
                        ->Attribute(AZ::Edit::Attributes::Min, 4)
                        ->Attribute(AZ::Edit::Attributes::Max, 256)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltMeshColliderComponent::m_decompositionConcavity,
                        "Concavity", "Maximum concavity error allowed before a hull is split further.")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &EditorJoltMeshColliderComponent::IsDecomposedModeVisible)
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltMeshColliderComponent::OnBakingSettingsChanged)
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0)
                    ->UIElement(AZ::Edit::UIHandlers::Button, "", "Re-bake the collision mesh from the entity's current render geometry.")
                        ->Attribute(AZ::Edit::Attributes::ButtonText, "Bake from render mesh")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltMeshColliderComponent::OnBakeButtonPressed)
                    ;
            }
        }
    }

    void EditorJoltMeshColliderComponent::Activate()
    {
        EditorJoltColliderComponentBase::Activate();

        m_debugLinesDirty = true;

        // No baked data yet: keep trying each editor tick. The immediate attempt in
        // the old code lost the race against the render mesh's async asset load on
        // almost every level open, and nothing ever retried - so colliders stayed
        // empty until someone found the bake button.
        if (m_shapeConfiguration.GetCookedMeshData().empty())
        {
            AZ::TickBus::Handler::BusConnect();
        }
    }

    void EditorJoltMeshColliderComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        CancelDecompositionJob();
        EditorJoltColliderComponentBase::Deactivate();
    }

    void EditorJoltMeshColliderComponent::OnTick(
        [[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        // A decomposition job runs on a worker thread; collect it once it reports done.
        if (m_decompositionJob)
        {
            if (m_decompositionJob->m_finished)
            {
                FinishDecompositionBake();
            }
            return;
        }

        // Quiet: "the mesh is not ready yet" is the expected state here, and the mesh
        // component already logs its own warning when asked too early.
        if (BakeFromRenderMesh(/*warnOnFailure*/ false))
        {
            MarkBakedDataDirty();
            AZ_Printf("JoltPhysics", "Jolt Mesh Collider on entity '%s' baked its collision mesh (%zu KiB). "
                "Save the level to keep it.\n",
                GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>",
                m_shapeConfiguration.GetCookedMeshData().size() / 1024);
            AZ::TickBus::Handler::BusDisconnect();
        }
    }

    void EditorJoltMeshColliderComponent::MarkBakedDataDirty()
    {
        AzToolsFramework::ScopedUndoBatch undoBatch("Bake collision mesh");
        AzToolsFramework::ScopedUndoBatch::MarkEntityDirty(GetEntityId());
    }

    bool EditorJoltMeshColliderComponent::BakeFromRenderMesh(bool warnOnFailure)
    {
        if (m_meshType == Physics::CookedMeshShapeConfiguration::MeshType::Convex &&
            m_convexMode == ConvexMode::Decomposed)
        {
            return StartDecompositionBake(warnOnFailure);
        }

        AzFramework::VisibleGeometryContainer geometryContainer;
        AzFramework::VisibleGeometryRequestBus::Event(
            GetEntityId(), &AzFramework::VisibleGeometryRequests::BuildVisibleGeometry,
            AZ::Aabb::CreateNull(), geometryContainer);

        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        if (!JoltMeshUtils::CookVisibleGeometry(
                geometryContainer, worldTransform, m_meshType, m_shapeConfiguration,
                m_convexMode == ConvexMode::HullPerMeshNode
                    ? JoltMeshUtils::ConvexGrouping::PerGeometryEntry
                    : JoltMeshUtils::ConvexGrouping::Single))
        {
            AZ_Warning("JoltPhysics", !warnOnFailure,
                "Jolt Mesh Collider: no render geometry found on entity '%s'. Add a Mesh component (and wait for "
                "its asset to load), then press 'Bake from render mesh'.",
                GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>");
            return false;
        }

        m_debugLinesDirty = true;
        return true;
    }

    bool EditorJoltMeshColliderComponent::StartDecompositionBake(bool warnOnFailure)
    {
        if (m_decompositionJob)
        {
            return false; // a worker is already decomposing the previous soup
        }

        AzFramework::VisibleGeometryContainer geometryContainer;
        AzFramework::VisibleGeometryRequestBus::Event(
            GetEntityId(), &AzFramework::VisibleGeometryRequests::BuildVisibleGeometry,
            AZ::Aabb::CreateNull(), geometryContainer);

        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        if (!JoltMeshUtils::GatherVisibleGeometrySoup(geometryContainer, worldTransform, vertices, indices))
        {
            AZ_Warning("JoltPhysics", !warnOnFailure,
                "Jolt Mesh Collider: no render geometry found on entity '%s'. Add a Mesh component (and wait for "
                "its asset to load), then press 'Bake from render mesh'.",
                GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>");
            return false;
        }

        EditorConvexDecomposition::DecompositionParams params;
        params.m_maxHulls = m_decompositionMaxHulls;
        params.m_voxelResolution = m_decompositionVoxelResolution;
        params.m_maxVerticesPerHull = m_decompositionMaxVerticesPerHull;
        params.m_concavity = m_decompositionConcavity;

        // The job struct is shared with the worker, so the component can drop its own
        // reference (Deactivate, settings change) without the worker ever touching it.
        m_decompositionJob = AZStd::make_shared<DecompositionJob>();
        AZStd::shared_ptr<DecompositionJob> job = m_decompositionJob;
        job->m_thread = AZStd::thread(
            [job, params, vertices = AZStd::move(vertices), indices = AZStd::move(indices)]() mutable
            {
                job->m_result = EditorConvexDecomposition::DecomposeToHullPointClouds(vertices, indices, params);
                job->m_finished = true;
            });

        // The tick bus polls for the job's completion.
        if (!AZ::TickBus::Handler::BusIsConnected())
        {
            AZ::TickBus::Handler::BusConnect();
        }
        return false; // the bake itself lands in FinishDecompositionBake
    }

    void EditorJoltMeshColliderComponent::FinishDecompositionBake()
    {
        AZStd::shared_ptr<DecompositionJob> job = AZStd::move(m_decompositionJob);
        if (job->m_thread.joinable())
        {
            job->m_thread.join();
        }

        const size_t hullCount = job->m_result.m_hulls.size();
        const AZStd::vector<AZ::u8> cookedData = JoltMeshUtils::PackConvexHulls(job->m_result.m_hulls);
        if (!cookedData.empty())
        {
            m_shapeConfiguration = Physics::CookedMeshShapeConfiguration();
            m_shapeConfiguration.SetCookedMeshData(cookedData.data(), cookedData.size(), m_meshType);
            m_debugLinesDirty = true;
            MarkBakedDataDirty();
            AZ_Printf("JoltPhysics", "Jolt Mesh Collider on entity '%s' decomposed into %zu hulls (%zu KiB). "
                "Save the level to keep it.\n",
                GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>",
                hullCount, cookedData.size() / 1024);
        }
        else
        {
            // The geometry was there and VHACD still produced nothing; retrying every
            // tick would just burn worker threads, so give up until the user re-bakes.
            AZ_Warning("JoltPhysics", false,
                "Jolt Mesh Collider: convex decomposition produced no hulls on entity '%s'.",
                GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>");
        }
        AZ::TickBus::Handler::BusDisconnect();
    }

    void EditorJoltMeshColliderComponent::CancelDecompositionJob()
    {
        if (!m_decompositionJob)
        {
            return;
        }
        if (m_decompositionJob->m_thread.joinable())
        {
            if (m_decompositionJob->m_finished)
            {
                m_decompositionJob->m_thread.join();
            }
            else
            {
                // The worker holds its own reference to the job struct and never touches
                // the component, so it can finish in peace after we detach.
                m_decompositionJob->m_thread.detach();
            }
        }
        m_decompositionJob.reset();
    }

    AZ::u32 EditorJoltMeshColliderComponent::OnBakeButtonPressed()
    {
        if (BakeFromRenderMesh(/*warnOnFailure*/ true))
        {
            // Success used to be silent, which reads as the button doing nothing when
            // the mesh happens to be off screen. Say what happened, and mark the entity
            // dirty so saving the level actually keeps the result.
            MarkBakedDataDirty();
            AZ_Printf("JoltPhysics", "Jolt Mesh Collider on entity '%s' baked its collision mesh (%zu KiB). "
                "Save the level to keep it.\n",
                GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>",
                m_shapeConfiguration.GetCookedMeshData().size() / 1024);
            AZ::TickBus::Handler::BusDisconnect();
        }
        else if (m_decompositionJob)
        {
            // Decomposed mode only starts the worker here; completion lands on tick.
            AZ_Printf("JoltPhysics", "Jolt Mesh Collider on entity '%s' is decomposing on a worker thread; "
                "the collider updates when it finishes.\n",
                GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>");
        }
        return AZ::Edit::PropertyRefreshLevels::AttributesAndValues;
    }

    AZ::u32 EditorJoltMeshColliderComponent::OnBakingSettingsChanged()
    {
        // Changing the type, mode, or decomposition parameters invalidates the baked
        // blob; drop any in-flight decomposition and re-bake right away when geometry
        // is available (quietly otherwise - the old data is cleared regardless).
        CancelDecompositionJob();
        if (!BakeFromRenderMesh(/*warnOnFailure*/ false))
        {
            m_shapeConfiguration = Physics::CookedMeshShapeConfiguration();
            m_debugLinesDirty = true;
        }
        return AZ::Edit::PropertyRefreshLevels::AttributesAndValues;
    }

    void EditorJoltMeshColliderComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        AZ_Warning("JoltPhysics", !m_shapeConfiguration.GetCookedMeshData().empty(),
            "Jolt Mesh Collider on entity '%s' has no baked collision mesh; the runtime collider will be empty. "
            "Press 'Bake from render mesh' in the editor.",
            GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>");

        if (auto* component = gameEntity->CreateComponent<JoltMeshColliderComponent>())
        {
            component->GetColliderConfiguration() = m_colliderConfiguration;
            component->GetShapeConfiguration() = m_shapeConfiguration;
        }
    }

    void EditorJoltMeshColliderComponent::RebuildDebugLines() const
    {
        m_debugLines.clear();
        m_debugBounds = AZ::Aabb::CreateNull();
        m_debugLinesDirty = false;

        const AZStd::vector<AZ::u8>& cookedData = m_shapeConfiguration.GetCookedMeshData();
        if (cookedData.empty())
        {
            return;
        }

        // Build a throwaway native shape from the baked blob and extract its triangle
        // soup; for convex hulls this draws the actual hull, not the input point cloud.
        const JPH::RefConst<JPH::Shape> shape =
            (m_shapeConfiguration.GetMeshType() == Physics::CookedMeshShapeConfiguration::MeshType::Convex)
            ? JoltMeshUtils::CreateConvexShapeFromCookedData(cookedData)
            : JoltMeshUtils::CreateMeshShapeFromCookedData(cookedData);

        EditorColliderGeometry::BuildShapeWireframe(shape.GetPtr(), m_debugLines, m_debugBounds);
    }

    AZ::Aabb EditorJoltMeshColliderComponent::GetLocalShapeBounds() const
    {
        // The baked triangles are the only description of this collider's extent, so the
        // same pass that builds the wireframe supplies what the viewport picks against.
        if (m_debugLinesDirty)
        {
            RebuildDebugLines();
        }
        return m_debugBounds;
    }

    void EditorJoltMeshColliderComponent::DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const
    {
        if (m_debugLinesDirty)
        {
            RebuildDebugLines();
        }
        if (m_debugLines.empty())
        {
            return;
        }

        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        const AZ::Transform colliderTransform = worldTransform * AZ::Transform::CreateFromQuaternionAndTranslation(
            m_colliderConfiguration.m_rotation, m_colliderConfiguration.m_position);

        m_debugLinesWorld.resize(m_debugLines.size());
        for (size_t i = 0; i < m_debugLines.size(); ++i)
        {
            m_debugLinesWorld[i] = colliderTransform.TransformPoint(m_debugLines[i]);
        }

        debugDisplay.DrawLines(m_debugLinesWorld, AZ::Color(0.0f, 1.0f, 0.0f, 1.0f));
    }

} // namespace JoltPhysics
