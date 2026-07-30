#include <Editor/Components/EditorJoltBakedMeshColliderComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Visibility/VisibleGeometryBus.h>

#include <Clients/Components/JoltBakedMeshColliderComponent.h>
#include <Editor/Components/EditorJoltColliderGeometryUtils.h>
#include <Shape/JoltMeshUtils.h>
#include <Utils/ReflectionUtils.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace JoltPhysics
{
    void EditorJoltBakedMeshColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltColliderComponentBase::Reflect(context);
        Internal::ReflectOnce<Physics::CookedMeshShapeConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {

            serializeContext->Class<EditorJoltBakedMeshColliderComponent, EditorJoltColliderComponentBase>()
                ->Version(2)
                ->Field("MeshType", &EditorJoltBakedMeshColliderComponent::m_meshType)
                ->Field("ConvexMode", &EditorJoltBakedMeshColliderComponent::m_convexMode)
                ->Field("DecompositionMaxHulls", &EditorJoltBakedMeshColliderComponent::m_decompositionMaxHulls)
                ->Field("DecompositionVoxelResolution", &EditorJoltBakedMeshColliderComponent::m_decompositionVoxelResolution)
                ->Field("DecompositionMaxVerticesPerHull", &EditorJoltBakedMeshColliderComponent::m_decompositionMaxVerticesPerHull)
                ->Field("DecompositionConcavity", &EditorJoltBakedMeshColliderComponent::m_decompositionConcavity)
                ->Field("ShapeConfiguration", &EditorJoltBakedMeshColliderComponent::m_shapeConfiguration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltBakedMeshColliderComponent>(
                    "Jolt Baked Mesh Collider", "Collider baked from the entity's render mesh for the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &EditorJoltBakedMeshColliderComponent::m_meshType,
                        "Mesh Type",
                        "Triangle Mesh matches the render geometry exactly (static bodies only); "
                        "Convex Hull wraps it in convex shape(s) and also works on dynamic rigid bodies.")
                        ->EnumAttribute(Physics::CookedMeshShapeConfiguration::MeshType::TriangleMesh, "Triangle Mesh")
                        ->EnumAttribute(Physics::CookedMeshShapeConfiguration::MeshType::Convex, "Convex Hull")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltBakedMeshColliderComponent::OnBakingSettingsChanged)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &EditorJoltBakedMeshColliderComponent::m_convexMode,
                        "Convex Mode",
                        "Single Hull wraps all render geometry in one hull; Hull per Mesh Node bakes one hull "
                        "per render node (e.g. wheels separate from the body); Decomposed runs VHACD over the "
                        "merged geometry to approximate it with a set of hulls.")
                        ->EnumAttribute(ConvexMode::SingleHull, "Single Hull")
                        ->EnumAttribute(ConvexMode::HullPerMeshNode, "Hull per Mesh Node")
                        ->EnumAttribute(ConvexMode::Decomposed, "Decomposed (VHACD)")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &EditorJoltBakedMeshColliderComponent::IsConvexModeVisible)
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltBakedMeshColliderComponent::OnBakingSettingsChanged)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltBakedMeshColliderComponent::m_decompositionMaxHulls,
                        "Max Hulls", "Maximum convex hulls the decomposition may produce.")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &EditorJoltBakedMeshColliderComponent::IsDecomposedModeVisible)
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltBakedMeshColliderComponent::OnBakingSettingsChanged)
                        ->Attribute(AZ::Edit::Attributes::Min, 1)
                        ->Attribute(AZ::Edit::Attributes::Max, 256)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltBakedMeshColliderComponent::m_decompositionVoxelResolution,
                        "Voxel Resolution", "Voxelization resolution for the decomposition; higher is more faithful and slower.")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &EditorJoltBakedMeshColliderComponent::IsDecomposedModeVisible)
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltBakedMeshColliderComponent::OnBakingSettingsChanged)
                        ->Attribute(AZ::Edit::Attributes::Min, 10000)
                        ->Attribute(AZ::Edit::Attributes::Max, 1000000)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltBakedMeshColliderComponent::m_decompositionMaxVerticesPerHull,
                        "Max Vertices per Hull", "Per-hull vertex cap for the decomposition (Jolt hulls cap at 256).")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &EditorJoltBakedMeshColliderComponent::IsDecomposedModeVisible)
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltBakedMeshColliderComponent::OnBakingSettingsChanged)
                        ->Attribute(AZ::Edit::Attributes::Min, 4)
                        ->Attribute(AZ::Edit::Attributes::Max, 256)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltBakedMeshColliderComponent::m_decompositionConcavity,
                        "Concavity", "Maximum concavity error allowed before a hull is split further.")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &EditorJoltBakedMeshColliderComponent::IsDecomposedModeVisible)
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltBakedMeshColliderComponent::OnBakingSettingsChanged)
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0)
                    ->UIElement(AZ::Edit::UIHandlers::Button, "", "Re-bake the collision mesh from the entity's current render geometry.")
                        ->Attribute(AZ::Edit::Attributes::ButtonText, "Bake from render mesh")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltBakedMeshColliderComponent::OnBakeButtonPressed)
                    ;
            }
        }
    }

    void EditorJoltBakedMeshColliderComponent::Activate()
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

    void EditorJoltBakedMeshColliderComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        CancelDecompositionJob();
        EditorJoltColliderComponentBase::Deactivate();
    }

    void EditorJoltBakedMeshColliderComponent::OnTick(
        [[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        // A decomposition runs on VHACD's own thread; polling it here is both the
        // completion check and what pumps its progress messages.
        if (m_decompositionSession)
        {
            if (m_decompositionSession->IsFinished())
            {
                FinishDecompositionBake();
            }
            else
            {
                ReportDecompositionProgress();
            }
            return;
        }

        // Quiet: "the mesh is not ready yet" is the expected state here, and the mesh
        // component already logs its own warning when asked too early.
        if (BakeFromRenderMesh(/*warnOnFailure*/ false))
        {
            MarkBakedDataDirty();
            AZ_Printf("JoltPhysics", "Jolt Baked Mesh Collider on entity '%s' baked its collision mesh (%zu KiB). "
                "Save the level to keep it.\n",
                GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>",
                m_shapeConfiguration.GetCookedMeshData().size() / 1024);
            AZ::TickBus::Handler::BusDisconnect();
        }
    }

    void EditorJoltBakedMeshColliderComponent::MarkBakedDataDirty()
    {
        AzToolsFramework::ScopedUndoBatch undoBatch("Bake collision mesh");
        AzToolsFramework::ScopedUndoBatch::MarkEntityDirty(GetEntityId());
    }

    bool EditorJoltBakedMeshColliderComponent::BakeFromRenderMesh(bool warnOnFailure)
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
                "Jolt Baked Mesh Collider: no render geometry found on entity '%s'. Add a Mesh component (and wait for "
                "its asset to load), then press 'Bake from render mesh'.",
                GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>");
            return false;
        }

        m_debugLinesDirty = true;
        return true;
    }

    bool EditorJoltBakedMeshColliderComponent::StartDecompositionBake(bool warnOnFailure)
    {
        if (m_decompositionSession)
        {
            return false; // a run is already decomposing the previous soup
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
                "Jolt Baked Mesh Collider: no render geometry found on entity '%s'. Add a Mesh component (and wait for "
                "its asset to load), then press 'Bake from render mesh'.",
                GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>");
            return false;
        }

        EditorConvexDecomposition::DecompositionParams params;
        params.m_maxHulls = m_decompositionMaxHulls;
        params.m_voxelResolution = m_decompositionVoxelResolution;
        params.m_maxVerticesPerHull = m_decompositionMaxVerticesPerHull;
        params.m_concavity = m_decompositionConcavity;

        auto session = AZStd::make_unique<EditorConvexDecomposition::DecompositionSession>(vertices, indices, params);
        if (!session->IsValid())
        {
            AZ_Warning("JoltPhysics", !warnOnFailure,
                "Jolt Baked Mesh Collider: the render geometry on entity '%s' cannot be decomposed (it is flat or "
                "not triangulated). Bake it as a single hull instead.",
                GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>");
            return false;
        }

        AZ_Printf("JoltPhysics", "Jolt Baked Mesh Collider on entity '%s': decomposing %zu triangles into at most "
            "%u hulls...\n",
            GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>",
            indices.size() / 3, m_decompositionMaxHulls);

        m_decompositionSession = AZStd::move(session);
        m_reportedProgressDecile = -1;

        // The tick bus polls the run for progress and completion.
        if (!AZ::TickBus::Handler::BusIsConnected())
        {
            AZ::TickBus::Handler::BusConnect();
        }
        return false; // the bake itself lands in FinishDecompositionBake
    }

    void EditorJoltBakedMeshColliderComponent::FinishDecompositionBake()
    {
        AZStd::unique_ptr<EditorConvexDecomposition::DecompositionSession> session =
            AZStd::move(m_decompositionSession);
        const EditorConvexDecomposition::DecompositionResult result = session->TakeResult();

        const size_t hullCount = result.m_hulls.size();
        const AZStd::vector<AZ::u8> cookedData = JoltMeshUtils::PackConvexHulls(result.m_hulls);
        if (!cookedData.empty())
        {
            m_shapeConfiguration = Physics::CookedMeshShapeConfiguration();
            m_shapeConfiguration.SetCookedMeshData(cookedData.data(), cookedData.size(), m_meshType);
            m_debugLinesDirty = true;
            MarkBakedDataDirty();
            AZ_Printf("JoltPhysics", "Jolt Baked Mesh Collider on entity '%s' decomposed into %zu hulls (%zu KiB). "
                "Save the level to keep it.\n",
                GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>",
                hullCount, cookedData.size() / 1024);
        }
        else
        {
            // The geometry was there and VHACD still produced nothing; retrying every
            // tick would just burn worker threads, so give up until the user re-bakes.
            AZ_Warning("JoltPhysics", false,
                "Jolt Baked Mesh Collider: convex decomposition produced no hulls on entity '%s'.",
                GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>");
        }
        AZ::TickBus::Handler::BusDisconnect();
    }

    void EditorJoltBakedMeshColliderComponent::ReportDecompositionProgress()
    {
        // One line per 10%: VHACD reports far more often than that, and a per-tick line
        // would bury whatever else the console is saying.
        const int decile = static_cast<int>(m_decompositionSession->GetProgress()) / 10;
        if (decile <= m_reportedProgressDecile)
        {
            return;
        }
        m_reportedProgressDecile = decile;

        AZ_Printf("JoltPhysics", "Jolt Baked Mesh Collider on entity '%s': decomposing, %d%%...\n",
            GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>",
            decile * 10);
    }

    void EditorJoltBakedMeshColliderComponent::CancelDecompositionJob()
    {
        if (!m_decompositionSession)
        {
            return;
        }

        if (!m_decompositionSession->IsFinished())
        {
            AZ_Printf("JoltPhysics", "Jolt Baked Mesh Collider on entity '%s': decomposition canceled at %.0f%%.\n",
                GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>",
                m_decompositionSession->GetProgress());
        }

        // The destructor signals VHACD and waits for its thread, so the work stops here
        // rather than running on to a result nobody will read.
        m_decompositionSession.reset();
        m_reportedProgressDecile = -1;
    }

    AZ::u32 EditorJoltBakedMeshColliderComponent::OnBakeButtonPressed()
    {
        if (BakeFromRenderMesh(/*warnOnFailure*/ true))
        {
            // Success used to be silent, which reads as the button doing nothing when
            // the mesh happens to be off screen. Say what happened, and mark the entity
            // dirty so saving the level actually keeps the result.
            MarkBakedDataDirty();
            AZ_Printf("JoltPhysics", "Jolt Baked Mesh Collider on entity '%s' baked its collision mesh (%zu KiB). "
                "Save the level to keep it.\n",
                GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>",
                m_shapeConfiguration.GetCookedMeshData().size() / 1024);
            AZ::TickBus::Handler::BusDisconnect();
        }
        else if (m_decompositionSession)
        {
            // Decomposed mode only starts the run here; progress and completion land on
            // tick (StartDecompositionBake has already said it started).
        }
        return AZ::Edit::PropertyRefreshLevels::AttributesAndValues;
    }

    AZ::u32 EditorJoltBakedMeshColliderComponent::OnBakingSettingsChanged()
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

    void EditorJoltBakedMeshColliderComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        AZ_Warning("JoltPhysics", !m_shapeConfiguration.GetCookedMeshData().empty(),
            "Jolt Baked Mesh Collider on entity '%s' has no baked collision mesh; the runtime collider will be empty. "
            "Press 'Bake from render mesh' in the editor.",
            GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>");

        if (auto* component = gameEntity->CreateComponent<JoltBakedMeshColliderComponent>())
        {
            component->GetColliderConfiguration() = m_colliderConfiguration;
            component->GetShapeConfiguration() = m_shapeConfiguration;
        }
    }

    void EditorJoltBakedMeshColliderComponent::RebuildDebugLines() const
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

    AZ::Aabb EditorJoltBakedMeshColliderComponent::GetLocalShapeBounds() const
    {
        // The baked triangles are the only description of this collider's extent, so the
        // same pass that builds the wireframe supplies what the viewport picks against.
        if (m_debugLinesDirty)
        {
            RebuildDebugLines();
        }
        return m_debugBounds;
    }

    void EditorJoltBakedMeshColliderComponent::DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const
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
