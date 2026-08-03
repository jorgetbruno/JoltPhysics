#include <Editor/Components/EditorJoltMeshColliderComponent.h>

#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Color.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/StringFunc/StringFunc.h>

#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/Components/JoltMeshColliderComponent.h>
#include <Editor/Components/EditorJoltColliderGeometryUtils.h>
#include <Editor/Components/JoltColliderOffsetComponentMode.h>
#include <Shape/JoltShapeUtils.h>
#include <Utils/ReflectionUtils.h>

#include <AzCore/Asset/AssetSerializer.h>

#include <JoltPhysics/Pipeline/JoltMeshAsset.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace JoltPhysics
{
    void EditorProxyJoltMeshAsset::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            // The asset field below needs the generic Asset<T> instantiation registered
            // for our asset type (the engine only registers the PhysX mesh asset's).
            serializeContext->RegisterGenericType<AZ::Data::Asset<Pipeline::JoltMeshAsset>>();

            serializeContext->Class<EditorProxyJoltMeshAsset>()
                ->Version(1)
                ->Field("Asset", &EditorProxyJoltMeshAsset::m_asset)
                ->Field("Configuration", &EditorProxyJoltMeshAsset::m_configuration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorProxyJoltMeshAsset>("EditorProxyJoltMeshAsset", "Jolt mesh asset.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorProxyJoltMeshAsset::m_asset,
                        "Jolt Mesh", "The .joltmesh physics asset this collider uses, cooked from a source scene by the Asset Processor.")
                        ->Attribute(AZ_CRC_CE("EditButton"), "")
                        ->Attribute(AZ_CRC_CE("EditDescription"), "Open in Scene Settings")
                        ->Attribute(AZ_CRC_CE("DisableEditButtonWhenNoAssetSelected"), true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorProxyJoltMeshAsset::m_configuration,
                        "Configuration", "Jolt mesh asset collider configuration.")
                        ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                    ;
            }
        }
    }

    void EditorJoltMeshColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltColliderComponentBase::Reflect(context);
        Internal::ReflectOnce<Physics::ShapeConfiguration>(context);
        Internal::ReflectOnce<Physics::PhysicsAssetShapeConfiguration>(context);
        Internal::ReflectOnce<JoltColliderOffsetComponentMode>(context);
        EditorProxyJoltMeshAsset::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltMeshColliderComponent, EditorJoltColliderComponentBase>()
                ->Version(1)
                ->Field("ShapeConfiguration", &EditorJoltMeshColliderComponent::m_proxyShapeConfiguration)
                ->Field("ContentLabel", &EditorJoltMeshColliderComponent::m_contentLabel)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltMeshColliderComponent>(
                    "Jolt Mesh Collider",
                    "Collider that references a .joltmesh physics asset cooked from a source scene by the Asset Processor "
                    "(Jolt physics backend, editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltMeshColliderComponent::m_proxyShapeConfiguration,
                        "Shape Configuration", "The .joltmesh asset and its shape settings.")
                        ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltMeshColliderComponent::OnShapeConfigurationChanged)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltMeshColliderComponent::m_contentLabel,
                        "Contents", "Collider shapes inside the loaded .joltmesh asset.")
                        ->Attribute(AZ::Edit::Attributes::ReadOnly, true)
                    ;
            }
        }
    }

    void EditorJoltMeshColliderComponent::Activate()
    {
        EditorJoltColliderComponentBase::Activate();

        m_debugLinesDirty = true;
        UpdateContentLabel();

        // Only the NonUniformScale component needs a listener: the uniform scale is
        // applied by the world transform at draw time, but the non-uniform part is
        // baked into the cached wireframe (same split as the runtime expansion).
        m_nonUniformScaleChangedHandler = AZ::NonUniformScaleChangedEvent::Handler(
            [this]([[maybe_unused]] const AZ::Vector3& nonUniformScale)
            {
                m_debugLinesDirty = true;
            });
        AZ::NonUniformScaleRequestBus::Event(
            GetEntityId(), &AZ::NonUniformScaleRequests::RegisterScaleChangedEvent, m_nonUniformScaleChangedHandler);

        // Connecting fires OnModelReady immediately when the render mesh is already
        // loaded; that event drives the .joltmesh auto-assignment.
        AZ::Render::MeshComponentNotificationBus::Handler::BusConnect(GetEntityId());

        // The offset is the one thing about a cooked collider a handle can express, and
        // without a delegate there is no Edit button to reach it with.
        m_componentModeDelegate.ConnectWithSingleComponentMode<
            EditorJoltMeshColliderComponent, JoltColliderOffsetComponentMode>(
            AZ::EntityComponentIdPair(GetEntityId(), GetId()), this);

        UpdateMeshAsset();
    }

    void EditorJoltMeshColliderComponent::Deactivate()
    {
        AZ::Data::AssetBus::Handler::BusDisconnect();
        AZ::Render::MeshComponentNotificationBus::Handler::BusDisconnect();
        m_nonUniformScaleChangedHandler.Disconnect();
        EditorJoltColliderComponentBase::Deactivate();
    }

    void EditorJoltMeshColliderComponent::UpdateMeshAsset()
    {
        // Disconnect first: the user may have swapped the asset for a different one.
        AZ::Data::AssetBus::Handler::BusDisconnect();
        m_proxyShapeConfiguration.m_configuration.m_asset = m_proxyShapeConfiguration.m_asset;

        if (m_proxyShapeConfiguration.m_asset.GetId().IsValid())
        {
            AZ::Data::AssetBus::Handler::BusConnect(m_proxyShapeConfiguration.m_asset.GetId());
            m_proxyShapeConfiguration.m_asset.QueueLoad();
        }

        m_debugLinesDirty = true;
        UpdateContentLabel();

        RebuildEditorCollider();
    }

    void EditorJoltMeshColliderComponent::OnAssetError(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        if (asset != m_proxyShapeConfiguration.m_asset)
        {
            return;
        }

        // Renaming a mesh group in Scene Settings orphans the references to it (see the
        // asset pipeline section of DIVERGENCES), so this is a failure authors reach by
        // doing something ordinary - and it used to leave no trace at all: a permanent
        // "Loading..." in the inspector and a body exported with no collision.
        m_assetFailedToLoad = true;
        m_contentLabel = "Failed to load - reprocess or reassign the asset";
        AZ_Warning("JoltPhysics", false,
            "Mesh collider on entity '%s': the .joltmesh asset '%s' failed to load, so this collider has no "
            "geometry. The asset may have failed to process, or the mesh group it came from was renamed or "
            "removed - check the Asset Processor and reassign the asset.",
            GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>",
            m_proxyShapeConfiguration.m_asset.GetHint().c_str());

        m_debugLinesDirty = true;
        AzToolsFramework::ToolsApplicationEvents::Bus::Broadcast(
            &AzToolsFramework::ToolsApplicationEvents::InvalidatePropertyDisplay,
            AzToolsFramework::Refresh_EntireTree);

        RebuildEditorCollider();
    }

    void EditorJoltMeshColliderComponent::OnAssetReady(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        if (asset == m_proxyShapeConfiguration.m_asset)
        {
            m_proxyShapeConfiguration.m_asset = asset;
            m_proxyShapeConfiguration.m_configuration.m_asset = asset;
            m_assetFailedToLoad = false;

            UpdateMaterialSlotsFromMeshAsset();
            UpdateContentLabel();
            m_debugLinesDirty = true;

            // The Contents row and the material slots changed behind the property
            // editor's back (the asset finished loading), so repaint the component.
            AzToolsFramework::ToolsApplicationEvents::Bus::Broadcast(
                &AzToolsFramework::ToolsApplicationEvents::InvalidatePropertyDisplay,
                AzToolsFramework::Refresh_EntireTree);
        }
    
        RebuildEditorCollider();
    }

    void EditorJoltMeshColliderComponent::OnAssetReloaded(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        // Same handling as the initial load: slots, label and wireframe all follow
        // the asset data.
        OnAssetReady(asset);
    }

    void EditorJoltMeshColliderComponent::UpdateMaterialSlotsFromMeshAsset()
    {
        const auto* asset = m_proxyShapeConfiguration.m_asset.GetAs<Pipeline::JoltMeshAsset>();
        if (!m_proxyShapeConfiguration.m_asset.IsReady() || asset == nullptr)
        {
            return;
        }

        ApplyJoltMeshAssetMaterialSlots(
            asset->m_assetData,
            m_proxyShapeConfiguration.m_configuration.m_useMaterialsFromAsset,
            m_colliderConfiguration.m_materialSlots);
        m_colliderConfiguration.m_materialSlots.SetSlotsReadOnly(
            m_proxyShapeConfiguration.m_configuration.m_useMaterialsFromAsset);

        // The slots were rewritten from the asset, outside any property edit; without
        // the dirty mark the prefab would save the stale slot list and the asset's
        // materials would silently revert on the next level open.
        AzToolsFramework::ScopedUndoBatch undoBatch("Jolt mesh asset collider material slots updated");
        AzToolsFramework::ScopedUndoBatch::MarkEntityDirty(GetEntityId());
    }

    void EditorJoltMeshColliderComponent::UpdateContentLabel()
    {
        if (!m_proxyShapeConfiguration.m_asset.GetId().IsValid())
        {
            m_contentLabel = "No asset assigned";
            return;
        }

        const auto* asset = m_proxyShapeConfiguration.m_asset.GetAs<Pipeline::JoltMeshAsset>();
        if (!m_proxyShapeConfiguration.m_asset.IsReady() || asset == nullptr)
        {
            // "Loading..." is only true while it might still arrive; past an error it is
            // the message that hides the problem.
            m_contentLabel = m_assetFailedToLoad ? "Failed to load - reprocess or reassign the asset" : "Loading...";
            return;
        }

        size_t convexCount = 0;
        size_t triangleMeshCount = 0;
        size_t primitiveCount = 0;
        for (const Pipeline::JoltMeshAssetData::ShapeConfigurationPair& shapeEntry : asset->m_assetData.m_colliderShapes)
        {
            const Physics::ShapeConfiguration* shapeConfig = shapeEntry.second.get();
            if (shapeConfig == nullptr)
            {
                continue;
            }
            if (shapeConfig->GetShapeType() == Physics::ShapeType::CookedMesh)
            {
                const auto* cookedMeshConfig = static_cast<const Physics::CookedMeshShapeConfiguration*>(shapeConfig);
                if (cookedMeshConfig->GetMeshType() == Physics::CookedMeshShapeConfiguration::MeshType::Convex)
                {
                    ++convexCount;
                }
                else
                {
                    ++triangleMeshCount;
                }
            }
            else
            {
                ++primitiveCount;
            }
        }

        m_contentLabel.clear();
        auto appendCount = [this](size_t count, const char* singular, const char* plural)
        {
            if (count == 0)
            {
                return;
            }
            if (!m_contentLabel.empty())
            {
                m_contentLabel += ", ";
            }
            m_contentLabel += AZStd::string::format("%zu %s", count, count == 1 ? singular : plural);
        };
        appendCount(convexCount, "convex", "convex");
        appendCount(triangleMeshCount, "triangle mesh", "triangle meshes");
        appendCount(primitiveCount, "primitive", "primitives");

        if (m_contentLabel.empty())
        {
            m_contentLabel = "No collider shapes";
        }
    }

    AZ::u32 EditorJoltMeshColliderComponent::OnShapeConfigurationChanged()
    {
        UpdateMeshAsset();
        return AZ::Edit::PropertyRefreshLevels::AttributesAndValues;
    }

    bool EditorJoltMeshColliderComponent::ShouldUpdateCollisionMeshFromRender() const
    {
        // Auto-assignment fills in an empty field only; a user-picked asset always wins.
        return !m_proxyShapeConfiguration.m_asset.GetId().IsValid();
    }

    void EditorJoltMeshColliderComponent::OnModelReady(
        [[maybe_unused]] const AZ::Data::Asset<AZ::RPI::ModelAsset>& modelAsset,
        [[maybe_unused]] const AZ::Data::Instance<AZ::RPI::Model>& model)
    {
        if (ShouldUpdateCollisionMeshFromRender())
        {
            SetCollisionMeshFromRender();
        }
    }

    void EditorJoltMeshColliderComponent::SetCollisionMeshFromRender()
    {
        AZ::Data::Asset<const AZ::RPI::ModelAsset> renderMeshAsset;
        AZ::Render::MeshComponentRequestBus::EventResult(
            renderMeshAsset, GetEntityId(), &AZ::Render::MeshComponentRequests::GetModelAsset);
        if (!renderMeshAsset.GetId().IsValid())
        {
            return; // no Mesh component (or no model set) on this entity
        }

        bool querySucceeded = false;
        AZStd::vector<AZ::Data::AssetInfo> productsInfo;
        AzToolsFramework::AssetSystemRequestBus::BroadcastResult(querySucceeded,
            &AzToolsFramework::AssetSystemRequestBus::Events::GetAssetsProducedBySourceUUID,
            renderMeshAsset.GetId().m_guid, productsInfo);
        if (!querySucceeded)
        {
            AZ_Warning("JoltPhysics", false,
                "Jolt Mesh Collider on entity '%s': could not query the assets produced by the render mesh's "
                "source scene; assign the .joltmesh asset manually.",
                GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>");
            return;
        }

        AZStd::vector<AZ::Data::AssetId> joltMeshAssets;
        for (const AZ::Data::AssetInfo& productInfo : productsInfo)
        {
            if (productInfo.m_assetType == azrtti_typeid<Pipeline::JoltMeshAsset>())
            {
                joltMeshAssets.push_back(productInfo.m_assetId);
            }
        }

        // One produced .joltmesh is unambiguous; with several, prefer the one named
        // after the render mesh (mirrors PhysX's SetCollisionMeshFromRender). None is
        // fine too: the source scene simply has no physics mesh group configured.
        if (joltMeshAssets.size() == 1)
        {
            AssignMeshAsset(joltMeshAssets.front());
        }
        else if (joltMeshAssets.size() > 1)
        {
            const AZ::Data::AssetId matchingAsset = FindMatchingJoltMeshAsset(renderMeshAsset.GetHint(), joltMeshAssets);
            if (matchingAsset.IsValid())
            {
                AssignMeshAsset(matchingAsset);
            }
            else
            {
                AZ_Warning("JoltPhysics", false,
                    "Jolt Mesh Collider on entity '%s': the render mesh's source scene produced several .joltmesh "
                    "assets and none matches the render mesh name; assign one manually.",
                    GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>");
            }
        }
    }

    AZ::Data::AssetId EditorJoltMeshColliderComponent::FindMatchingJoltMeshAsset(
        const AZStd::string& renderMeshHint, const AZStd::vector<AZ::Data::AssetId>& joltMeshAssets) const
    {
        AZStd::string renderMeshFileName;
        AzFramework::StringFunc::Path::Split(renderMeshHint.c_str(), nullptr, nullptr, &renderMeshFileName);

        for (const AZ::Data::AssetId& assetId : joltMeshAssets)
        {
            AZStd::string assetPath;
            AZ::Data::AssetCatalogRequestBus::BroadcastResult(
                assetPath, &AZ::Data::AssetCatalogRequests::GetAssetPathById, assetId);

            AZStd::string assetFileName;
            AzFramework::StringFunc::Path::Split(assetPath.c_str(), nullptr, nullptr, &assetFileName);

            if (assetFileName == renderMeshFileName)
            {
                return assetId;
            }
        }
        return AZ::Data::AssetId();
    }

    void EditorJoltMeshColliderComponent::AssignMeshAsset(const AZ::Data::AssetId& assetId)
    {
        m_proxyShapeConfiguration.m_asset.Create(assetId);
        UpdateMeshAsset();

        // The assignment happened outside the property editor; without the dirty mark
        // saving the level would drop it (same trap as the bake component's baked data).
        AzToolsFramework::ScopedUndoBatch undoBatch("Assign Jolt mesh asset");
        AzToolsFramework::ScopedUndoBatch::MarkEntityDirty(GetEntityId());
    }

    void EditorJoltMeshColliderComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        AZ_Warning("JoltPhysics", m_proxyShapeConfiguration.m_asset.GetId().IsValid(),
            "Jolt Mesh Collider on entity '%s' has no .joltmesh asset assigned; the runtime collider will be empty.",
            GetEntity() ? GetEntity()->GetName().c_str() : "<unknown>");

        if (auto* component = gameEntity->CreateComponent<JoltMeshColliderComponent>())
        {
            component->GetColliderConfiguration() = m_colliderConfiguration;
            // The runtime component owns its configuration; copy it, never share the
            // editor's serialized instance.
            component->GetShapeConfiguration() = m_proxyShapeConfiguration.m_configuration;
            component->GetShapeConfiguration().m_asset = m_proxyShapeConfiguration.m_asset;
        }
    }

    AzPhysics::ShapeColliderPairList EditorJoltMeshColliderComponent::GetEditorShapeColliderPairs() const
    {
        const auto* asset = m_proxyShapeConfiguration.m_asset.GetAs<Pipeline::JoltMeshAsset>();
        if (!m_proxyShapeConfiguration.m_asset.IsReady() || asset == nullptr)
        {
            return {};
        }

        // The full overall scale, uniform world scale included: the editor body's
        // transform carries no scale, unlike the wireframe draw path whose world
        // transform does (mirrors the runtime component's GetShapeColliderPairs).
        float uniformScale = 1.0f;
        AZ::TransformBus::EventResult(uniformScale, GetEntityId(), &AZ::TransformBus::Events::GetWorldUniformScale);
        AZ::Vector3 nonUniformScale = AZ::Vector3::CreateOne();
        AZ::NonUniformScaleRequestBus::EventResult(nonUniformScale, GetEntityId(), &AZ::NonUniformScaleRequests::GetScale);
        const AZ::Vector3 overallScale =
            nonUniformScale * uniformScale * m_proxyShapeConfiguration.m_configuration.m_assetScale;

        return ExpandJoltMeshAssetColliderShapes(asset->m_assetData, m_colliderConfiguration, overallScale);
    }

    void EditorJoltMeshColliderComponent::RebuildDebugLines() const
    {
        m_debugLines.clear();
        m_debugBounds = AZ::Aabb::CreateNull();
        m_debugLinesDirty = false;

        const auto* asset = m_proxyShapeConfiguration.m_asset.GetAs<Pipeline::JoltMeshAsset>();
        if (!m_proxyShapeConfiguration.m_asset.IsReady() || asset == nullptr)
        {
            return;
        }

        AZ::Vector3 nonUniformScale = AZ::Vector3::CreateOne();
        AZ::NonUniformScaleRequestBus::EventResult(nonUniformScale, GetEntityId(), &AZ::NonUniformScaleRequests::GetScale);

        // Expand exactly like the runtime component, minus the entity's uniform scale:
        // DrawShape transforms by the world transform, which already carries it.
        const AZ::Vector3 drawScale = nonUniformScale * m_proxyShapeConfiguration.m_configuration.m_assetScale;
        const AzPhysics::ShapeColliderPairList shapeColliderPairs =
            ExpandJoltMeshAssetColliderShapes(asset->m_assetData, m_colliderConfiguration, drawScale);

        AZStd::vector<AZ::Vector3> shapeLines;
        for (const AzPhysics::ShapeColliderPair& pair : shapeColliderPairs)
        {
            // Build the same composed native shape the runtime body gets - offset,
            // rotation, and the entity-space scale outside the rotation - so the
            // wireframe is the collision, not a re-derivation that can drift from it
            // (BuildShapeWireframe flattens the decorator chain via Jolt itself).
            const JPH::RefConst<JPH::Shape> shape =
                JoltShapeUtils::CreateJoltShapeFromPair(pair, GetEntity() ? GetEntity()->GetName() : AZStd::string());
            if (!shape)
            {
                continue;
            }

            AZ::Aabb shapeBounds = AZ::Aabb::CreateNull();
            EditorColliderGeometry::BuildShapeWireframe(shape.GetPtr(), shapeLines, shapeBounds);
            m_debugLines.insert(m_debugLines.end(), shapeLines.begin(), shapeLines.end());
            m_debugBounds.AddAabb(shapeBounds);
        }
    }

    AZ::Aabb EditorJoltMeshColliderComponent::GetLocalShapeBounds() const
    {
        if (m_debugLinesDirty)
        {
            RebuildDebugLines();
        }
        return m_debugBounds;
    }

    AZ::Aabb EditorJoltMeshColliderComponent::GetEditorSelectionBoundsViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo)
    {
        if (m_debugLinesDirty)
        {
            RebuildDebugLines();
        }
        if (!m_debugBounds.IsValid())
        {
            return AZ::Aabb::CreateNull();
        }

        // The cached bounds are entity-local with every shape's collider offset baked
        // in, so only the world transform applies (the base would add the component's
        // own offset a second time).
        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        return m_debugBounds.GetTransformedAabb(worldTransform);
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

        m_debugLinesWorld.resize(m_debugLines.size());
        for (size_t i = 0; i < m_debugLines.size(); ++i)
        {
            m_debugLinesWorld[i] = worldTransform.TransformPoint(m_debugLines[i]);
        }

        debugDisplay.DrawLines(m_debugLinesWorld, AZ::Color(0.0f, 1.0f, 0.0f, 1.0f));
    }

} // namespace JoltPhysics
