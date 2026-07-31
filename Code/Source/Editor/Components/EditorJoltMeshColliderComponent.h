#pragma once

#include <Editor/Components/EditorJoltColliderComponentBase.h>

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/NonUniformScaleBus.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

#include <AzFramework/Physics/ShapeConfiguration.h>

#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentBus.h>

#include <Pipeline/JoltMeshAsset.h>

namespace JoltPhysics
{
    //! Edit-context wrapper for the .joltmesh asset and the asset-specific shape
    //! settings (mirrors PhysX's EditorProxyPhysicsAsset). The asset lives here -
    //! rather than only inside the shape configuration - because the engine's own
    //! PhysicsAssetShapeConfiguration edit context deliberately hides its asset field,
    //! so binding the picker to it would render no picker at all. The configuration
    //! copy below carries the rest (asset scale, use-materials-from-asset) and is what
    //! BuildGameEntity hands to the runtime component.
    struct EditorProxyJoltMeshAsset
    {
        AZ_CLASS_ALLOCATOR(EditorProxyJoltMeshAsset, AZ::SystemAllocator);
        AZ_TYPE_INFO(EditorProxyJoltMeshAsset, "{09FA8317-C805-4123-8F5D-24A3CE0D8C56}");

        static void Reflect(AZ::ReflectContext* context);

        // Typed (not AssetData) so the inspector's asset picker filters to .joltmesh assets.
        AZ::Data::Asset<Pipeline::JoltMeshAsset> m_asset{ AZ::Data::AssetLoadBehavior::QueueLoad };
        Physics::PhysicsAssetShapeConfiguration m_configuration;
    };

    //! Editor Jolt Mesh Collider: references a .joltmesh physics asset (cooked
    //! from a source scene by the Asset Processor), draws its shapes in the Edit
    //! viewport, and spawns the runtime JoltMeshColliderComponent via
    //! BuildGameEntity. When no asset is assigned and the entity has a Mesh component,
    //! the collider auto-assigns the .joltmesh asset produced from the render mesh's
    //! source scene once that model finishes loading.
    class EditorJoltMeshColliderComponent
        : public EditorJoltColliderComponentBase
        , private AZ::Data::AssetBus::Handler
        , private AZ::Render::MeshComponentNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(EditorJoltMeshColliderComponent, "{F3B6A8D6-CE7C-4C21-B12F-9D9C09F7AECB}", EditorJoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    protected:
        // EditorJoltColliderComponentBase
        void DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const override;
        AZ::Aabb GetLocalShapeBounds() const override;
        AzPhysics::ShapeColliderPairList GetEditorShapeColliderPairs() const override;

        // AzToolsFramework::EditorComponentSelectionRequestsBus - the cached lines are
        // already in entity space (each shape's collider offset is baked in), so the
        // base's transform (world * collider offset) would apply the offset twice.
        AZ::Aabb GetEditorSelectionBoundsViewport(const AzFramework::ViewportInfo& viewportInfo) override;

    private:
        // AZ::Data::AssetBus::Handler - single handler: this component watches exactly
        // one asset, the one it references.
        void OnAssetReady(AZ::Data::Asset<AZ::Data::AssetData> asset) override;
        void OnAssetReloaded(AZ::Data::Asset<AZ::Data::AssetData> asset) override;

        // AZ::Render::MeshComponentNotificationBus
        void OnModelReady(const AZ::Data::Asset<AZ::RPI::ModelAsset>& modelAsset,
            const AZ::Data::Instance<AZ::RPI::Model>& model) override;

        //! (Re)connects the asset bus and queues the load for the referenced asset.
        //! The editor must not BlockUntilLoadComplete the way the runtime component
        //! does: activation happens while the Asset Processor may still be cooking,
        //! and stalling the editor main thread on it would hang level opens.
        void UpdateMeshAsset();

        //! Copies the asset's material slots into the collider configuration and marks
        //! the entity dirty, so the slots persist into the level (same reasoning as
        //! PhysX's UpdateMaterialSlotsFromMeshAsset: without the dirty mark the prefab
        //! saves the stale slot list).
        void UpdateMaterialSlotsFromMeshAsset();

        //! Recomputes the read-only "Contents" row from the loaded asset.
        void UpdateContentLabel();

        AZ::u32 OnShapeConfigurationChanged();

        // Auto-assignment (PhysX SetCollisionMeshFromRender parity): only ever fills in
        // an empty asset field, never overwrites a user's choice.
        bool ShouldUpdateCollisionMeshFromRender() const;
        void SetCollisionMeshFromRender();
        void AssignMeshAsset(const AZ::Data::AssetId& assetId);

        //! Picks the produced .joltmesh asset whose file name matches the render mesh's,
        //! for source scenes that emit more than one.
        AZ::Data::AssetId FindMatchingJoltMeshAsset(
            const AZStd::string& renderMeshHint,
            const AZStd::vector<AZ::Data::AssetId>& joltMeshAssets) const;

        //! Rebuilds the viewport line list from the asset (expanding it exactly like
        //! the runtime component does) and the bounds the editor picks against.
        void RebuildDebugLines() const;

        EditorProxyJoltMeshAsset m_proxyShapeConfiguration;

        //! Read-only inspector summary of the loaded asset ("2 convex, 1 triangle
        //! mesh"). Serialized only because edit-context rows must be backed by a
        //! serialized member; recomputed whenever the asset state changes.
        AZStd::string m_contentLabel;

        AZ::NonUniformScaleChangedEvent::Handler m_nonUniformScaleChangedHandler;

        //! Edge list (point pairs, entity-local space) of every shape in the asset.
        mutable AZStd::vector<AZ::Vector3> m_debugLines;
        mutable AZStd::vector<AZ::Vector3> m_debugLinesWorld; //!< Per-frame transformed scratch.
        mutable AZ::Aabb m_debugBounds = AZ::Aabb::CreateNull(); //!< Bounds of the same vertices.
        mutable bool m_debugLinesDirty = true;
    };
} // namespace JoltPhysics
