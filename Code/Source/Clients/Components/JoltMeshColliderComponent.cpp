#include <Clients/Components/JoltMeshColliderComponent.h>

#include <AzCore/Component/NonUniformScaleBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <AzFramework/Physics/ColliderComponentBus.h>

#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    namespace
    {
        // The full entity scale: uniform scale from the transform hierarchy times the
        // optional NonUniformScale component (mirrors PhysX's Utils::GetOverallScale).
        // Colliders never see a transform with scale on it - the rigid body takes only
        // translation and rotation from the world transform - so the scale has to be
        // folded into the shape configurations themselves.
        AZ::Vector3 GetOverallEntityScale(AZ::EntityId entityId)
        {
            float uniformScale = 1.0f;
            AZ::TransformBus::EventResult(uniformScale, entityId, &AZ::TransformBus::Events::GetWorldUniformScale);

            AZ::Vector3 nonUniformScale = AZ::Vector3::CreateOne();
            AZ::NonUniformScaleRequestBus::EventResult(nonUniformScale, entityId, &AZ::NonUniformScaleRequests::GetScale);

            return nonUniformScale * uniformScale;
        }
    } // namespace

    AzPhysics::ShapeColliderPairList ExpandJoltMeshAssetColliderShapes(
        const Pipeline::JoltMeshAssetData& assetData,
        const Physics::ColliderConfiguration& colliderConfiguration,
        const AZ::Vector3& overallScale)
    {
        AzPhysics::ShapeColliderPairList expandedPairs;
        expandedPairs.reserve(assetData.m_colliderShapes.size());

        for (size_t shapeIndex = 0; shapeIndex < assetData.m_colliderShapes.size(); ++shapeIndex)
        {
            const Pipeline::JoltMeshAssetData::ShapeConfigurationPair& shapeEntry = assetData.m_colliderShapes[shapeIndex];
            if (!shapeEntry.second)
            {
                continue;
            }

            auto expandedColliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>(colliderConfiguration);

            // Every shape in the asset points at one of the component's material slots.
            // Give the pair a single default slot holding that shape's material, so the
            // native shape gets exactly one material regardless of how many slots the
            // component carries.
            if (shapeIndex < assetData.m_materialIndexPerShape.size() &&
                assetData.m_materialIndexPerShape[shapeIndex] != Pipeline::JoltMeshAssetData::TriangleMeshMaterialIndex)
            {
                expandedColliderConfig->m_materialSlots.SetSlots(Physics::MaterialDefaultSlot::Default);
                expandedColliderConfig->m_materialSlots.SetMaterialAsset(
                    0, colliderConfiguration.m_materialSlots.GetMaterialAsset(assetData.m_materialIndexPerShape[shapeIndex]));
            }
            // Triangle-mesh shapes are the exception: they keep the component's slot
            // list untouched. Their materials are meant to be assigned per face from
            // data cooked into the mesh; that per-face path is a follow-up, and until
            // it exists the whole slot list passes through as-is.

            // Per-shape data stored by the Scene Builder (collision layer, offset,
            // rotation, tag...) overrides the component's collider configuration.
            if (shapeEntry.first)
            {
                shapeEntry.first->UpdateColliderConfiguration(*expandedColliderConfig);
            }

            auto expandedShapeConfig = shapeEntry.second->Clone();
            expandedShapeConfig->m_scale = overallScale;

            // The collider offset is in unscaled entity space; scale it with the shape
            // so the pair stays coherent (mirrors PhysX's Utils::CreateShapesFromAsset).
            expandedColliderConfig->m_position *= overallScale;

            expandedPairs.emplace_back(AZStd::move(expandedColliderConfig), AZStd::move(expandedShapeConfig));
        }

        return expandedPairs;
    }

    void ApplyJoltMeshAssetMaterialSlots(
        const Pipeline::JoltMeshAssetData& assetData,
        bool useMaterialsFromAsset,
        Physics::MaterialSlots& materialSlots)
    {
        if (useMaterialsFromAsset)
        {
            // Slots and the material assets assigned to them.
            materialSlots = assetData.m_materialSlots;
        }
        else
        {
            // Slot names only, so the user's own material assignments survive.
            materialSlots.SetSlots(assetData.m_materialSlots.GetSlotsNames());
        }
    }

    void JoltMeshColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<JoltColliderComponentBase>(context);
        Internal::ReflectOnce<Physics::ShapeConfiguration>(context);
        Internal::ReflectOnce<Physics::PhysicsAssetShapeConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->RegisterGenericType<AZStd::shared_ptr<Physics::PhysicsAssetShapeConfiguration>>();

            serializeContext->Class<JoltMeshColliderComponent, JoltColliderComponentBase>()
                ->Version(1)
                ->Field("ShapeConfiguration", &JoltMeshColliderComponent::m_shapeConfig)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltMeshColliderComponent>(
                    "Jolt Mesh Collider", "Collider from a .joltmesh physics asset for the Jolt physics backend")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        // No AppearsInAddComponentMenu: the editor-side EditorJoltMeshColliderComponent
                        // owns the menu entry (PhysX-style editor/runtime split).
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::RemoveableByUser, true)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void JoltMeshColliderComponent::Activate()
    {
        if (m_shapeConfig->m_asset.GetId().IsValid())
        {
            AZ::Data::AssetBus::MultiHandler::BusConnect(m_shapeConfig->m_asset.GetId());
            m_shapeConfig->m_asset.QueueLoad();

            // The rigid body collects this collider's shape pairs when the entity
            // finishes activating (OnEntityActivated), and the expansion needs the
            // asset's shape list - so the load has to be finished by then. Blocking
            // here is what PhysX does for the same reason; without it the first body
            // would be built from an empty shape list and never corrected.
            m_shapeConfig->m_asset.BlockUntilLoadComplete();

            ApplyMaterialSlotsFromAsset();
        }

        JoltColliderComponentBase::Activate();
    }

    void JoltMeshColliderComponent::Deactivate()
    {
        AZ::Data::AssetBus::MultiHandler::BusDisconnect();
        JoltColliderComponentBase::Deactivate();
    }

    void JoltMeshColliderComponent::OnAssetReady(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        if (asset == m_shapeConfig->m_asset)
        {
            m_shapeConfig->m_asset = asset;
            ApplyMaterialSlotsFromAsset();
        }
    }

    void JoltMeshColliderComponent::OnAssetReloaded(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        if (asset == m_shapeConfig->m_asset)
        {
            m_shapeConfig->m_asset = asset;
            ApplyMaterialSlotsFromAsset();

            // The reloaded asset may carry different shapes, so any body built from the
            // old data is stale. The rigid body listens for this and rebuilds on the
            // next tick (never inside this dispatch).
            Physics::ColliderComponentEventBus::Event(
                GetEntityId(), &Physics::ColliderComponentEvents::OnColliderChanged);
        }
    }

    void JoltMeshColliderComponent::ApplyMaterialSlotsFromAsset()
    {
        if (const auto* asset = m_shapeConfig->m_asset.GetAs<Pipeline::JoltMeshAsset>())
        {
            ApplyJoltMeshAssetMaterialSlots(
                asset->m_assetData, m_shapeConfig->m_useMaterialsFromAsset, m_colliderConfiguration->m_materialSlots);
        }
    }

    AzPhysics::ShapeColliderPair JoltMeshColliderComponent::GetShapeColliderPair() const
    {
        return { m_colliderConfiguration, m_shapeConfig };
    }

    AzPhysics::ShapeColliderPairList JoltMeshColliderComponent::GetShapeColliderPairs() const
    {
        const auto* asset = m_shapeConfig->m_asset.GetAs<Pipeline::JoltMeshAsset>();
        if (!m_shapeConfig->m_asset.IsReady() || asset == nullptr || asset->m_assetData.m_colliderShapes.empty())
        {
            // No shapes to expand (asset missing, still loading, or cooked empty):
            // fall back to the single unexpanded pair, matching the base behavior.
            // Activate already blocked on the load, so this only happens when the
            // asset failed or genuinely has no collider shapes.
            return { GetShapeColliderPair() };
        }

        const AZ::Vector3 overallScale = GetOverallEntityScale(GetEntityId()) * m_shapeConfig->m_assetScale;
        return ExpandJoltMeshAssetColliderShapes(asset->m_assetData, *m_colliderConfiguration, overallScale);
    }
} // namespace JoltPhysics
