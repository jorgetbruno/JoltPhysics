#pragma once

#include <Clients/Components/JoltColliderComponentBase.h>

#include <AzCore/Asset/AssetCommon.h>

#include <AzFramework/Physics/ShapeConfiguration.h>

#include <Pipeline/JoltMeshAsset.h>

namespace JoltPhysics
{
    //! Expands a loaded .joltmesh asset into one collider/shape pair per asset shape.
    //! Each pair starts from the component's collider configuration, then takes the
    //! shape's own material slot and any per-shape overrides the Scene Builder stored
    //! in the asset (collision layer, offset, tag, ...). One pair per shape - rather
    //! than a single PhysicsAsset pair - is what lets the rigid body compound them
    //! into one body with each shape at its own offset (see
    //! JoltShapeUtils::CreateJoltShapeFromVariant).
    //!
    //! overallScale is baked into both the cloned shape configuration and the collider
    //! offset: native Jolt shapes carry no entity transform, so scale has to live in
    //! the shape (mirrors PhysX's Utils::GetColliderShapeConfigsFromAsset). Callers
    //! pass entityScale * assetScale; the editor passes only nonUniformScale * assetScale
    //! because its draw transform already applies the entity's uniform scale.
    AzPhysics::ShapeColliderPairList ExpandJoltMeshAssetColliderShapes(
        const Pipeline::JoltMeshAssetData& assetData,
        const Physics::ColliderConfiguration& colliderConfiguration,
        const AZ::Vector3& overallScale);

    //! Copies the asset's material slots into the collider configuration. With
    //! useMaterialsFromAsset the slots and their assigned materials are taken wholesale;
    //! without it only the slot names are copied, keeping whatever materials the user
    //! assigned on the component (mirrors PhysX's Utils::SetMaterialsFromPhysicsAssetShape).
    void ApplyJoltMeshAssetMaterialSlots(
        const Pipeline::JoltMeshAssetData& assetData,
        bool useMaterialsFromAsset,
        Physics::MaterialSlots& materialSlots);

    //! Component that provides the collider shapes of a .joltmesh physics asset (cooked
    //! from a source scene by the Scene Builder; authored in the editor through
    //! EditorJoltMeshColliderComponent) for the Jolt physics backend. The asset's
    //! shapes are expanded into one collider/shape pair each, so a Jolt Rigid Body or
    //! Jolt Static Rigid Body on the entity compounds them into a single body.
    //! Triangle-mesh shapes in the asset are static-geometry only; use convex exports
    //! for dynamic bodies.
    class JoltMeshColliderComponent
        : public JoltColliderComponentBase
        , private AZ::Data::AssetBus::MultiHandler
    {
    public:
        AZ_COMPONENT(JoltMeshColliderComponent, "{3A069B43-1B63-4ECD-A382-B5618C1D97C3}", JoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // JoltColliderComponentBase
        AzPhysics::ShapeColliderPair GetShapeColliderPair() const override;
        AzPhysics::ShapeColliderPairList GetShapeColliderPairs() const override;

        Physics::PhysicsAssetShapeConfiguration& GetShapeConfiguration()
        {
            return *m_shapeConfig;
        }
        const Physics::PhysicsAssetShapeConfiguration& GetShapeConfiguration() const
        {
            return *m_shapeConfig;
        }

    private:
        // AZ::Data::AssetBus::MultiHandler
        void OnAssetReady(AZ::Data::Asset<AZ::Data::AssetData> asset) override;
        void OnAssetReloaded(AZ::Data::Asset<AZ::Data::AssetData> asset) override;

        //! Pushes the asset's material slots into the collider configuration when the
        //! asset says to use them. Called once the asset is loaded (and on reload), so
        //! the slots exist before the rigid body builds its shapes from them.
        void ApplyMaterialSlotsFromAsset();

        AZStd::shared_ptr<Physics::PhysicsAssetShapeConfiguration> m_shapeConfig =
            AZStd::make_shared<Physics::PhysicsAssetShapeConfiguration>();
    };
} // namespace JoltPhysics
