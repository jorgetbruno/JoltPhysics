#include <Material/JoltMaterialManager.h>
#include <Material/JoltMaterial.h>

#include <AzCore/Asset/AssetManager.h>
#include <AzCore/Interface/Interface.h>

#include <AzFramework/Physics/Shape.h>

namespace JoltPhysics
{
    AZStd::shared_ptr<Physics::Material> JoltMaterialManager::CreateDefaultMaterialInternal()
    {
        // In-memory default material asset with the default property values,
        // mirroring PhysX's MaterialManager::CreateDefaultMaterialInternal.
        const Physics::MaterialAsset::MaterialProperties defaultProperties = {
            { "DynamicFriction", JoltMaterial::DefaultFriction },
            { "StaticFriction", JoltMaterial::DefaultFriction },
            { "Restitution", JoltMaterial::DefaultRestitution },
            { "Density", JoltMaterial::DefaultDensity },
        };

        AZ::Data::Asset<Physics::MaterialAsset> defaultAsset =
            AZ::Data::AssetManager::Instance().CreateAsset<Physics::MaterialAsset>(
                AZ::Data::AssetId(AZ::Uuid::CreateRandom()));
        defaultAsset->SetData("JoltMaterial", 1, defaultProperties);

        return CreateMaterialInternal(Physics::MaterialId::CreateFromAssetId(defaultAsset.GetId()), defaultAsset);
    }

    AZStd::shared_ptr<Physics::Material> JoltMaterialManager::CreateMaterialInternal(
        const Physics::MaterialId& id, const AZ::Data::Asset<Physics::MaterialAsset>& materialAsset)
    {
        return AZStd::make_shared<JoltMaterial>(id, materialAsset);
    }

    AZStd::pair<float, float> JoltMaterialManager::ResolveFrictionRestitution(
        const Physics::ColliderConfiguration& colliderConfiguration)
    {
        auto* materialManager = AZ::Interface<Physics::MaterialManager>::Get();
        if (!materialManager)
        {
            return { JoltMaterial::DefaultFriction, JoltMaterial::DefaultRestitution };
        }

        AZStd::shared_ptr<Physics::Material> material;

        const Physics::MaterialSlots& materialSlots = colliderConfiguration.m_materialSlots;
        if (materialSlots.GetSlotsCount() > 0)
        {
            const AZ::Data::Asset<Physics::MaterialAsset> materialAsset = materialSlots.GetMaterialAsset(0);
            if (materialAsset.GetId().IsValid())
            {
                material = materialManager->FindOrCreateMaterial(
                    Physics::MaterialId::CreateFromAssetId(materialAsset.GetId()), materialAsset);
            }
        }

        if (!material)
        {
            material = materialManager->GetDefaultMaterial();
        }

        if (const auto* joltMaterial = azrtti_cast<const JoltMaterial*>(material.get()))
        {
            return { joltMaterial->GetDynamicFriction(), joltMaterial->GetRestitution() };
        }

        return { JoltMaterial::DefaultFriction, JoltMaterial::DefaultRestitution };
    }

} // namespace JoltPhysics
