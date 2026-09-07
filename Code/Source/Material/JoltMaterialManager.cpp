#include <AzCore/std/parallel/atomic.h>
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

    AZStd::vector<AZStd::shared_ptr<Physics::Material>> JoltMaterialManager::ResolveMaterialSlots(
        const Physics::ColliderConfiguration& colliderConfiguration)
    {
        const size_t slotCount = colliderConfiguration.m_materialSlots.GetSlotsCount();

        AZStd::vector<AZStd::shared_ptr<Physics::Material>> slotMaterials;

        // A collider with one slot - which is nearly all of them - needs no table: every
        // slot index clamps to that one material, which is exactly what GetSlot falls back
        // to when this is empty. Building a one-element vector for it cost a heap
        // allocation per collider on the body-creation path, measured at 0.8 us against
        // the 0.03 us the material lookup itself takes. Only a mesh painted with two or
        // more materials needs the table, and only that case had the race it exists for.
        if (slotCount <= 1)
        {
            return slotMaterials;
        }

        slotMaterials.reserve(slotCount);
        for (size_t slotIndex = 0; slotIndex < slotCount; ++slotIndex)
        {
            slotMaterials.push_back(ResolveMaterial(colliderConfiguration, slotIndex));
        }
        return slotMaterials;
    }

    AZStd::shared_ptr<Physics::Material> JoltMaterialManager::ResolveMaterial(
        const Physics::ColliderConfiguration& colliderConfiguration, size_t slotIndex)
    {
        return ResolveMaterialFromSlots(colliderConfiguration.m_materialSlots, slotIndex);
    }

    AZStd::shared_ptr<Physics::Material> JoltMaterialManager::ResolveMaterialFromSlots(
        const Physics::MaterialSlots& materialSlots, size_t slotIndex)
    {
        auto* materialManager = AZ::Interface<Physics::MaterialManager>::Get();
        if (!materialManager)
        {
            return nullptr;
        }

        if (materialSlots.GetSlotsCount() > 0)
        {
            const AZ::Data::Asset<Physics::MaterialAsset> materialAsset =
                materialSlots.GetMaterialAsset(AZStd::min(slotIndex, materialSlots.GetSlotsCount() - 1));
            if (materialAsset.GetId().IsValid())
            {
                if (auto material = materialManager->FindOrCreateMaterial(
                        Physics::MaterialId::CreateFromAssetId(materialAsset.GetId()), materialAsset))
                {
                    return material;
                }
            }
        }

        return materialManager->GetDefaultMaterial();
    }

    namespace
    {
        //! Process-wide: materials are shared across scenes, so one counter serves them.
        AZStd::atomic<AZ::u32> g_materialGeneration{ 1 };
    }

    AZ::u32 JoltMaterialManager::GetMaterialGeneration()
    {
        return g_materialGeneration.load(AZStd::memory_order_relaxed);
    }

    void JoltMaterialManager::BumpMaterialGeneration()
    {
        g_materialGeneration.fetch_add(1, AZStd::memory_order_relaxed);
    }

    AZStd::pair<float, float> JoltMaterialManager::GetFrictionRestitution(const Physics::Material* material)
    {
        if (const auto* joltMaterial = azrtti_cast<const JoltMaterial*>(material))
        {
            return { joltMaterial->GetDynamicFriction(), joltMaterial->GetRestitution() };
        }
        return { JoltMaterial::DefaultFriction, JoltMaterial::DefaultRestitution };
    }

    AZStd::pair<float, float> JoltMaterialManager::ResolveFrictionRestitution(
        const Physics::ColliderConfiguration& colliderConfiguration)
    {
        return GetFrictionRestitution(ResolveMaterial(colliderConfiguration).get());
    }

} // namespace JoltPhysics
