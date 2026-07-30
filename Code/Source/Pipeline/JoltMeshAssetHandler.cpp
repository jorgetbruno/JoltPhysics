#include <Pipeline/JoltMeshAsset.h>
#include <Pipeline/JoltMeshAssetHandler.h>

#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/Utils.h>
#include <AzFramework/Physics/Material/PhysicsMaterialAsset.h>

namespace JoltPhysics
{
    namespace Pipeline
    {
        const char* JoltMeshAssetHandler::s_assetFileExtension = "joltmesh";

        JoltMeshAssetHandler::~JoltMeshAssetHandler()
        {
            // Safe even if Register was never called: Unregister is guarded by m_registered.
            Unregister();
        }

        void JoltMeshAssetHandler::Register()
        {
            if (m_registered)
            {
                return;
            }

            const bool assetManagerReady = AZ::Data::AssetManager::IsReady();
            AZ_Error("Jolt Mesh Asset", assetManagerReady, "Asset manager isn't ready.");
            if (assetManagerReady)
            {
                AZ::Data::AssetManager::Instance().RegisterHandler(this, AZ::AzTypeInfo<JoltMeshAsset>::Uuid());
            }

            AZ::AssetTypeInfoBus::Handler::BusConnect(AZ::AzTypeInfo<JoltMeshAsset>::Uuid());
            m_registered = true;
        }

        void JoltMeshAssetHandler::Unregister()
        {
            if (!m_registered)
            {
                return;
            }

            AZ::AssetTypeInfoBus::Handler::BusDisconnect();

            if (AZ::Data::AssetManager::IsReady())
            {
                AZ::Data::AssetManager::Instance().UnregisterHandler(this);
            }

            m_registered = false;
        }

        // AZ::AssetTypeInfoBus
        AZ::Data::AssetType JoltMeshAssetHandler::GetAssetType() const
        {
            return AZ::AzTypeInfo<JoltMeshAsset>::Uuid();
        }

        void JoltMeshAssetHandler::GetAssetTypeExtensions(AZStd::vector<AZStd::string>& extensions)
        {
            extensions.push_back(s_assetFileExtension);
        }

        const char* JoltMeshAssetHandler::GetAssetTypeDisplayName() const
        {
            return "Jolt Collision Mesh";
        }

        const char* JoltMeshAssetHandler::GetBrowserIcon() const
        {
            // The gem ships no Jolt-specific icon yet; an empty path makes the asset browser
            // fall back to its default icon.
            return "";
        }

        const char* JoltMeshAssetHandler::GetGroup() const
        {
            return "Physics";
        }

        // Disable spawning of physics asset entities on drag and drop
        AZ::Uuid JoltMeshAssetHandler::GetComponentTypeId() const
        {
            // NOTE: This doesn't do anything when CanCreateComponent returns false
            return AZ::Uuid::CreateNull();
        }

        bool JoltMeshAssetHandler::CanCreateComponent([[maybe_unused]] const AZ::Data::AssetId& assetId) const
        {
            return false;
        }

        // AZ::Data::AssetHandler
        AZ::Data::AssetPtr JoltMeshAssetHandler::CreateAsset([[maybe_unused]] const AZ::Data::AssetId& id, const AZ::Data::AssetType& type)
        {
            if (type == AZ::AzTypeInfo<JoltMeshAsset>::Uuid())
            {
                return aznew JoltMeshAsset();
            }

            AZ_Error("Jolt Mesh Asset", false, "This handler deals only with JoltMeshAsset type.");
            return nullptr;
        }

        // Fixes up an asset by querying the asset catalog for its id using its hint path.
        // Returns true if it was able to find the asset in the catalog and set the asset id.
        template<typename AssetType>
        static bool FixUpAssetIdByHint(AZ::Data::Asset<AssetType>& asset)
        {
            AZ::Data::AssetId assetId;
            AZ::Data::AssetCatalogRequestBus::BroadcastResult(
                assetId, &AZ::Data::AssetCatalogRequestBus::Events::GetAssetIdByPath, asset.GetHint().c_str(),
                AZ::Data::s_invalidAssetType, false);

            if (assetId.IsValid())
            {
                asset.Create(assetId, false);
                return true;
            }
            return false;
        }

        AZ::Data::AssetHandler::LoadResult JoltMeshAssetHandler::LoadAssetData(
            const AZ::Data::Asset<AZ::Data::AssetData>& asset,
            AZStd::shared_ptr<AZ::Data::AssetDataStream> stream,
            const AZ::Data::AssetFilterCB& assetLoadFilterCB)
        {
            JoltMeshAsset* meshAsset = asset.GetAs<JoltMeshAsset>();
            if (!meshAsset)
            {
                AZ_Error("Jolt Mesh Asset", false, "This should be a JoltMeshAsset, as this is the only type we process.");
                return AZ::Data::AssetHandler::LoadResult::Error;
            }

            AZ::SerializeContext* serializeContext = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(serializeContext, &AZ::ComponentApplicationRequests::GetSerializeContext);

            if (!AZ::Utils::LoadObjectFromStreamInPlace(
                *stream, meshAsset->m_assetData, serializeContext, AZ::ObjectStream::FilterDescriptor(assetLoadFilterCB)))
            {
                return AZ::Data::AssetHandler::LoadResult::Error;
            }

            // A Jolt mesh asset could have been saved with material assets that only have the hint valid
            // (the asset path in the cache). This could happen for example when the mesh group was filled
            // procedurally and the materials were only given the path (hint).
            // Now at runtime, after the Jolt mesh is loaded, let's complete the asset by looking for it
            // in the catalog and assigning its id. At runtime a physics material will always be in the
            // catalog because it's a critical asset.
            for (size_t slotId = 0; slotId < meshAsset->m_assetData.m_materialSlots.GetSlotsCount(); ++slotId)
            {
                AZ::Data::Asset<Physics::MaterialAsset> materialAsset = meshAsset->m_assetData.m_materialSlots.GetMaterialAsset(slotId);
                // Does it need to resolve its id from the hint path?
                if (!materialAsset.GetId().IsValid() && !materialAsset.GetHint().empty())
                {
                    if (FixUpAssetIdByHint(materialAsset))
                    {
                        meshAsset->m_assetData.m_materialSlots.SetMaterialAsset(slotId, materialAsset);
                    }
                    else
                    {
                        AZ_Warning("Jolt Mesh Asset", false,
                            "Loading Jolt Mesh '%s' it didn't find physics material '%s', assigned to slot '%.*s'. Default physics material will be used.",
                            asset.GetHint().c_str(),
                            materialAsset.GetHint().c_str(),
                            AZ_STRING_ARG(meshAsset->m_assetData.m_materialSlots.GetSlotName(slotId)));
                    }
                }
            }

            return AZ::Data::AssetHandler::LoadResult::LoadComplete;
        }

        void JoltMeshAssetHandler::DestroyAsset(AZ::Data::AssetPtr ptr)
        {
            delete ptr;
        }

        void JoltMeshAssetHandler::GetHandledAssetTypes(AZStd::vector<AZ::Data::AssetType>& assetTypes)
        {
            assetTypes.push_back(AZ::AzTypeInfo<JoltMeshAsset>::Uuid());
        }
    } // namespace Pipeline
} // namespace JoltPhysics
