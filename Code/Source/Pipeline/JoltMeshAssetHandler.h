#pragma once

#include <AzCore/Asset/AssetManager.h>
#include <AzCore/Asset/AssetTypeInfoBus.h>
#include <AzCore/RTTI/ReflectContext.h>

namespace JoltPhysics
{
    namespace Pipeline
    {
        //! Asset handler for loading and initializing JoltMeshAsset assets.
        //!
        //! Unlike PhysX's MeshAssetHandler (which registers itself from its constructor),
        //! registration here is explicit: the system component decides when to Register/Unregister
        //! so that activation and deactivation stay symmetric. Register/UnRegister calls are
        //! reference-counted by a flag, so calling either twice is harmless, and the destructor
        //! unregisters whatever is still registered.
        class JoltMeshAssetHandler
            : public AZ::Data::AssetHandler
            , private AZ::AssetTypeInfoBus::Handler
        {
        public:
            static const char* s_assetFileExtension;

            AZ_CLASS_ALLOCATOR(JoltMeshAssetHandler, AZ::SystemAllocator);

            JoltMeshAssetHandler() = default;
            ~JoltMeshAssetHandler() override;

            void Register();
            void Unregister();

            // AZ::Data::AssetHandler
            AZ::Data::AssetPtr CreateAsset(const AZ::Data::AssetId& id, const AZ::Data::AssetType& type) override;
            AZ::Data::AssetHandler::LoadResult LoadAssetData(
                const AZ::Data::Asset<AZ::Data::AssetData>& asset,
                AZStd::shared_ptr<AZ::Data::AssetDataStream> stream,
                const AZ::Data::AssetFilterCB& assetLoadFilterCB) override;
            void DestroyAsset(AZ::Data::AssetPtr ptr) override;
            void GetHandledAssetTypes(AZStd::vector<AZ::Data::AssetType>& assetTypes) override;

            // AZ::AssetTypeInfoBus
            AZ::Data::AssetType GetAssetType() const override;
            void GetAssetTypeExtensions(AZStd::vector<AZStd::string>& extensions) override;
            const char* GetAssetTypeDisplayName() const override;
            const char* GetBrowserIcon() const override;
            const char* GetGroup() const override;
            AZ::Uuid GetComponentTypeId() const override;
            bool CanCreateComponent(const AZ::Data::AssetId& assetId) const override;

        private:
            bool m_registered = false;
        };
    } // namespace Pipeline
} // namespace JoltPhysics
