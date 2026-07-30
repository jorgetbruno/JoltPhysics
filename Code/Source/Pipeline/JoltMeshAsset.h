#pragma once

#include <AzCore/Asset/AssetManager.h>
#include <AzCore/Asset/AssetTypeInfoBus.h>
#include <AzCore/std/limits.h>
#include <AzCore/std/optional.h>
#include <AzFramework/Physics/Material/PhysicsMaterialSlots.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Physics/Collision/CollisionGroups.h>
#include <AzFramework/Physics/Collision/CollisionLayers.h>

namespace JoltPhysics
{
    namespace Pipeline
    {
        class JoltMeshAsset;

        //! Optional collider configuration data that is stored in the asset.
        //! All the fields here are optional. If you set them at the asset building stage
        //! that data will then be used in the collider.
        class JoltAssetColliderConfiguration
        {
        public:
            AZ_CLASS_ALLOCATOR(JoltAssetColliderConfiguration, AZ::SystemAllocator);
            AZ_TYPE_INFO(JoltAssetColliderConfiguration, "{8EC9D61B-9180-47C7-87C0-17E13C5A8358}");

            static void Reflect(AZ::ReflectContext* context);
            void UpdateColliderConfiguration(Physics::ColliderConfiguration& colliderConfiguration) const;

            AZStd::optional<AzPhysics::CollisionLayer> m_collisionLayer; //!< Which collision layer is this collider on.
            AZStd::optional<AzPhysics::CollisionGroups::Id> m_collisionGroupId; //!< Which layers does this collider collide with.
            AZStd::optional<bool> m_isTrigger; //!< Should this shape act as a trigger shape.
            AZStd::optional<AZ::Transform> m_transform; //!< Shape offset relative to the connected rigid body.
            AZStd::optional<AZStd::string> m_tag; //!< Identification tag for the collider.
        };

        //! Physics mesh asset data structure produced by the Scene Builder (.joltmesh products).
        class JoltMeshAssetData
        {
        public:
            AZ_CLASS_ALLOCATOR(JoltMeshAssetData, AZ::SystemAllocator);
            AZ_TYPE_INFO(JoltMeshAssetData, "{4E3329AA-3F48-4942-8D08-DC97AAA9901F}");

            static void Reflect(AZ::ReflectContext* context);

            //! Creates a mesh asset with a random Id from the
            //! properties of this mesh asset data.
            AZ::Data::Asset<JoltMeshAsset> CreateMeshAsset() const;

            // Reserved material index indicating that the cooked mesh itself stores the indices.
            // Wrapping the numeric_limits<AZ::u16>::max function in parenthesis to get around the issue with windows.h defining max as a macro.
            static constexpr AZ::u16 TriangleMeshMaterialIndex = (AZStd::numeric_limits<AZ::u16>::max)();

            using ShapeConfigurationPair = AZStd::pair<AZStd::shared_ptr<JoltAssetColliderConfiguration>,
                AZStd::shared_ptr<Physics::ShapeConfiguration>>; // Have to use shared_ptr here because AzPhysics::ShapeColliderPairList uses it
            using ShapeConfigurationList = AZStd::vector<ShapeConfigurationPair>;

            ShapeConfigurationList m_colliderShapes; //!< Shapes data with optional collider configuration override.
            Physics::MaterialSlots m_materialSlots; //!< List of material slots of the mesh asset.
            AZStd::vector<AZ::u16> m_materialIndexPerShape; //!< An index of the material in m_materialSlots for each shape.
        };

        //! Represents a Jolt mesh asset. This is an AZ::Data::AssetData wrapper around JoltMeshAssetData.
        class JoltMeshAsset
            : public AZ::Data::AssetData
        {
        public:
            AZ_CLASS_ALLOCATOR(JoltMeshAsset, AZ::SystemAllocator);
            AZ_RTTI(JoltMeshAsset, "{5D012381-E9E8-4327-8F31-8D3006A1E776}", AZ::Data::AssetData);

            static void Reflect(AZ::ReflectContext* context);

            //! Sets the mesh data for this mesh asset and marks it as ready.
            //! This is useful when creating an in-memory mesh asset.
            void SetData(const JoltMeshAssetData& assetData);

            JoltMeshAssetData m_assetData;
        };
    } // namespace Pipeline
} // namespace JoltPhysics
