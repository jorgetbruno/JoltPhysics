#include <JoltPhysics/Pipeline/JoltMeshAsset.h>

#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>

namespace JoltPhysics
{
    namespace Pipeline
    {
        void JoltAssetColliderConfiguration::Reflect(AZ::ReflectContext* context)
        {
            if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
            {
                serializeContext->Class<JoltAssetColliderConfiguration>()
                    ->Version(1)
                    ->Field("CollisionLayer", &JoltAssetColliderConfiguration::m_collisionLayer)
                    ->Field("CollisionGroupId", &JoltAssetColliderConfiguration::m_collisionGroupId)
                    ->Field("isTrigger", &JoltAssetColliderConfiguration::m_isTrigger)
                    ->Field("Transform", &JoltAssetColliderConfiguration::m_transform)
                    ->Field("Tag", &JoltAssetColliderConfiguration::m_tag)
                    ;

                if (AZ::EditContext* editContext = serializeContext->GetEditContext())
                {
                    editContext->Class<JoltAssetColliderConfiguration>(
                        "Jolt Asset Collider Configuration", "Optional collider configuration overrides stored in a Jolt mesh asset.")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &JoltAssetColliderConfiguration::m_collisionLayer,
                            "Collision Layer", "Which collision layer is this collider on.")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &JoltAssetColliderConfiguration::m_collisionGroupId,
                            "Collision Group", "Which layers does this collider collide with.")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &JoltAssetColliderConfiguration::m_isTrigger,
                            "Is Trigger", "Should this shape act as a trigger shape.")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &JoltAssetColliderConfiguration::m_transform,
                            "Transform", "Shape offset relative to the connected rigid body.")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &JoltAssetColliderConfiguration::m_tag,
                            "Tag", "Identification tag for the collider.")
                        ;
                }
            }
        }

        void JoltAssetColliderConfiguration::UpdateColliderConfiguration(Physics::ColliderConfiguration& colliderConfiguration) const
        {
            if (m_collisionLayer)
            {
                colliderConfiguration.m_collisionLayer = *m_collisionLayer;
            }

            if (m_collisionGroupId)
            {
                colliderConfiguration.m_collisionGroupId = *m_collisionGroupId;
            }

            if (m_isTrigger)
            {
                colliderConfiguration.m_isTrigger = *m_isTrigger;
            }

            if (m_transform)
            {
                // Apply the local shape transform to the collider one
                AZ::Transform existingTransform =
                    AZ::Transform::CreateFromQuaternionAndTranslation(colliderConfiguration.m_rotation, colliderConfiguration.m_position);

                AZ::Transform shapeTransform = *m_transform;
                shapeTransform.ExtractUniformScale();

                shapeTransform = existingTransform * shapeTransform;

                colliderConfiguration.m_position = shapeTransform.GetTranslation();
                colliderConfiguration.m_rotation = shapeTransform.GetRotation();
                colliderConfiguration.m_rotation.Normalize();
            }

            if (m_tag)
            {
                colliderConfiguration.m_tag = *m_tag;
            }
        }

        void JoltMeshAssetData::Reflect(AZ::ReflectContext* context)
        {
            JoltAssetColliderConfiguration::Reflect(context);

            if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
            {
                serializeContext->Class<JoltMeshAssetData>()
                    ->Version(1)
                    ->Field("ColliderShapes", &JoltMeshAssetData::m_colliderShapes)
                    ->Field("MaterialSlots", &JoltMeshAssetData::m_materialSlots)
                    ->Field("MaterialIndexPerShape", &JoltMeshAssetData::m_materialIndexPerShape)
                    ;

                if (AZ::EditContext* editContext = serializeContext->GetEditContext())
                {
                    editContext->Class<JoltMeshAssetData>("Jolt Mesh Asset Data", "Shape and material data stored in a Jolt mesh asset.")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &JoltMeshAssetData::m_colliderShapes,
                            "Collider Shapes", "Shapes data with optional collider configuration override.")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &JoltMeshAssetData::m_materialSlots,
                            "Material Slots", "List of material slots of the mesh asset.")
                        ;
                }
            }
        }

        AZ::Data::Asset<JoltMeshAsset> JoltMeshAssetData::CreateMeshAsset() const
        {
            AZ::Data::Asset<JoltMeshAsset> meshAsset =
                AZ::Data::AssetManager::Instance().CreateAsset<JoltMeshAsset>(AZ::Data::AssetId(AZ::Uuid::CreateRandom()));

            meshAsset->SetData(*this);

            return meshAsset;
        }

        void JoltMeshAsset::Reflect(AZ::ReflectContext* context)
        {
            JoltMeshAssetData::Reflect(context);

            if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
            {
                serializeContext->Class<JoltMeshAsset, AZ::Data::AssetData>()
                    ->Field("MeshAssetData", &JoltMeshAsset::m_assetData)
                    ;

                // Note: This class needs to have edit context reflection so PropertyAssetCtrl::OnEditButtonClicked
                // can open the asset with the preferred asset editor (Scene Settings).
                if (auto* editContext = serializeContext->GetEditContext())
                {
                    editContext->Class<JoltMeshAsset>("Jolt Mesh Asset", "")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ;
                }
            }
        }

        void JoltMeshAsset::SetData(const JoltMeshAssetData& assetData)
        {
            m_assetData = assetData;
            m_status = AZ::Data::AssetData::AssetStatus::Ready;
        }
    } // namespace Pipeline
} // namespace JoltPhysics
