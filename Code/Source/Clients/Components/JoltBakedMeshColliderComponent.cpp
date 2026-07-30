#include <Clients/Components/JoltBakedMeshColliderComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void JoltBakedMeshColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<JoltColliderComponentBase>(context);
        Internal::ReflectOnce<Physics::ShapeConfiguration>(context);
        Internal::ReflectOnce<Physics::CookedMeshShapeConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->RegisterGenericType<AZStd::shared_ptr<Physics::CookedMeshShapeConfiguration>>();

            serializeContext->Class<JoltBakedMeshColliderComponent, JoltColliderComponentBase>()
                ->Version(1)
                ->Field("ShapeConfiguration", &JoltBakedMeshColliderComponent::m_shapeConfiguration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltBakedMeshColliderComponent>(
                    "Jolt Baked Mesh Collider", "Mesh collider baked into the prefab for the Jolt physics backend")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        // No AppearsInAddComponentMenu: the editor-side EditorJoltBakedMeshColliderComponent
                        // owns the menu entry (PhysX-style editor/runtime split).
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::RemoveableByUser, true)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    AzPhysics::ShapeColliderPair JoltBakedMeshColliderComponent::GetShapeColliderPair() const
    {
        return { m_colliderConfiguration, m_shapeConfiguration };
    }
} // namespace JoltPhysics
