#include <Clients/Components/JoltMeshColliderComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void JoltMeshColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<JoltColliderComponentBase>(context);
        Internal::ReflectOnce<Physics::ShapeConfiguration>(context);
        Internal::ReflectOnce<Physics::CookedMeshShapeConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->RegisterGenericType<AZStd::shared_ptr<Physics::CookedMeshShapeConfiguration>>();

            serializeContext->Class<JoltMeshColliderComponent, JoltColliderComponentBase>()
                ->Version(1)
                ->Field("ShapeConfiguration", &JoltMeshColliderComponent::m_shapeConfiguration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltMeshColliderComponent>(
                    "Jolt Mesh Collider", "Mesh collider for the Jolt physics backend")
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

    AzPhysics::ShapeColliderPair JoltMeshColliderComponent::GetShapeColliderPair() const
    {
        return { m_colliderConfiguration, m_shapeConfiguration };
    }
} // namespace JoltPhysics
