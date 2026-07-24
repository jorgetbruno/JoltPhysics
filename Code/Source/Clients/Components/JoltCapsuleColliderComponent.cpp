#include <Clients/Components/JoltCapsuleColliderComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void JoltCapsuleColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<JoltColliderComponentBase>(context);
        Internal::ReflectOnce<Physics::ShapeConfiguration>(context);
        Internal::ReflectOnce<Physics::CapsuleShapeConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltCapsuleColliderComponent, JoltColliderComponentBase>()
                ->Version(1)
                ->Field("ShapeConfiguration", &JoltCapsuleColliderComponent::m_shapeConfiguration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltCapsuleColliderComponent>("Jolt Capsule Collider", "Capsule shaped collider for the Jolt physics backend")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        // No AppearsInAddComponentMenu: EditorJoltCapsuleColliderComponent owns the
                        // menu entry (PhysX-style editor/runtime split). The runtime component stays
                        // registered for old prefabs and BuildGameEntity.
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::RemoveableByUser, true)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltCapsuleColliderComponent::m_shapeConfiguration,
                        "Shape Configuration", "Capsule shape properties")
                    ;
            }
        }
    }

    AzPhysics::ShapeColliderPair JoltCapsuleColliderComponent::GetShapeColliderPair() const
    {
        return { m_colliderConfiguration, m_shapeConfiguration };
    }
} // namespace JoltPhysics
