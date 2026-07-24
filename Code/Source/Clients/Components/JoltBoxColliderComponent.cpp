#include <Clients/Components/JoltBoxColliderComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void JoltBoxColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<JoltColliderComponentBase>(context);
        Internal::ReflectOnce<Physics::ShapeConfiguration>(context);
        Internal::ReflectOnce<Physics::BoxShapeConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltBoxColliderComponent, JoltColliderComponentBase>()
                ->Version(1)
                ->Field("ShapeConfiguration", &JoltBoxColliderComponent::m_shapeConfiguration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltBoxColliderComponent>("Jolt Box Collider", "Box shaped collider for the Jolt physics backend")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        // No AppearsInAddComponentMenu: the editor-side EditorJoltBoxColliderComponent
                        // owns the menu entry (PhysX-style editor/runtime split). This runtime
                        // component stays registered so existing prefabs load and BuildGameEntity
                        // can spawn it, and the EditContext below keeps its properties editable
                        // through the GenericComponentWrapper in those old prefabs.
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::RemoveableByUser, true)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltBoxColliderComponent::m_shapeConfiguration,
                        "Shape Configuration", "Box shape properties")
                    ;
            }
        }
    }

    AzPhysics::ShapeColliderPair JoltBoxColliderComponent::GetShapeColliderPair() const
    {
        return { m_colliderConfiguration, m_shapeConfiguration };
    }
} // namespace JoltPhysics
