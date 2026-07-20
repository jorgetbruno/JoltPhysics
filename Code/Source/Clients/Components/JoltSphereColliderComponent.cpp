#include <Clients/Components/JoltSphereColliderComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void JoltSphereColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<JoltColliderComponentBase>(context);
        Internal::ReflectOnce<Physics::ShapeConfiguration>(context);
        Internal::ReflectOnce<Physics::SphereShapeConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltSphereColliderComponent, JoltColliderComponentBase>()
                ->Version(1)
                ->Field("ShapeConfiguration", &JoltSphereColliderComponent::m_shapeConfiguration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltSphereColliderComponent>("Jolt Sphere Collider", "Sphere shaped collider for the Jolt physics backend")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSphereColliderComponent::m_shapeConfiguration,
                        "Shape Configuration", "Sphere shape properties")
                    ;
            }
        }
    }

    AzPhysics::ShapeColliderPair JoltSphereColliderComponent::GetShapeColliderPair() const
    {
        return { m_colliderConfiguration, m_shapeConfiguration };
    }
} // namespace JoltPhysics
