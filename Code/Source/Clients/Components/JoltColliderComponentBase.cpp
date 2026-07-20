#include <Clients/Components/JoltColliderComponentBase.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void JoltColliderComponentBase::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<Physics::ColliderConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltColliderComponentBase, AZ::Component>()
                ->Version(1)
                ->Field("ColliderConfiguration", &JoltColliderComponentBase::m_colliderConfiguration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltColliderComponentBase>("Jolt Collider Base", "Base configuration for Jolt colliders")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltColliderComponentBase::m_colliderConfiguration,
                        "Collider Configuration", "Configuration shared by all Jolt colliders")
                    ;
            }
        }
    }

    void JoltColliderComponentBase::Activate()
    {
    }

    void JoltColliderComponentBase::Deactivate()
    {
    }

    void JoltColliderComponentBase::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltColliderService"));
    }

    void JoltColliderComponentBase::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        // Only one Jolt collider per entity for now.
        incompatible.push_back(AZ_CRC_CE("JoltColliderService"));
    }

    void JoltColliderComponentBase::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }
} // namespace JoltPhysics
