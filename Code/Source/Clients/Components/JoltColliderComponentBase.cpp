#include <Clients/Components/JoltColliderComponentBase.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Physics/ColliderComponentBus.h>

#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void JoltColliderComponentBase::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<Physics::ColliderConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->RegisterGenericType<AZStd::shared_ptr<Physics::ColliderConfiguration>>();

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
        Physics::ColliderComponentEventBus::Event(
            GetEntityId(), &Physics::ColliderComponentEvents::OnColliderChanged);
    }

    void JoltColliderComponentBase::Deactivate()
    {
        Physics::ColliderComponentEventBus::Event(
            GetEntityId(), &Physics::ColliderComponentEvents::OnColliderChanged);
    }

    void JoltColliderComponentBase::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltColliderService"));
    }

    void JoltColliderComponentBase::GetIncompatibleServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        // Multiple colliders per entity are allowed; they are combined into a
        // single compound simulated body (mirrors PhysX BaseColliderComponent).
    }

    void JoltColliderComponentBase::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }
} // namespace JoltPhysics
