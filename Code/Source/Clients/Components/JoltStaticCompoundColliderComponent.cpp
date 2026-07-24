#include <Clients/Components/JoltStaticCompoundColliderComponent.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Physics/ColliderComponentBus.h>

namespace JoltPhysics
{
    void JoltStaticCompoundColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltStaticCompoundColliderComponent, JoltColliderComponentBase>()
                ->Version(1)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltStaticCompoundColliderComponent>(
                    "Jolt Static Compound Collider",
                    "Combines the colliders of all child entities into a single compound collider")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        // No AppearsInAddComponentMenu: EditorJoltStaticCompoundColliderComponent owns
                        // the menu entry (PhysX-style editor/runtime split). The runtime component
                        // stays registered for old prefabs and BuildGameEntity.
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void JoltStaticCompoundColliderComponent::Activate()
    {
        GatherChildColliders();
        JoltColliderComponentBase::Activate();
    }

    AzPhysics::ShapeColliderPair JoltStaticCompoundColliderComponent::GetShapeColliderPair() const
    {
        // A compound collider has no shape of its own; the pair list is what matters.
        return {};
    }

    AzPhysics::ShapeColliderPairList JoltStaticCompoundColliderComponent::GetShapeColliderPairs() const
    {
        return m_childPairs;
    }

    void JoltStaticCompoundColliderComponent::GatherChildColliders()
    {
        m_childPairs.clear();

        AZStd::vector<AZ::EntityId> children;
        AZ::TransformBus::EventResult(children, GetEntityId(), &AZ::TransformBus::Events::GetChildren);

        for (const AZ::EntityId& childId : children)
        {
            AZ::Entity* childEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(childEntity, &AZ::ComponentApplicationRequests::FindEntity, childId);
            if (!childEntity)
            {
                continue;
            }

            for (AZ::Component* component : childEntity->GetComponents())
            {
                if (auto* collider = azrtti_cast<JoltColliderComponentBase*>(component))
                {
                    for (const AzPhysics::ShapeColliderPair& pair : collider->GetShapeColliderPairs())
                    {
                        if (pair.second)
                        {
                            // Child collider offsets compose with the child entity's
                            // transform relative to this entity.
                            auto compoundPair = pair;
                            if (pair.first)
                            {
                                auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>(*pair.first);

                                AZ::Transform childLocal = AZ::Transform::CreateIdentity();
                                AZ::TransformBus::EventResult(childLocal, childId, &AZ::TransformBus::Events::GetLocalTM);

                                colliderConfig->m_position = childLocal.GetTranslation() +
                                    childLocal.GetRotation().TransformVector(pair.first->m_position);
                                colliderConfig->m_rotation = childLocal.GetRotation() * pair.first->m_rotation;

                                compoundPair.first = colliderConfig;
                            }
                            m_childPairs.push_back(compoundPair);
                        }
                    }
                }
            }
        }
    }

    void JoltMutableCompoundColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltMutableCompoundColliderComponent, JoltStaticCompoundColliderComponent>()
                ->Version(1)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltMutableCompoundColliderComponent>(
                    "Jolt Mutable Compound Collider",
                    "Compound collider that supports adding and removing child collider entities at runtime")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        // No AppearsInAddComponentMenu: EditorJoltMutableCompoundColliderComponent owns
                        // the menu entry (PhysX-style editor/runtime split). The runtime component
                        // stays registered for old prefabs and BuildGameEntity.
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void JoltMutableCompoundColliderComponent::Activate()
    {
        JoltStaticCompoundColliderComponent::Activate();
        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
    }

    void JoltMutableCompoundColliderComponent::Deactivate()
    {
        AZ::TransformNotificationBus::Handler::BusDisconnect();
        JoltStaticCompoundColliderComponent::Deactivate();
    }

    void JoltMutableCompoundColliderComponent::OnChildAdded([[maybe_unused]] AZ::EntityId child)
    {
        GatherChildColliders();
        Physics::ColliderComponentEventBus::Event(
            GetEntityId(), &Physics::ColliderComponentEvents::OnColliderChanged);
    }

    void JoltMutableCompoundColliderComponent::OnChildRemoved([[maybe_unused]] AZ::EntityId child)
    {
        GatherChildColliders();
        Physics::ColliderComponentEventBus::Event(
            GetEntityId(), &Physics::ColliderComponentEvents::OnColliderChanged);
    }

} // namespace JoltPhysics
