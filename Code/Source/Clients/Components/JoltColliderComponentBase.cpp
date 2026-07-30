#include <Utils/JoltComponentUtils.h>
#include <Clients/Components/JoltColliderComponentBase.h>

#include <AzCore/Component/NonUniformScaleBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Color.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/algorithm.h>

#include <AzFramework/Physics/ColliderComponentBus.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    namespace
    {
        // World uniform scale times any NonUniformScale component: the scale the render
        // mesh is drawn at, so the collision has to match (mirrors PhysX's
        // Utils::GetOverallScale).
        AZ::Vector3 GetOverallEntityScale(AZ::EntityId entityId)
        {
            float uniformScale = 1.0f;
            AZ::TransformBus::EventResult(uniformScale, entityId, &AZ::TransformBus::Events::GetWorldUniformScale);
            AZ::Vector3 nonUniformScale = AZ::Vector3::CreateOne();
            AZ::NonUniformScaleRequestBus::EventResult(nonUniformScale, entityId, &AZ::NonUniformScaleRequests::GetScale);
            return nonUniformScale * uniformScale;
        }
    } // namespace

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

    AzPhysics::ShapeColliderPairList JoltColliderComponentBase::GetShapeColliderPairs() const
    {
        AzPhysics::ShapeColliderPairList pairs{ GetShapeColliderPair() };
        ApplyOverallScale(pairs);
        return pairs;
    }

    void JoltColliderComponentBase::ApplyOverallScale(AzPhysics::ShapeColliderPairList& pairs) const
    {
        const AZ::Vector3 overallScale = GetOverallEntityScale(GetEntityId());
        if (overallScale == AZ::Vector3::CreateOne())
        {
            return;
        }

        for (AzPhysics::ShapeColliderPair& pair : pairs)
        {
            if (!pair.second || pair.second->GetShapeType() == Physics::ShapeType::Heightfield)
            {
                continue;
            }

            // Idempotent assignment on the component's own configuration: in this gem
            // the shape config carries no authored scale of its own, so m_scale is
            // exactly the entity scale (mirrors PhysX's UpdateScaleForShapeConfigs).
            pair.second->m_scale = overallScale;

            // The collider offset is authored in unscaled entity space and must scale
            // with the shape; clone rather than mutate the serialized offset.
            if (pair.first && !pair.first->m_position.IsZero())
            {
                pair.first = AZStd::make_shared<Physics::ColliderConfiguration>(*pair.first);
                pair.first->m_position *= overallScale;
            }
        }
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

    void JoltColliderComponentBase::OnAfterEntitySet()
    {
        if (m_serializedIdentifier.empty())
        {
            m_serializedIdentifier = Internal::GenerateSerializedIdentifier(this);
        }
    }

    AZStd::string JoltColliderComponentBase::GetSerializedIdentifier() const
    {
        return m_serializedIdentifier;
    }
} // namespace JoltPhysics
