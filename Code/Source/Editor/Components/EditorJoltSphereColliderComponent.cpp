#include <Editor/Components/EditorJoltSphereColliderComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzToolsFramework/ComponentModes/SphereComponentMode.h>

#include <Clients/Components/JoltSphereColliderComponent.h>
#include <Editor/Components/EditorJoltColliderDrawUtils.h>
#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void EditorJoltSphereColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltColliderComponentBase::Reflect(context);
        Internal::ReflectOnce<Physics::SphereShapeConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->RegisterGenericType<AZStd::shared_ptr<Physics::SphereShapeConfiguration>>();

            serializeContext->Class<EditorJoltSphereColliderComponent, EditorJoltColliderComponentBase>()
                ->Version(1)
                ->Field("ShapeConfiguration", &EditorJoltSphereColliderComponent::m_shapeConfiguration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltSphereColliderComponent>(
                    "Jolt Sphere Collider", "Sphere shaped collider for the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltSphereColliderComponent::m_shapeConfiguration,
                        "Shape Configuration", "Sphere shape properties")
                    ;
            }
        }
    }

    void EditorJoltSphereColliderComponent::Activate()
    {
        EditorJoltColliderComponentBase::Activate();

        const AZ::EntityComponentIdPair entityComponentIdPair(GetEntityId(), GetId());
        AzToolsFramework::RadiusManipulatorRequestBus::Handler::BusConnect(entityComponentIdPair);

        m_componentModeDelegate.ConnectWithSingleComponentMode<
            EditorJoltSphereColliderComponent, AzToolsFramework::SphereComponentMode>(entityComponentIdPair, this);
    }

    void EditorJoltSphereColliderComponent::Deactivate()
    {
        AzToolsFramework::RadiusManipulatorRequestBus::Handler::BusDisconnect();
        EditorJoltColliderComponentBase::Deactivate();
    }

    float EditorJoltSphereColliderComponent::GetRadius() const
    {
        return m_shapeConfiguration->m_radius;
    }

    void EditorJoltSphereColliderComponent::SetRadius(float radius)
    {
        m_shapeConfiguration->m_radius = AZ::GetMax(radius, 0.0001f);
        OnShapeChangedByManipulator();
    }

    AZ::Aabb EditorJoltSphereColliderComponent::GetLocalShapeBounds() const
    {
        return AZ::Aabb::CreateCenterRadius(AZ::Vector3::CreateZero(), m_shapeConfiguration->m_radius);
    }

    void EditorJoltSphereColliderComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltSphereColliderComponent>())
        {
            component->GetColliderConfiguration() = *m_colliderConfiguration;
            component->GetShapeConfiguration() = *m_shapeConfiguration;
        }
    }

    void EditorJoltSphereColliderComponent::DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const
    {
        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        const AZ::Transform colliderTransform = worldTransform * AZ::Transform::CreateFromQuaternionAndTranslation(
            m_colliderConfiguration->m_rotation, m_colliderConfiguration->m_position);

        for (AZ::u32 axis = 0; axis < 3; ++axis)
        {
            EditorColliderDraw::DrawWireCircle(debugDisplay, colliderTransform, m_shapeConfiguration->m_radius, axis);
        }
    }

} // namespace JoltPhysics
