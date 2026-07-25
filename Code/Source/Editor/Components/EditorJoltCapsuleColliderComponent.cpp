#include <Editor/Components/EditorJoltCapsuleColliderComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzToolsFramework/ComponentModes/CapsuleComponentMode.h>

#include <Clients/Components/JoltCapsuleColliderComponent.h>
#include <Editor/Components/EditorJoltColliderDrawUtils.h>
#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void EditorJoltCapsuleColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltColliderComponentBase::Reflect(context);
        Internal::ReflectOnce<Physics::CapsuleShapeConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->RegisterGenericType<AZStd::shared_ptr<Physics::CapsuleShapeConfiguration>>();

            serializeContext->Class<EditorJoltCapsuleColliderComponent, EditorJoltColliderComponentBase>()
                ->Version(1)
                ->Field("ShapeConfiguration", &EditorJoltCapsuleColliderComponent::m_shapeConfiguration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltCapsuleColliderComponent>(
                    "Jolt Capsule Collider", "Capsule shaped collider for the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltCapsuleColliderComponent::m_shapeConfiguration,
                        "Shape Configuration", "Capsule shape properties")
                    ;
            }
        }
    }

    void EditorJoltCapsuleColliderComponent::Activate()
    {
        EditorJoltColliderComponentBase::Activate();

        const AZ::EntityComponentIdPair entityComponentIdPair(GetEntityId(), GetId());
        AzToolsFramework::CapsuleManipulatorRequestBus::Handler::BusConnect(entityComponentIdPair);
        AzToolsFramework::RadiusManipulatorRequestBus::Handler::BusConnect(entityComponentIdPair);

        m_componentModeDelegate.ConnectWithSingleComponentMode<
            EditorJoltCapsuleColliderComponent, AzToolsFramework::CapsuleComponentMode>(entityComponentIdPair, this);
    }

    void EditorJoltCapsuleColliderComponent::Deactivate()
    {
        AzToolsFramework::RadiusManipulatorRequestBus::Handler::BusDisconnect();
        AzToolsFramework::CapsuleManipulatorRequestBus::Handler::BusDisconnect();
        EditorJoltColliderComponentBase::Deactivate();
    }

    float EditorJoltCapsuleColliderComponent::GetHeight() const
    {
        return m_shapeConfiguration->m_height;
    }

    void EditorJoltCapsuleColliderComponent::SetHeight(float height)
    {
        // O3DE capsule height is the total including both caps, so it can never be shorter
        // than the sphere those caps would form. Clamping here rather than letting the
        // manipulator produce a degenerate capsule the backend would have to reject.
        m_shapeConfiguration->m_height = AZ::GetMax(height, 2.0f * m_shapeConfiguration->m_radius);
        OnShapeChangedByManipulator();
    }

    float EditorJoltCapsuleColliderComponent::GetRadius() const
    {
        return m_shapeConfiguration->m_radius;
    }

    void EditorJoltCapsuleColliderComponent::SetRadius(float radius)
    {
        m_shapeConfiguration->m_radius = AZ::GetClamp(radius, 0.0001f, 0.5f * m_shapeConfiguration->m_height);
        OnShapeChangedByManipulator();
    }

    AZ::Aabb EditorJoltCapsuleColliderComponent::GetLocalShapeBounds() const
    {
        const float radius = m_shapeConfiguration->m_radius;
        return AZ::Aabb::CreateCenterHalfExtents(
            AZ::Vector3::CreateZero(), AZ::Vector3(radius, radius, 0.5f * m_shapeConfiguration->m_height));
    }

    void EditorJoltCapsuleColliderComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltCapsuleColliderComponent>())
        {
            component->GetColliderConfiguration() = *m_colliderConfiguration;
            component->GetShapeConfiguration() = *m_shapeConfiguration;
        }
    }

    void EditorJoltCapsuleColliderComponent::DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const
    {
        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        const AZ::Transform colliderTransform = worldTransform * AZ::Transform::CreateFromQuaternionAndTranslation(
            m_colliderConfiguration->m_rotation, m_colliderConfiguration->m_position);

        // O3DE capsules are Z-aligned in shape-local space with total height m_height.
        const float radius = m_shapeConfiguration->m_radius;
        const float halfCylinder = AZ::GetMax(0.0f, m_shapeConfiguration->m_height * 0.5f - radius);

        EditorColliderDraw::DrawWireCircle(debugDisplay, colliderTransform, radius, 2, halfCylinder);
        EditorColliderDraw::DrawWireCircle(debugDisplay, colliderTransform, radius, 2, -halfCylinder);
        for (bool useX : { true, false })
        {
            for (bool top : { true, false })
            {
                EditorColliderDraw::DrawWireArc(debugDisplay, colliderTransform, radius, halfCylinder, useX, top);
            }
        }
        // Vertical rails at the four compass points.
        for (const AZ::Vector3& across : { AZ::Vector3(radius, 0.0f, 0.0f), AZ::Vector3(-radius, 0.0f, 0.0f),
                                           AZ::Vector3(0.0f, radius, 0.0f), AZ::Vector3(0.0f, -radius, 0.0f) })
        {
            EditorColliderDraw::DrawWireLine(
                debugDisplay,
                colliderTransform.TransformPoint(across + AZ::Vector3(0.0f, 0.0f, -halfCylinder)),
                colliderTransform.TransformPoint(across + AZ::Vector3(0.0f, 0.0f, halfCylinder)));
        }
    }

} // namespace JoltPhysics
