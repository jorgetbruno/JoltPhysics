#include <Editor/Components/EditorJoltCylinderColliderComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzToolsFramework/ComponentModes/CylinderComponentMode.h>

#include <Clients/Components/JoltCylinderColliderComponent.h>
#include <Editor/Components/EditorJoltColliderDrawUtils.h>
#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void EditorJoltCylinderColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltColliderComponentBase::Reflect(context);
        Internal::ReflectOnce<JoltCylinderShapeConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->RegisterGenericType<AZStd::shared_ptr<JoltCylinderShapeConfiguration>>();

            serializeContext->Class<EditorJoltCylinderColliderComponent, EditorJoltColliderComponentBase>()
                ->Version(1)
                ->Field("ShapeConfiguration", &EditorJoltCylinderColliderComponent::m_shapeConfiguration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltCylinderColliderComponent>(
                    "Jolt Cylinder Collider", "Cylinder shaped collider for the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltCylinderColliderComponent::m_shapeConfiguration,
                        "Shape Configuration", "Cylinder shape properties")
                    ;
            }
        }
    }

    void EditorJoltCylinderColliderComponent::Activate()
    {
        EditorJoltColliderComponentBase::Activate();

        const AZ::EntityComponentIdPair entityComponentIdPair(GetEntityId(), GetId());
        AzToolsFramework::CylinderManipulatorRequestBus::Handler::BusConnect(entityComponentIdPair);
        AzToolsFramework::RadiusManipulatorRequestBus::Handler::BusConnect(entityComponentIdPair);

        m_componentModeDelegate.ConnectWithSingleComponentMode<
            EditorJoltCylinderColliderComponent, AzToolsFramework::CylinderComponentMode>(entityComponentIdPair, this);
    }

    void EditorJoltCylinderColliderComponent::Deactivate()
    {
        AzToolsFramework::RadiusManipulatorRequestBus::Handler::BusDisconnect();
        AzToolsFramework::CylinderManipulatorRequestBus::Handler::BusDisconnect();
        EditorJoltColliderComponentBase::Deactivate();
    }

    float EditorJoltCylinderColliderComponent::GetHeight() const
    {
        return m_shapeConfiguration->m_height;
    }

    void EditorJoltCylinderColliderComponent::SetHeight(float height)
    {
        m_shapeConfiguration->m_height = AZ::GetMax(height, 0.0001f);
        OnShapeChangedByManipulator();
    }

    float EditorJoltCylinderColliderComponent::GetRadius() const
    {
        return m_shapeConfiguration->m_radius;
    }

    void EditorJoltCylinderColliderComponent::SetRadius(float radius)
    {
        m_shapeConfiguration->m_radius = AZ::GetMax(radius, 0.0001f);
        OnShapeChangedByManipulator();
    }

    AZ::Aabb EditorJoltCylinderColliderComponent::GetLocalShapeBounds() const
    {
        const float radius = m_shapeConfiguration->m_radius;
        return AZ::Aabb::CreateCenterHalfExtents(
            AZ::Vector3::CreateZero(), AZ::Vector3(radius, radius, 0.5f * m_shapeConfiguration->m_height));
    }

    void EditorJoltCylinderColliderComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltCylinderColliderComponent>())
        {
            component->GetColliderConfiguration() = *m_colliderConfiguration;
            component->GetShapeConfiguration() = *m_shapeConfiguration;
        }
    }

    void EditorJoltCylinderColliderComponent::DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const
    {
        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        const AZ::Transform colliderTransform = worldTransform * AZ::Transform::CreateFromQuaternionAndTranslation(
            m_colliderConfiguration->m_rotation, m_colliderConfiguration->m_position);

        // The cylinder is Z-aligned in shape-local space with total height m_height.
        const float radius = m_shapeConfiguration->m_radius;
        const float halfHeight = m_shapeConfiguration->m_height * 0.5f;

        EditorColliderDraw::DrawWireCircle(debugDisplay, colliderTransform, radius, 2, halfHeight);
        EditorColliderDraw::DrawWireCircle(debugDisplay, colliderTransform, radius, 2, -halfHeight);

        // Rails joining the two caps at the four compass points.
        for (const AZ::Vector3& across : { AZ::Vector3(radius, 0.0f, 0.0f), AZ::Vector3(-radius, 0.0f, 0.0f),
                                           AZ::Vector3(0.0f, radius, 0.0f), AZ::Vector3(0.0f, -radius, 0.0f) })
        {
            EditorColliderDraw::DrawWireLine(
                debugDisplay,
                colliderTransform.TransformPoint(across + AZ::Vector3(0.0f, 0.0f, -halfHeight)),
                colliderTransform.TransformPoint(across + AZ::Vector3(0.0f, 0.0f, halfHeight)));
        }
    }

} // namespace JoltPhysics
