#include <Editor/Components/EditorJoltBoxColliderComponent.h>

#include <AzCore/std/smart_ptr/make_shared.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzToolsFramework/ComponentModes/BoxComponentMode.h>

#include <Clients/Components/JoltBoxColliderComponent.h>
#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    namespace
    {
        // The 12 edges of a unit box centered at the origin (pairs of corner indices).
        constexpr AZ::u32 BoxEdges[][2] = {
            { 0, 1 }, { 1, 3 }, { 3, 2 }, { 2, 0 },
            { 4, 5 }, { 5, 7 }, { 7, 6 }, { 6, 4 },
            { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 },
        };

        AZ::Vector3 BoxCorner(const AZ::Vector3& halfExtents, AZ::u32 corner)
        {
            return AZ::Vector3(
                (corner & 1) ? halfExtents.GetX() : -halfExtents.GetX(),
                (corner & 2) ? halfExtents.GetY() : -halfExtents.GetY(),
                (corner & 4) ? halfExtents.GetZ() : -halfExtents.GetZ());
        }
    }

    void EditorJoltBoxColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltColliderComponentBase::Reflect(context);
        Internal::ReflectOnce<Physics::BoxShapeConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {

            serializeContext->Class<EditorJoltBoxColliderComponent, EditorJoltColliderComponentBase>()
                ->Version(1)
                ->Field("ShapeConfiguration", &EditorJoltBoxColliderComponent::m_shapeConfiguration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltBoxColliderComponent>(
                    "Jolt Box Collider", "Box shaped collider for the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltBoxColliderComponent::m_shapeConfiguration,
                        "Shape Configuration", "Box shape properties")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify,
                            &EditorJoltBoxColliderComponent::OnColliderConfigurationChangedInEditor)
                    ;
            }
        }
    }

    void EditorJoltBoxColliderComponent::Activate()
    {
        EditorJoltColliderComponentBase::Activate();

        const AZ::EntityComponentIdPair entityComponentIdPair(GetEntityId(), GetId());
        AzToolsFramework::BoxManipulatorRequestBus::Handler::BusConnect(entityComponentIdPair);

        // AzToolsFramework ships the box mode and its manipulators; all this component
        // supplies is the dimensions bus above, so the collider behaves like the engine's
        // own box shape in the viewport.
        m_componentModeDelegate.ConnectWithSingleComponentMode<
            EditorJoltBoxColliderComponent, AzToolsFramework::BoxComponentMode>(entityComponentIdPair, this);
    }

    void EditorJoltBoxColliderComponent::Deactivate()
    {
        AzToolsFramework::BoxManipulatorRequestBus::Handler::BusDisconnect();
        EditorJoltColliderComponentBase::Deactivate();
    }

    AZ::Vector3 EditorJoltBoxColliderComponent::GetDimensions() const
    {
        return m_shapeConfiguration.m_dimensions;
    }

    void EditorJoltBoxColliderComponent::SetDimensions(const AZ::Vector3& dimensions)
    {
        m_shapeConfiguration.m_dimensions = dimensions;
        OnShapeChangedByManipulator();
    }

    AZ::Transform EditorJoltBoxColliderComponent::GetCurrentLocalTransform() const
    {
        return GetColliderLocalTransform();
    }

    AzPhysics::ShapeColliderPairList EditorJoltBoxColliderComponent::GetEditorShapeColliderPairs() const
    {
        return { MakeScaledEditorPair(AZStd::make_shared<Physics::BoxShapeConfiguration>(m_shapeConfiguration)) };
    }

    AZ::Aabb EditorJoltBoxColliderComponent::GetLocalShapeBounds() const
    {
        return AZ::Aabb::CreateCenterHalfExtents(AZ::Vector3::CreateZero(), m_shapeConfiguration.m_dimensions * 0.5f);
    }

    void EditorJoltBoxColliderComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltBoxColliderComponent>())
        {
            component->GetColliderConfiguration() = m_colliderConfiguration;
            component->GetShapeConfiguration() = m_shapeConfiguration;
        }
    }

    void EditorJoltBoxColliderComponent::DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const
    {
        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        const AZ::Transform colliderTransform = worldTransform * AZ::Transform::CreateFromQuaternionAndTranslation(
            m_colliderConfiguration.m_rotation, m_colliderConfiguration.m_position);
        const AZ::Vector3 halfExtents = m_shapeConfiguration.m_dimensions * 0.5f;

        for (const auto& edge : BoxEdges)
        {
            const AZ::Vector3 from = colliderTransform.TransformPoint(BoxCorner(halfExtents, edge[0]));
            const AZ::Vector3 to = colliderTransform.TransformPoint(BoxCorner(halfExtents, edge[1]));
            debugDisplay.DrawLine(from, to, AZ::Vector4(0.0f, 1.0f, 0.0f, 1.0f), AZ::Vector4(0.0f, 1.0f, 0.0f, 1.0f));
        }
    }

} // namespace JoltPhysics
