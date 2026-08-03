#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Editor/Components/EditorJoltShapeColliderComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <Clients/Components/JoltShapeColliderComponent.h>
#include <Editor/Components/EditorJoltColliderGeometryUtils.h>
#include <Shape/JoltShapeUtils.h>
#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void EditorJoltShapeColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltColliderComponentBase::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltShapeColliderComponent, EditorJoltColliderComponentBase>()->Version(1);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltShapeColliderComponent>("Jolt Shape Collider",
                    "Collision geometry taken from the shape component on this entity - the way to give a "
                    "Polygon Prism, or any other shape component, collision")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void EditorJoltShapeColliderComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        EditorJoltColliderComponentBase::GetRequiredServices(required);
        required.push_back(AZ_CRC_CE("ShapeService"));
    }

    void EditorJoltShapeColliderComponent::Activate()
    {
        LmbrCentral::ShapeComponentNotificationsBus::Handler::BusConnect(GetEntityId());
        EditorJoltColliderComponentBase::Activate();
    }

    void EditorJoltShapeColliderComponent::Deactivate()
    {
        EditorJoltColliderComponentBase::Deactivate();
        LmbrCentral::ShapeComponentNotificationsBus::Handler::BusDisconnect();
    }

    void EditorJoltShapeColliderComponent::OnShapeChanged([[maybe_unused]] ShapeChangeReasons changeReasons)
    {
        // Resizing the shape is resizing the collider, so the wireframe and the edit-mode
        // body both have to follow it.
        m_debugLinesDirty = true;
        RebuildEditorCollider();
    }

    AzPhysics::ShapeColliderPairList EditorJoltShapeColliderComponent::GetEditorShapeColliderPairs() const
    {
        // The same conversion the runtime component uses, so the viewport and the game
        // cannot disagree about what a given shape becomes.
        AZStd::shared_ptr<Physics::ShapeConfiguration> shapeConfig =
            JoltShapeColliderComponent::BuildShapeConfigurationForEntity(GetEntityId());
        if (!shapeConfig)
        {
            return {};
        }
        return { MakeScaledEditorPair(shapeConfig) };
    }

    void EditorJoltShapeColliderComponent::RebuildDebugLines() const
    {
        m_debugLines.clear();
        m_debugBounds = AZ::Aabb::CreateNull();
        m_debugLinesDirty = false;

        const AzPhysics::ShapeColliderPairList pairs = GetEditorShapeColliderPairs();
        if (pairs.empty() || !pairs.front().second)
        {
            return;
        }

        const JPH::RefConst<JPH::Shape> shape = JoltShapeUtils::CreateJoltShapeFromConfig(*pairs.front().second);
        EditorColliderGeometry::BuildShapeWireframe(shape.GetPtr(), m_debugLines, m_debugBounds);
    }

    void EditorJoltShapeColliderComponent::DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const
    {
        if (m_debugLinesDirty)
        {
            RebuildDebugLines();
        }
        if (m_debugLines.empty())
        {
            return;
        }

        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        worldTransform.ExtractUniformScale(); // the pairs already carry the scale

        m_debugLinesWorld.resize(m_debugLines.size());
        for (size_t i = 0; i < m_debugLines.size(); ++i)
        {
            m_debugLinesWorld[i] = worldTransform.TransformPoint(m_debugLines[i]);
        }
        debugDisplay.DrawLines(m_debugLinesWorld, AZ::Color(0.0f, 1.0f, 0.0f, 1.0f));
    }

    AZ::Aabb EditorJoltShapeColliderComponent::GetLocalShapeBounds() const
    {
        if (m_debugLinesDirty)
        {
            RebuildDebugLines();
        }
        return m_debugBounds;
    }

    void EditorJoltShapeColliderComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        auto* component = gameEntity->CreateComponent<JoltShapeColliderComponent>();
        component->GetColliderConfiguration() = GetColliderConfiguration();
    }
} // namespace JoltPhysics
