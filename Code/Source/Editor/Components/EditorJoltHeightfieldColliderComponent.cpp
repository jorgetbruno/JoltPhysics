#include <Editor/Components/EditorJoltHeightfieldColliderComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltHeightfieldColliderComponent.h>

namespace JoltPhysics
{
    void EditorJoltHeightfieldColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltColliderComponentBase::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltHeightfieldColliderComponent, EditorJoltColliderComponentBase>()
                ->Version(1)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltHeightfieldColliderComponent>(
                    "Jolt Heightfield Collider",
                    "Heightfield collider fed by a Physics::HeightfieldProviderBus implementation (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void EditorJoltHeightfieldColliderComponent::Activate()
    {
        EditorJoltColliderComponentBase::Activate();
        Physics::HeightfieldProviderNotificationBus::Handler::BusConnect(GetEntityId());
        m_dirty = true;
    }

    void EditorJoltHeightfieldColliderComponent::Deactivate()
    {
        Physics::HeightfieldProviderNotificationBus::Handler::BusDisconnect();
        EditorJoltColliderComponentBase::Deactivate();
    }

    void EditorJoltHeightfieldColliderComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltHeightfieldColliderComponent>())
        {
            component->GetColliderConfiguration() = m_colliderConfiguration;
        }
    }

    void EditorJoltHeightfieldColliderComponent::OnHeightfieldDataChanged(
        [[maybe_unused]] const AZ::Aabb& dirtyRegion,
        [[maybe_unused]] Physics::HeightfieldProviderNotifications::HeightfieldChangeMask changeMask)
    {
        // Any change mask can move the drawn surface: heights obviously, but a settings
        // or region change alters the grid extent too.
        m_dirty = true;
    }

    void EditorJoltHeightfieldColliderComponent::RefreshFromProvider() const
    {
        m_dirty = false;
        m_grid = EditorColliderGeometry::HeightfieldGrid();
        m_wireframeLines.clear();
        m_localBounds = AZ::Aabb::CreateNull();

        size_t numColumns = 0;
        size_t numRows = 0;
        Physics::HeightfieldProviderRequestsBus::EventResult(
            numColumns, GetEntityId(), &Physics::HeightfieldProviderRequests::GetHeightfieldGridColumns);
        Physics::HeightfieldProviderRequestsBus::EventResult(
            numRows, GetEntityId(), &Physics::HeightfieldProviderRequests::GetHeightfieldGridRows);

        AZ::Vector2 gridSpacing = AZ::Vector2::CreateOne();
        Physics::HeightfieldProviderRequestsBus::EventResult(
            gridSpacing, GetEntityId(), &Physics::HeightfieldProviderRequests::GetHeightfieldGridSpacing);

        AZStd::vector<float> heights;
        Physics::HeightfieldProviderRequestsBus::EventResult(
            heights, GetEntityId(), &Physics::HeightfieldProviderRequests::GetHeights);

        m_grid.m_columns = static_cast<AZ::u32>(numColumns);
        m_grid.m_rows = static_cast<AZ::u32>(numRows);
        m_grid.m_spacing = gridSpacing;
        m_grid.m_heights = AZStd::move(heights);

        if (!m_grid.IsValid())
        {
            // No provider on this entity yet, or it has nothing to offer: nothing to draw
            // and nothing to pick against. Quiet - the runtime component makes the same
            // allowance while a level loads.
            return;
        }

        m_wireframeLines = EditorColliderGeometry::BuildHeightfieldWireframe(m_grid, MaxWireframeLinesPerAxis);
        m_localBounds = EditorColliderGeometry::ComputeHeightfieldBounds(m_grid);
    }

    void EditorJoltHeightfieldColliderComponent::DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const
    {
        if (m_dirty)
        {
            RefreshFromProvider();
        }
        if (m_wireframeLines.empty())
        {
            return;
        }

        const AZ::Transform colliderTransform = GetColliderWorldTransform();
        m_wireframeLinesWorld.resize(m_wireframeLines.size());
        for (size_t i = 0; i < m_wireframeLines.size(); ++i)
        {
            m_wireframeLinesWorld[i] = colliderTransform.TransformPoint(m_wireframeLines[i]);
        }

        debugDisplay.DrawLines(m_wireframeLinesWorld, AZ::Color(0.0f, 1.0f, 0.0f, 1.0f));
    }

    AZ::Aabb EditorJoltHeightfieldColliderComponent::GetLocalShapeBounds() const
    {
        if (m_dirty)
        {
            RefreshFromProvider();
        }
        return m_localBounds;
    }

} // namespace JoltPhysics
