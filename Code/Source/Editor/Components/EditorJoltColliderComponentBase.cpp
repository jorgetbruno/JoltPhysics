#include <Editor/Components/EditorJoltColliderComponentBase.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void EditorJoltColliderComponentBase::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<Physics::ColliderConfiguration>(context);
        AzToolsFramework::ComponentModeFramework::ComponentModeDelegate::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->RegisterGenericType<AZStd::shared_ptr<Physics::ColliderConfiguration>>();

            serializeContext->Class<EditorJoltColliderComponentBase, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(2)
                ->Field("ColliderConfiguration", &EditorJoltColliderComponentBase::m_colliderConfiguration)
                ->Field("ComponentMode", &EditorJoltColliderComponentBase::m_componentModeDelegate)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltColliderComponentBase>(
                    "Jolt Collider Base", "Base configuration shared by Jolt colliders")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltColliderComponentBase::m_colliderConfiguration,
                        "Collider Configuration", "Configuration shared by all Jolt colliders")
                    // Renders the Edit button that enters component mode.
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltColliderComponentBase::m_componentModeDelegate,
                        "Component Mode", "Collider component mode")
                    ;
            }
        }
    }

    void EditorJoltColliderComponentBase::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltColliderService"));
    }

    void EditorJoltColliderComponentBase::GetIncompatibleServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        // Multiple colliders per entity are allowed (compound bodies), mirroring the
        // runtime collider base and PhysX's BaseColliderComponent.
    }

    void EditorJoltColliderComponentBase::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void EditorJoltColliderComponentBase::Activate()
    {
        AzToolsFramework::Components::EditorComponentBase::Activate();

        const AZ::EntityComponentIdPair entityComponentIdPair(GetEntityId(), GetId());
        AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(GetEntityId());
        AzToolsFramework::ShapeManipulatorRequestBus::Handler::BusConnect(entityComponentIdPair);
        // Addressed by entity, not by entity+component, unlike the manipulator buses.
        AzToolsFramework::EditorComponentSelectionRequestsBus::Handler::BusConnect(GetEntityId());
    }

    void EditorJoltColliderComponentBase::Deactivate()
    {
        m_componentModeDelegate.Disconnect();

        AzToolsFramework::EditorComponentSelectionRequestsBus::Handler::BusDisconnect();
        AzToolsFramework::ShapeManipulatorRequestBus::Handler::BusDisconnect();
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();

        AzToolsFramework::Components::EditorComponentBase::Deactivate();
    }

    void EditorJoltColliderComponentBase::DisplayEntityViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo,
        AzFramework::DebugDisplayRequests& debugDisplay)
    {
        DrawShape(debugDisplay);
    }

    AZ::Vector3 EditorJoltColliderComponentBase::GetTranslationOffset() const
    {
        return m_colliderConfiguration->m_position;
    }

    void EditorJoltColliderComponentBase::SetTranslationOffset(const AZ::Vector3& translationOffset)
    {
        m_colliderConfiguration->m_position = translationOffset;
        OnShapeChangedByManipulator();
    }

    AZ::Transform EditorJoltColliderComponentBase::GetManipulatorSpace() const
    {
        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        return worldTransform;
    }

    AZ::Quaternion EditorJoltColliderComponentBase::GetRotationOffset() const
    {
        return m_colliderConfiguration->m_rotation;
    }

    AZ::Aabb EditorJoltColliderComponentBase::GetEditorSelectionBoundsViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo)
    {
        const AZ::Aabb localBounds = GetLocalShapeBounds();
        if (!localBounds.IsValid())
        {
            return AZ::Aabb::CreateNull();
        }
        return localBounds.GetTransformedAabb(GetColliderWorldTransform());
    }

    bool EditorJoltColliderComponentBase::SupportsEditorRayIntersect()
    {
        // Selection falls back to the bounds above. A per-shape ray test would be tighter
        // for a thin collider, but the shapes here are all convex primitives whose bounds
        // are a close fit.
        return false;
    }

    void EditorJoltColliderComponentBase::OnShapeChangedByManipulator()
    {
        // Values only: refreshing the whole property tree during a drag destroys and
        // rebuilds the manipulators mid-gesture.
        AzToolsFramework::ToolsApplicationEvents::Bus::Broadcast(
            &AzToolsFramework::ToolsApplicationEvents::InvalidatePropertyDisplay,
            AzToolsFramework::Refresh_Values);
    }

    AZ::Transform EditorJoltColliderComponentBase::GetColliderLocalTransform() const
    {
        return AZ::Transform::CreateFromQuaternionAndTranslation(
            m_colliderConfiguration->m_rotation, m_colliderConfiguration->m_position);
    }

    AZ::Transform EditorJoltColliderComponentBase::GetColliderWorldTransform() const
    {
        return GetManipulatorSpace() * GetColliderLocalTransform();
    }

} // namespace JoltPhysics
