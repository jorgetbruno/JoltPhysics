#include <Editor/Components/EditorJoltColliderComponentBase.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/SystemBus.h>

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    namespace
    {
        // World uniform scale times any NonUniformScale component: the scale the render
        // mesh is drawn at (mirrors the runtime JoltColliderComponentBase).
        AZ::Vector3 GetOverallEntityScale(AZ::EntityId entityId)
        {
            float uniformScale = 1.0f;
            AZ::TransformBus::EventResult(uniformScale, entityId, &AZ::TransformBus::Events::GetWorldUniformScale);
            AZ::Vector3 nonUniformScale = AZ::Vector3::CreateOne();
            AZ::NonUniformScaleRequestBus::EventResult(nonUniformScale, entityId, &AZ::NonUniformScaleRequests::GetScale);
            return nonUniformScale * uniformScale;
        }
    } // namespace

    void EditorJoltColliderComponentBase::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<Physics::ColliderConfiguration>(context);
        Internal::ReflectOnce<AzToolsFramework::ComponentModeFramework::ComponentModeDelegate>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            // Every derived collider calls this base Reflect, so guard against the
            // second and later passes - re-registering trips a duplicated-Uuid error.
            if (serializeContext->FindClassData(azrtti_typeid<EditorJoltColliderComponentBase>()) != nullptr)
            {
                return;
            }

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
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify,
                            &EditorJoltColliderComponentBase::OnColliderConfigurationChangedInEditor)
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
        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
        AzToolsFramework::ShapeManipulatorRequestBus::Handler::BusConnect(entityComponentIdPair);
        // Addressed by entity, not by entity+component, unlike the manipulator buses.
        AzToolsFramework::EditorComponentSelectionRequestsBus::Handler::BusConnect(GetEntityId());

        m_nonUniformScaleChangedHandler = AZ::NonUniformScaleChangedEvent::Handler(
            [this]([[maybe_unused]] const AZ::Vector3& scale)
            {
                RebuildEditorCollider();
            });
        AZ::NonUniformScaleRequestBus::Event(
            GetEntityId(), &AZ::NonUniformScaleRequests::RegisterScaleChangedEvent, m_nonUniformScaleChangedHandler);

        RebuildEditorCollider();
    }

    void EditorJoltColliderComponentBase::Deactivate()
    {
        DestroyEditorCollider();
        m_nonUniformScaleChangedHandler.Disconnect();

        m_componentModeDelegate.Disconnect();

        AzToolsFramework::EditorComponentSelectionRequestsBus::Handler::BusDisconnect();
        AzToolsFramework::ShapeManipulatorRequestBus::Handler::BusDisconnect();
        AZ::TransformNotificationBus::Handler::BusDisconnect();
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
        return m_colliderConfiguration.m_position;
    }

    void EditorJoltColliderComponentBase::SetTranslationOffset(const AZ::Vector3& translationOffset)
    {
        m_colliderConfiguration.m_position = translationOffset;
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
        return m_colliderConfiguration.m_rotation;
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

        RebuildEditorCollider();
    }

    void EditorJoltColliderComponentBase::OnTransformChanged(
        [[maybe_unused]] const AZ::Transform& local, const AZ::Transform& world)
    {
        // A move or rotation just re-poses the static body; only a scale change (which
        // lives in the shape, not the body transform) needs the shape rebuilt.
        if (!GetOverallEntityScale(GetEntityId()).IsClose(m_editorBodyBuiltScale))
        {
            RebuildEditorCollider();
            return;
        }

        if (m_editorBodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
        {
            return;
        }
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AzPhysics::Scene* editorScene = physicsSystem ? physicsSystem->GetScene(m_editorBodySceneHandle) : nullptr;
        if (AzPhysics::SimulatedBody* body =
                editorScene ? editorScene->GetSimulatedBodyFromHandle(m_editorBodyHandle) : nullptr)
        {
            body->SetTransform(world);
        }
    }

    AzPhysics::ShapeColliderPair EditorJoltColliderComponentBase::MakeScaledEditorPair(
        AZStd::shared_ptr<Physics::ShapeConfiguration> shapeConfiguration) const
    {
        const AZ::Vector3 overallScale = GetOverallEntityScale(GetEntityId());
        auto colliderConfiguration = AZStd::make_shared<Physics::ColliderConfiguration>(m_colliderConfiguration);
        shapeConfiguration->m_scale = overallScale;
        // The collider offset is authored in unscaled entity space and must scale with
        // the shape (mirrors the runtime ApplyOverallScale).
        colliderConfiguration->m_position *= overallScale;
        return { AZStd::move(colliderConfiguration), AZStd::move(shapeConfiguration) };
    }

    void EditorJoltColliderComponentBase::RebuildEditorCollider()
    {
        DestroyEditorCollider();

        // Without an editor world (a game launcher, or a test without the editor system
        // component) there is simply no editor body; everything else still works.
        AzPhysics::SceneHandle editorSceneHandle = AzPhysics::InvalidSceneHandle;
        Physics::EditorWorldBus::BroadcastResult(
            editorSceneHandle, &Physics::EditorWorldRequests::GetEditorSceneHandle);
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AzPhysics::Scene* editorScene = physicsSystem ? physicsSystem->GetScene(editorSceneHandle) : nullptr;
        if (!editorScene)
        {
            return;
        }

        AzPhysics::ShapeColliderPairList pairs = GetEditorShapeColliderPairs();
        AZStd::erase_if(pairs, [](const AzPhysics::ShapeColliderPair& pair) { return pair.second == nullptr; });
        if (pairs.empty())
        {
            return;
        }

        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        AzPhysics::StaticRigidBodyConfiguration bodyConfiguration;
        bodyConfiguration.m_position = worldTransform.GetTranslation();
        bodyConfiguration.m_orientation = worldTransform.GetRotation();
        bodyConfiguration.m_entityId = GetEntityId();
        bodyConfiguration.m_debugName = GetEntity() ? GetEntity()->GetName() : AZStd::string();
        bodyConfiguration.m_colliderAndShapeData = AZStd::move(pairs);

        m_editorBodyHandle = editorScene->AddSimulatedBody(&bodyConfiguration);
        m_editorBodySceneHandle = editorSceneHandle;
        m_editorBodyBuiltScale = GetOverallEntityScale(GetEntityId());
    }

    void EditorJoltColliderComponentBase::DestroyEditorCollider()
    {
        if (m_editorBodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
        {
            return;
        }

        // The scene may already be gone (editor shutdown removes it, and with it every
        // body it held), so a missing scene just means there is nothing left to remove.
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        if (AzPhysics::Scene* editorScene = physicsSystem ? physicsSystem->GetScene(m_editorBodySceneHandle) : nullptr)
        {
            editorScene->RemoveSimulatedBody(m_editorBodyHandle);
        }
        m_editorBodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        m_editorBodySceneHandle = AzPhysics::InvalidSceneHandle;
    }

    AZ::u32 EditorJoltColliderComponentBase::OnColliderConfigurationChangedInEditor()
    {
        RebuildEditorCollider();
        return AZ::Edit::PropertyRefreshLevels::None;
    }

    AZ::Transform EditorJoltColliderComponentBase::GetColliderLocalTransform() const
    {
        return AZ::Transform::CreateFromQuaternionAndTranslation(
            m_colliderConfiguration.m_rotation, m_colliderConfiguration.m_position);
    }

    AZ::Transform EditorJoltColliderComponentBase::GetColliderWorldTransform() const
    {
        return GetManipulatorSpace() * GetColliderLocalTransform();
    }

} // namespace JoltPhysics
