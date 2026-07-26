#include <Editor/Components/EditorJoltJointComponentBase.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Editor/Components/EditorJoltDebugDrawUtils.h>
#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void EditorJoltJointComponentBase::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<AzToolsFramework::ComponentModeFramework::ComponentModeDelegate>(context);
        Internal::ReflectOnce<JoltJointComponentMode>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            // Every joint type calls this base Reflect, so guard against the second
            // and later passes - re-registering trips a duplicated-Uuid error.
            if (serializeContext->FindClassData(azrtti_typeid<EditorJoltJointComponentBase>()) != nullptr)
            {
                return;
            }

            serializeContext->Class<EditorJoltJointComponentBase, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(2)
                ->Field("Configuration", &EditorJoltJointComponentBase::m_configuration)
                ->Field("ComponentMode", &EditorJoltJointComponentBase::m_componentModeDelegate)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                // The JoltJointComponentConfiguration field-level edit context is registered
                // by the runtime JoltJointComponentBase::Reflect, which also runs in this dll.
                editContext->Class<EditorJoltJointComponentBase>(
                    "Jolt Joint Base", "Base configuration shared by Jolt joints")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltJointComponentBase::m_configuration,
                        "Configuration", "Lead/follower entities and local joint frame")
                    // Renders the Edit button that enters component mode.
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltJointComponentBase::m_componentModeDelegate,
                        "Component Mode", "Joint frame component mode")
                    ;
            }
        }
    }

    void EditorJoltJointComponentBase::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltJointService"));
    }

    void EditorJoltJointComponentBase::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltJointService"));
    }

    void EditorJoltJointComponentBase::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void EditorJoltJointComponentBase::Activate()
    {
        AzToolsFramework::Components::EditorComponentBase::Activate();

        const AZ::EntityComponentIdPair entityComponentIdPair(GetEntityId(), GetId());
        AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(GetEntityId());
        JoltJointFrameRequestBus::Handler::BusConnect(entityComponentIdPair);
        // Addressed by entity, not by entity+component, unlike the frame bus.
        AzToolsFramework::EditorComponentSelectionRequestsBus::Handler::BusConnect(GetEntityId());

        // The component mode itself is connected by each derived joint, which alone
        // knows its own type - see ConnectJointComponentMode.
    }

    void EditorJoltJointComponentBase::Deactivate()
    {
        m_componentModeDelegate.Disconnect();

        AzToolsFramework::EditorComponentSelectionRequestsBus::Handler::BusDisconnect();
        JoltJointFrameRequestBus::Handler::BusDisconnect();
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();

        AzToolsFramework::Components::EditorComponentBase::Deactivate();
    }

    AZ::Transform EditorJoltJointComponentBase::GetJointFrameSpace() const
    {
        // The frame is expressed in the follower's space; the follower is this entity
        // unless one was named explicitly, matching JoltJointComponentBase's own rule.
        const AZ::EntityId followerId =
            m_configuration.m_followerEntity.IsValid() ? m_configuration.m_followerEntity : GetEntityId();

        AZ::Transform followerTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(followerTransform, followerId, &AZ::TransformBus::Events::GetWorldTM);
        // Scale would stretch the frame, the limit cones and the manipulators without
        // changing anything the joint actually does.
        followerTransform.ExtractUniformScale();
        return followerTransform;
    }

    AZ::Transform EditorJoltJointComponentBase::GetJointLocalFrame() const
    {
        return m_configuration.m_localTransformFromFollower;
    }

    void EditorJoltJointComponentBase::SetJointLocalFrame(const AZ::Transform& localFrame)
    {
        m_configuration.m_localTransformFromFollower = localFrame;

        // Values only: rebuilding the property tree mid-drag destroys and recreates the
        // manipulators under the cursor.
        AzToolsFramework::ToolsApplicationEvents::Bus::Broadcast(
            &AzToolsFramework::ToolsApplicationEvents::InvalidatePropertyDisplay,
            AzToolsFramework::Refresh_Values);
    }

    AZ::Aabb EditorJoltJointComponentBase::GetEditorSelectionBoundsViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo)
    {
        // A joint has no geometry, so it is picked against the frame it draws.
        return AZ::Aabb::CreateCenterHalfExtents(
            GetJointWorldTransform().GetTranslation(), AZ::Vector3(EditorDebugDraw::JointAxisLength));
    }

    bool EditorJoltJointComponentBase::SupportsEditorRayIntersect()
    {
        return false;
    }

    AZ::Transform EditorJoltJointComponentBase::GetJointWorldTransform() const
    {
        return GetJointFrameSpace() * m_configuration.m_localTransformFromFollower;
    }

    void EditorJoltJointComponentBase::DisplayEntityViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo,
        AzFramework::DebugDisplayRequests& debugDisplay)
    {
        const AZ::Transform jointTransform = GetJointWorldTransform();

        EditorDebugDraw::DrawJointFrame(debugDisplay, jointTransform);
        DrawJointLimits(debugDisplay, jointTransform);

        // A line to the lead body makes it obvious what the joint is attached to,
        // which is otherwise only visible by reading the Configuration fields.
        if (m_configuration.m_leadEntity.IsValid())
        {
            AZ::Transform leadTransform = AZ::Transform::CreateIdentity();
            AZ::TransformBus::EventResult(
                leadTransform, m_configuration.m_leadEntity, &AZ::TransformBus::Events::GetWorldTM);
            EditorDebugDraw::DrawLine(
                debugDisplay, jointTransform.GetTranslation(), leadTransform.GetTranslation(),
                EditorDebugDraw::LinkColor);
        }
    }

} // namespace JoltPhysics
