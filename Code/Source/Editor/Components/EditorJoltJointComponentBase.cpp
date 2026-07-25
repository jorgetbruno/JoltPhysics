#include <Editor/Components/EditorJoltJointComponentBase.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Editor/Components/EditorJoltDebugDrawUtils.h>

namespace JoltPhysics
{
    void EditorJoltJointComponentBase::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltJointComponentBase, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(1)
                ->Field("Configuration", &EditorJoltJointComponentBase::m_configuration)
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
        AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(GetEntityId());
    }

    void EditorJoltJointComponentBase::Deactivate()
    {
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();
        AzToolsFramework::Components::EditorComponentBase::Deactivate();
    }

    AZ::Transform EditorJoltJointComponentBase::GetJointWorldTransform() const
    {
        // The frame is expressed in the follower's space; the follower is this entity
        // unless one was named explicitly, matching JoltJointComponentBase's own rule.
        const AZ::EntityId followerId =
            m_configuration.m_followerEntity.IsValid() ? m_configuration.m_followerEntity : GetEntityId();

        AZ::Transform followerTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(followerTransform, followerId, &AZ::TransformBus::Events::GetWorldTM);
        // Scale would stretch the frame and the limit cones without changing anything
        // the joint actually does.
        followerTransform.ExtractUniformScale();

        return followerTransform * m_configuration.m_localTransformFromFollower;
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
