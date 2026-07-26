#pragma once

#include <Clients/Components/JoltJointComponentBase.h>

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzToolsFramework/API/ComponentEntitySelectionBus.h>
#include <AzToolsFramework/ComponentMode/ComponentModeDelegate.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

#include <Editor/Components/JoltJointComponentMode.h>
#include <Editor/Components/JoltJointFrameRequestBus.h>

namespace JoltPhysics
{
    //! Editor-side base for Jolt joint components (PhysX-style editor/runtime
    //! split). Holds the shared joint configuration (lead/follower entities and
    //! local frame); derived classes hold their type-specific properties and
    //! spawn the matching runtime joint component via BuildGameEntity.
    //!
    //! Also draws every joint's frame in the viewport, plus a link to the lead
    //! entity. Derived classes add their own limit visualisation by overriding
    //! DrawJointLimits.
    //!
    //! The frame is editable in the viewport too: the base serves
    //! JoltJointFrameRequestBus and owns the ComponentModeDelegate that puts the Edit
    //! button on the component, so every joint type gets drag handles from
    //! JoltJointComponentMode without implementing anything itself.
    class EditorJoltJointComponentBase
        : public AzToolsFramework::Components::EditorComponentBase
        , private AzFramework::EntityDebugDisplayEventBus::Handler
        , protected JoltJointFrameRequestBus::Handler
        , protected AzToolsFramework::EditorComponentSelectionRequestsBus::Handler
    {
    public:
        AZ_RTTI(EditorJoltJointComponentBase, "{F2A3B4C5-D6E7-4890-E1F2-A3B4C5D6E7F8}", AzToolsFramework::Components::EditorComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // EditorComponentBase
        void Activate() override;
        void Deactivate() override;

        //! The lead/follower entities and the local joint frame, mirroring the accessor
        //! the runtime joint components expose.
        JoltJointComponentConfiguration& GetJointConfiguration()
        {
            return m_configuration;
        }
        const JoltJointComponentConfiguration& GetJointConfiguration() const
        {
            return m_configuration;
        }

    protected:
        //! The joint frame in world space: the follower's transform combined with the
        //! configured local frame. X is the primary axis, Y the normal/plane reference.
        AZ::Transform GetJointWorldTransform() const;

        //! Connects the component-mode delegate for the concrete joint type, which each
        //! joint calls from its own Activate. The delegate identifies the mode by the
        //! component's type Uuid, so it cannot be given this base class - a joint
        //! connecting as the base would put its Edit button on the wrong component.
        template<typename EditorJointComponentType>
        void ConnectJointComponentMode()
        {
            const AZ::EntityComponentIdPair entityComponentIdPair(GetEntityId(), GetId());
            m_componentModeDelegate.ConnectWithSingleComponentMode<EditorJointComponentType, JoltJointComponentMode>(
                entityComponentIdPair, this);
        }

        //! Draws the type-specific limits, given the joint frame in world space.
        //! The default draws nothing, which is right for joints that have no limits
        //! to speak of (fixed) and for any type whose limits are disabled.
        virtual void DrawJointLimits(
            [[maybe_unused]] AzFramework::DebugDisplayRequests& debugDisplay,
            [[maybe_unused]] const AZ::Transform& jointTransform) const
        {
        }

        // JoltJointFrameRequestBus
        AZ::Transform GetJointLocalFrame() const override;
        void SetJointLocalFrame(const AZ::Transform& localFrame) override;
        AZ::Transform GetJointFrameSpace() const override;

        // AzToolsFramework::EditorComponentSelectionRequestsBus - required by the
        // ComponentModeDelegate, and what the editor picks the joint against.
        AZ::Aabb GetEditorSelectionBoundsViewport(const AzFramework::ViewportInfo& viewportInfo) override;
        bool SupportsEditorRayIntersect() override;

        JoltJointComponentConfiguration m_configuration;

        //! Puts the Edit button on the component in the inspector and enters component
        //! mode on double click.
        AzToolsFramework::ComponentModeFramework::ComponentModeDelegate m_componentModeDelegate;

    private:
        // AzFramework::EntityDebugDisplayEvents
        void DisplayEntityViewport(
            const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;
    };
} // namespace JoltPhysics
