#pragma once

#include <Clients/Components/JoltJointComponentBase.h>

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

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
    class EditorJoltJointComponentBase
        : public AzToolsFramework::Components::EditorComponentBase
        , private AzFramework::EntityDebugDisplayEventBus::Handler
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

    protected:
        //! The joint frame in world space: the follower's transform combined with the
        //! configured local frame. X is the primary axis, Y the normal/plane reference.
        AZ::Transform GetJointWorldTransform() const;

        //! Draws the type-specific limits, given the joint frame in world space.
        //! The default draws nothing, which is right for joints that have no limits
        //! to speak of (fixed) and for any type whose limits are disabled.
        virtual void DrawJointLimits(
            [[maybe_unused]] AzFramework::DebugDisplayRequests& debugDisplay,
            [[maybe_unused]] const AZ::Transform& jointTransform) const
        {
        }

        JoltJointComponentConfiguration m_configuration;

    private:
        // AzFramework::EntityDebugDisplayEvents
        void DisplayEntityViewport(
            const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;
    };
} // namespace JoltPhysics
