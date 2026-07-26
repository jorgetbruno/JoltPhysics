#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Math/Transform.h>

namespace JoltPhysics
{
    //! The joint frame, as the viewport edits it.
    //!
    //! Lets JoltJointComponentMode drive any joint without knowing which type it has:
    //! every editor joint carries the same frame, expressed in its follower entity's
    //! local space. Addressed by entity and component id, since an entity can hold more
    //! than one joint.
    class JoltJointFrameRequests : public AZ::EntityComponentBus
    {
    public:
        //! The joint frame in the follower entity's local space - the value the
        //! inspector shows as "Joint frame".
        virtual AZ::Transform GetJointLocalFrame() const = 0;
        virtual void SetJointLocalFrame(const AZ::Transform& localFrame) = 0;

        //! The space that frame is expressed in: the follower's world transform, with
        //! scale removed. Scale would stretch the manipulators without changing anything
        //! the joint actually does.
        virtual AZ::Transform GetJointFrameSpace() const = 0;

    protected:
        ~JoltJointFrameRequests() = default;
    };

    using JoltJointFrameRequestBus = AZ::EBus<JoltJointFrameRequests>;
} // namespace JoltPhysics
