#pragma once

#include <AzCore/Interface/Interface.h>

#include <AzFramework/Physics/Common/PhysicsJoint.h>

namespace JoltPhysics
{
    //! Answers AzPhysics::EditorJointHelpersInterface: given a joint limit and a set of
    //! poses the joint is known to reach, produce a tighter limit that still contains
    //! them all.
    //!
    //! This is the "auto-fit" the Animation Editor's ragdoll joint-limit widget calls
    //! when an author asks it to derive limits from an animation. Editor-only, exactly as
    //! PhysX splits it, because it exists to serve an authoring gesture and has no
    //! runtime caller.
    class JoltEditorJointHelpers
        : public AZ::Interface<AzPhysics::EditorJointHelpersInterface>::Registrar
    {
    public:
        AZ_RTTI(
            JoltPhysics::JoltEditorJointHelpers,
            "{7D41B0A5-6E2C-4F98-B1D7-3A5C9E048F26}",
            AzPhysics::EditorJointHelpersInterface);

        // AzPhysics::EditorJointHelpersInterface
        AZStd::unique_ptr<AzPhysics::JointConfiguration> ComputeOptimalJointLimit(
            const AzPhysics::JointConfiguration* currentConfiguration,
            const AZStd::vector<AZ::Quaternion>& localRotationSamples) override;
    };
} // namespace JoltPhysics
