#include <Editor/JoltEditorJointHelpers.h>

#include <AzCore/std/smart_ptr/unique_ptr.h>

#include <Joint/JoltJointConfiguration.h>
#include <Joint/JoltJointLimitMath.h>

namespace JoltPhysics
{
    AZStd::unique_ptr<AzPhysics::JointConfiguration> JoltEditorJointHelpers::ComputeOptimalJointLimit(
        const AzPhysics::JointConfiguration* currentConfiguration,
        const AZStd::vector<AZ::Quaternion>& localRotationSamples)
    {
        if (currentConfiguration == nullptr)
        {
            return nullptr;
        }

        const auto* d6 = azrtti_cast<const JoltD6JointLimitConfiguration*>(currentConfiguration);
        if (d6 == nullptr)
        {
            // Only the 6-DOF limit has both a cone and a twist range to fit. Handing back
            // a copy leaves the author's configuration as they left it, which is the right
            // answer for a joint with nothing to optimise - better than a null they would
            // have to guard, or a silently different type.
            AZ_Warning("JoltPhysics", false,
                "Joint limit auto-fit is only defined for the 6-DOF limit; the configuration was returned unchanged.");
            return nullptr;
        }

        auto optimized = AZStd::make_unique<JoltD6JointLimitConfiguration>(*d6);
        if (localRotationSamples.empty())
        {
            // Nothing observed, so nothing learned. Widening or narrowing on no evidence
            // would silently move limits the author set by hand.
            return optimized;
        }

        // The samples are raw joint rotations sampled straight out of a motion - the
        // child relative to its parent, in the parent bone's space, not the joint's. They
        // have to be brought into the joint frame before anything is measured, the same
        // way ComputeInitialJointLimitConfiguration does with its example poses; fitting
        // them as they arrive describes rotations about the parent bone's axes instead.
        AZStd::vector<AZ::Quaternion> jointFrameSamples;
        jointFrameSamples.reserve(localRotationSamples.size());
        for (const AZ::Quaternion& sample : localRotationSamples)
        {
            jointFrameSamples.push_back(JointLimitMath::RelativeRotationInJointFrame(
                d6->m_parentLocalRotation, sample, d6->m_childLocalRotation));
        }

        const JointLimitMath::SwingTwistLimits limits = JointLimitMath::FitLimits(jointFrameSamples);
        optimized->m_swingLimitY = limits.m_swingYDegrees;
        optimized->m_swingLimitZ = limits.m_swingZDegrees;
        optimized->m_twistLimitLower = limits.m_twistLowerDegrees;
        optimized->m_twistLimitUpper = limits.m_twistUpperDegrees;
        return optimized;
    }
} // namespace JoltPhysics
