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

        // The samples are already in the joint's own frame: the caller measured the child
        // against the parent through the frames this configuration carries, which is the
        // same space ComputeInitialJointLimitConfiguration fitted in.
        auto optimized = AZStd::make_unique<JoltD6JointLimitConfiguration>(*d6);
        if (localRotationSamples.empty())
        {
            // Nothing observed, so nothing learned. Widening or narrowing on no evidence
            // would silently move limits the author set by hand.
            return optimized;
        }

        const JointLimitMath::SwingTwistLimits limits = JointLimitMath::FitLimits(localRotationSamples);
        optimized->m_swingLimitY = limits.m_swingYDegrees;
        optimized->m_swingLimitZ = limits.m_swingZDegrees;
        optimized->m_twistLimitLower = limits.m_twistLowerDegrees;
        optimized->m_twistLimitUpper = limits.m_twistUpperDegrees;
        return optimized;
    }
} // namespace JoltPhysics
