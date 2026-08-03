#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <AzCore/Math/Quaternion.h>

#include <Editor/JoltEditorJointHelpers.h>
#include <Joint/JoltJointConfiguration.h>
#include <Joint/JoltJointLimitMath.h>

#include "JoltTestWarningCatcher.h"

namespace JoltPhysics
{
    //! The joint-limit auto-fit the Animation Editor offers: hand it the poses an
    //! animation puts a bone through and it comes back with limits that admit them.
    class JoltEditorJointHelpersTests : public ::testing::Test
    {
    protected:
        JoltEditorJointHelpers m_helpers;
    };

    TEST_F(JoltEditorJointHelpersTests, TheFitTightensAWideLimitOntoTheSamples)
    {
        // A limit left at its defaults, and an animation that never goes near them.
        JoltD6JointLimitConfiguration current;
        current.m_swingLimitY = 90.0f;
        current.m_swingLimitZ = 90.0f;
        current.m_twistLimitLower = -90.0f;
        current.m_twistLimitUpper = 90.0f;

        const AZStd::vector<AZ::Quaternion> samples = {
            AZ::Quaternion::CreateRotationX(AZ::DegToRad(12.0f)),
            AZ::Quaternion::CreateRotationX(AZ::DegToRad(-8.0f)),
            AZ::Quaternion::CreateRotationZ(AZ::DegToRad(20.0f)),
        };

        AZStd::unique_ptr<AzPhysics::JointConfiguration> fitted = m_helpers.ComputeOptimalJointLimit(&current, samples);
        ASSERT_NE(fitted, nullptr);

        auto* d6 = azrtti_cast<JoltD6JointLimitConfiguration*>(fitted.get());
        ASSERT_NE(d6, nullptr);

        EXPECT_LT(d6->m_swingLimitY, current.m_swingLimitY) << "the fit did not tighten anything";
        EXPECT_GT(d6->m_twistLimitLower, current.m_twistLimitLower);
        EXPECT_LT(d6->m_twistLimitUpper, current.m_twistLimitUpper);

        // Tighter, but still admitting every pose it was shown - a fit that excluded one
        // would have the ragdoll fighting the animation it was derived from.
        for (const AZ::Quaternion& sample : samples)
        {
            const JointLimitMath::SwingTwist decomposed = JointLimitMath::DecomposeSwingTwist(sample);
            EXPECT_LE(fabsf(decomposed.m_swingYDegrees), d6->m_swingLimitY);
            EXPECT_LE(fabsf(decomposed.m_swingZDegrees), d6->m_swingLimitZ);
            EXPECT_GE(decomposed.m_twistDegrees, d6->m_twistLimitLower);
            EXPECT_LE(decomposed.m_twistDegrees, d6->m_twistLimitUpper);
        }

        // The frames are the author's, not the fit's: it bounds the motion, it does not
        // move the joint.
        EXPECT_TRUE(d6->m_parentLocalRotation.IsClose(current.m_parentLocalRotation));
        EXPECT_TRUE(d6->m_childLocalRotation.IsClose(current.m_childLocalRotation));
    }

    TEST_F(JoltEditorJointHelpersTests, NoSamplesLeavesTheAuthoredLimitUntouched)
    {
        // Nothing was observed, so nothing was learned. Narrowing here would quietly
        // rewrite limits someone had set by hand.
        JoltD6JointLimitConfiguration current;
        current.m_swingLimitY = 33.0f;
        current.m_swingLimitZ = 44.0f;
        current.m_twistLimitLower = -11.0f;
        current.m_twistLimitUpper = 22.0f;

        AZStd::unique_ptr<AzPhysics::JointConfiguration> fitted = m_helpers.ComputeOptimalJointLimit(&current, {});
        ASSERT_NE(fitted, nullptr);

        auto* d6 = azrtti_cast<JoltD6JointLimitConfiguration*>(fitted.get());
        ASSERT_NE(d6, nullptr);
        EXPECT_FLOAT_EQ(d6->m_swingLimitY, 33.0f);
        EXPECT_FLOAT_EQ(d6->m_swingLimitZ, 44.0f);
        EXPECT_FLOAT_EQ(d6->m_twistLimitLower, -11.0f);
        EXPECT_FLOAT_EQ(d6->m_twistLimitUpper, 22.0f);
    }

    TEST_F(JoltEditorJointHelpersTests, ALimitWithNothingToFitIsRefusedRatherThanReshaped)
    {
        JoltHingeJointConfiguration hinge;

        JoltWarningCatcher warnings;
        AZStd::unique_ptr<AzPhysics::JointConfiguration> fitted = m_helpers.ComputeOptimalJointLimit(
            &hinge, { AZ::Quaternion::CreateRotationX(AZ::DegToRad(10.0f)) });

        EXPECT_EQ(fitted, nullptr);
        EXPECT_FALSE(warnings.m_warnings.empty()) << "the refusal was silent";
    }

    TEST_F(JoltEditorJointHelpersTests, ANullConfigurationIsAnswered)
    {
        EXPECT_EQ(m_helpers.ComputeOptimalJointLimit(nullptr, {}), nullptr);
    }
} // namespace JoltPhysics
