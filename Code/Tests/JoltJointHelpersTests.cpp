#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Vector3.h>

#include <algorithm>

#include "JoltTestWarningCatcher.h"

#include <Joint/JoltJointConfiguration.h>
#include <Joint/JoltJointHelpers.h>
#include <Joint/JoltJointLimitMath.h>

namespace JoltPhysics
{
    //! The joint authoring surface the Animation Editor's ragdoll tools drive: what joint
    //! types this backend can author, where a joint frame lands, how wide its limits have
    //! to be to admit a set of poses, and the geometry that draws all of it.
    class JoltJointHelpersTests : public ::testing::Test
    {
    protected:
        static constexpr float Tolerance = 0.1f; // degrees

        //! Twist about the joint frame's X axis.
        static AZ::Quaternion Twist(float degrees)
        {
            return AZ::Quaternion::CreateRotationX(AZ::DegToRad(degrees));
        }

        JoltJointHelpers m_helpers;
    };

    TEST_F(JoltJointHelpersTests, APureTwistDecomposesToTwistAlone)
    {
        const JointLimitMath::SwingTwist decomposed = JointLimitMath::DecomposeSwingTwist(Twist(30.0f));

        EXPECT_NEAR(decomposed.m_twistDegrees, 30.0f, Tolerance);
        EXPECT_NEAR(decomposed.m_swingYDegrees, 0.0f, Tolerance);
        EXPECT_NEAR(decomposed.m_swingZDegrees, 0.0f, Tolerance);
    }

    TEST_F(JoltJointHelpersTests, SwingTowardsEachAxisIsReportedSeparately)
    {
        // The two swing angles have to stay independent: a limit cone is authored as two
        // half-angles, one per axis, and an axis-angle reading would collapse them into
        // one number and lose which way the bone actually bends.
        //
        // Stated as tilts rather than as rotations, because the two name opposite axes -
        // a rotation *about* Y leans X towards Z - and that crossing is exactly what this
        // is here to pin.
        const JointLimitMath::SwingTwist towardsY =
            JointLimitMath::DecomposeSwingTwist(AZ::Quaternion::CreateRotationZ(AZ::DegToRad(25.0f)));
        EXPECT_NEAR(towardsY.m_swingYDegrees, 25.0f, Tolerance);
        EXPECT_NEAR(towardsY.m_swingZDegrees, 0.0f, Tolerance);
        EXPECT_NEAR(towardsY.m_twistDegrees, 0.0f, Tolerance);

        const JointLimitMath::SwingTwist towardsZ =
            JointLimitMath::DecomposeSwingTwist(AZ::Quaternion::CreateRotationY(AZ::DegToRad(-15.0f)));
        EXPECT_NEAR(towardsZ.m_swingZDegrees, 15.0f, Tolerance);
        EXPECT_NEAR(towardsZ.m_swingYDegrees, 0.0f, Tolerance);
        EXPECT_NEAR(towardsZ.m_twistDegrees, 0.0f, Tolerance);
    }

    TEST_F(JoltJointHelpersTests, ATwistPastAHalfTurnReadsAsTheShorterWayRound)
    {
        // Otherwise a fitted range would run from -180 to +190 and admit everything.
        const JointLimitMath::SwingTwist decomposed = JointLimitMath::DecomposeSwingTwist(Twist(-170.0f));
        EXPECT_NEAR(decomposed.m_twistDegrees, -170.0f, 1.0f);
    }

    TEST_F(JoltJointHelpersTests, FittedLimitsAdmitEverySampleAndNothingIsZeroWidth)
    {
        const AZStd::vector<AZ::Quaternion> samples = {
            Twist(10.0f),
            Twist(-20.0f),
            AZ::Quaternion::CreateRotationZ(AZ::DegToRad(30.0f)),
            AZ::Quaternion::CreateRotationY(AZ::DegToRad(-12.0f)),
        };

        const JointLimitMath::SwingTwistLimits limits = JointLimitMath::FitLimits(samples);

        for (const AZ::Quaternion& sample : samples)
        {
            const JointLimitMath::SwingTwist decomposed = JointLimitMath::DecomposeSwingTwist(sample);
            EXPECT_LE(fabsf(decomposed.m_swingYDegrees), limits.m_swingYDegrees);
            EXPECT_LE(fabsf(decomposed.m_swingZDegrees), limits.m_swingZDegrees);
            EXPECT_GE(decomposed.m_twistDegrees, limits.m_twistLowerDegrees);
            EXPECT_LE(decomposed.m_twistDegrees, limits.m_twistUpperDegrees);
        }

        // A limit fitted to poses that never leave one axis must still leave the others
        // some room, or the fit would pin a bone the author had simply not exercised.
        EXPECT_GT(limits.m_swingYDegrees, 0.0f);
        EXPECT_GT(limits.m_swingZDegrees, 0.0f);
        EXPECT_LT(limits.m_twistLowerDegrees, limits.m_twistUpperDegrees);
    }

    TEST_F(JoltJointHelpersTests, NoSamplesLeavesTheDefaultsAlone)
    {
        // "Nothing observed" is not "nothing allowed": with no poses to learn from, a
        // zero-width limit would weld the bone solid the moment the ragdoll ran.
        const JointLimitMath::SwingTwistLimits limits = JointLimitMath::FitLimits({});
        EXPECT_GT(limits.m_swingYDegrees, 0.0f);
        EXPECT_GT(limits.m_swingZDegrees, 0.0f);
        EXPECT_LT(limits.m_twistLowerDegrees, limits.m_twistUpperDegrees);
    }

    TEST_F(JoltJointHelpersTests, TheBackendReportsTheJointTypesItCanAuthor)
    {
        // The Animation Editor asks this before it offers a joint type in its dropdown;
        // with nothing registered - which is what a Jolt project had until this existed -
        // it has no types to offer at all.
        const AZStd::vector<AZ::TypeId> supported = m_helpers.GetSupportedJointTypeIds();
        EXPECT_FALSE(supported.empty());

        const auto d6 = m_helpers.GetSupportedJointTypeId(AzPhysics::JointType::D6Joint);
        ASSERT_TRUE(d6.has_value());
        EXPECT_EQ(*d6, azrtti_typeid<JoltD6JointLimitConfiguration>());

        const auto hinge = m_helpers.GetSupportedJointTypeId(AzPhysics::JointType::HingeJoint);
        ASSERT_TRUE(hinge.has_value());
        EXPECT_EQ(*hinge, azrtti_typeid<JoltHingeJointConfiguration>());

        const auto ball = m_helpers.GetSupportedJointTypeId(AzPhysics::JointType::BallJoint);
        ASSERT_TRUE(ball.has_value());
        EXPECT_EQ(*ball, azrtti_typeid<JoltBallJointConfiguration>());

        const auto fixed = m_helpers.GetSupportedJointTypeId(AzPhysics::JointType::FixedJoint);
        ASSERT_TRUE(fixed.has_value());
        EXPECT_EQ(*fixed, azrtti_typeid<JoltFixedJointConfiguration>());

        for (const AZ::TypeId& typeId : { *d6, *hinge, *ball, *fixed })
        {
            EXPECT_NE(std::find(supported.begin(), supported.end(), typeId), supported.end());
        }
    }

    TEST_F(JoltJointHelpersTests, TheComputedFrameSitsOnTheRequestedAxisAndReadsAsRest)
    {
        // The bone direction a ragdoll hands over, in WORLD space - EMotionFX resolves it
        // before calling. The child body is deliberately rotated, so reading the axis as
        // child-local instead would land the frame somewhere else entirely and be caught
        // here.
        const AZ::Vector3 axis = AZ::Vector3(0.0f, 1.0f, 0.0f);
        const AZ::Quaternion parentWorldRotation = AZ::Quaternion::CreateRotationZ(AZ::DegToRad(35.0f));
        const AZ::Quaternion childWorldRotation = AZ::Quaternion::CreateRotationX(AZ::DegToRad(-20.0f));

        AZStd::unique_ptr<AzPhysics::JointConfiguration> configuration = m_helpers.ComputeInitialJointLimitConfiguration(
            azrtti_typeid<JoltD6JointLimitConfiguration>(), parentWorldRotation, childWorldRotation, axis, {});
        ASSERT_NE(configuration, nullptr);

        // The joint's X axis is its twist axis, and in world space it must be the axis
        // that was asked for - not that axis pushed through the child's rotation.
        const AZ::Quaternion jointWorldRotation = childWorldRotation * configuration->m_childLocalRotation;
        const AZ::Vector3 jointX = jointWorldRotation.TransformVector(AZ::Vector3::CreateAxisX());
        EXPECT_TRUE(jointX.IsClose(axis.GetNormalized(), 1e-3f));

        // And the parent names the same frame.
        const AZ::Vector3 jointXFromParent =
            (parentWorldRotation * configuration->m_parentLocalRotation).TransformVector(AZ::Vector3::CreateAxisX());
        EXPECT_TRUE(jointXFromParent.IsClose(axis.GetNormalized(), 1e-3f));

        // Both bodies name the same frame, so the pose it was computed from is the joint's
        // rest pose: zero swing, zero twist. Without that, a ragdoll would start life
        // already pushing against its own limits.
        const AZ::Quaternion childRelativeToParent = parentWorldRotation.GetConjugate() * childWorldRotation;
        const AZ::Quaternion relativeInJointFrame = configuration->m_parentLocalRotation.GetConjugate() *
            childRelativeToParent * configuration->m_childLocalRotation;
        const JointLimitMath::SwingTwist atRest = JointLimitMath::DecomposeSwingTwist(relativeInJointFrame);
        EXPECT_NEAR(atRest.m_twistDegrees, 0.0f, Tolerance);
        EXPECT_NEAR(atRest.m_swingYDegrees, 0.0f, Tolerance);
        EXPECT_NEAR(atRest.m_swingZDegrees, 0.0f, Tolerance);
    }

    TEST_F(JoltJointHelpersTests, AHingeTakesTheTwistRangeAndDropsTheOffAxisSwing)
    {
        // A hinge has one degree of freedom. Poses that swing off its axis describe motion
        // it will never permit, so widening the hinge to admit them would be describing a
        // joint that does not exist.
        const AZStd::vector<AZ::Quaternion> samples = {
            Twist(40.0f),
            Twist(-10.0f),
            AZ::Quaternion::CreateRotationZ(AZ::DegToRad(50.0f)),
        };

        AZStd::unique_ptr<AzPhysics::JointConfiguration> configuration = m_helpers.ComputeInitialJointLimitConfiguration(
            azrtti_typeid<JoltHingeJointConfiguration>(), AZ::Quaternion::CreateIdentity(),
            AZ::Quaternion::CreateIdentity(), AZ::Vector3::CreateAxisX(), samples);
        ASSERT_NE(configuration, nullptr);

        auto* hinge = azrtti_cast<JoltHingeJointConfiguration*>(configuration.get());
        ASSERT_NE(hinge, nullptr);
        EXPECT_TRUE(hinge->m_limitProperties.m_isLimited);
        EXPECT_LE(hinge->m_limitProperties.m_limitFirst, -10.0f);
        EXPECT_GE(hinge->m_limitProperties.m_limitSecond, 40.0f);
    }

    TEST_F(JoltJointHelpersTests, AnUnsupportedLimitTypeIsRefusedRatherThanGuessed)
    {
        // Refused, not approximated: handing back some other joint's limit would put a
        // configuration the caller did not ask for onto a ragdoll node.
        JoltWarningCatcher warnings;
        AZStd::unique_ptr<AzPhysics::JointConfiguration> configuration = m_helpers.ComputeInitialJointLimitConfiguration(
            azrtti_typeid<JoltDistanceJointConfiguration>(), AZ::Quaternion::CreateIdentity(),
            AZ::Quaternion::CreateIdentity(), AZ::Vector3::CreateAxisX(), {});

        EXPECT_EQ(configuration, nullptr);
        EXPECT_FALSE(warnings.m_warnings.empty()) << "the refusal was silent";
    }

    TEST_F(JoltJointHelpersTests, LimitsAreDrawnAsLinesInTheJointFrameOfTheParentBody)
    {
        // The caller transforms every point by the parent body's world transform before
        // drawing it (CharacterPhysicsDebugDraw::RenderJointLimit), so the geometry must
        // come out in the joint frame *within the parent* - carrying only
        // m_parentLocalRotation. Folding the parent's own rotation in here as well would
        // rotate the whole drawing twice, which is exactly what this catches: the parent
        // is rotated a quarter turn and the child is not.
        JoltD6JointLimitConfiguration configuration;
        configuration.m_swingLimitY = 20.0f;
        configuration.m_swingLimitZ = 20.0f;
        configuration.m_parentLocalRotation = AZ::Quaternion::CreateIdentity();
        configuration.m_childLocalRotation = AZ::Quaternion::CreateIdentity();

        AZStd::vector<AZ::Vector3> vertexBuffer;
        AZStd::vector<AZ::u32> indexBuffer;
        AZStd::vector<AZ::Vector3> lineBuffer;
        AZStd::vector<bool> lineValidityBuffer;

        const AZ::Quaternion parentRotation = AZ::Quaternion::CreateRotationZ(AZ::DegToRad(90.0f));
        m_helpers.GenerateJointLimitVisualizationData(
            configuration, parentRotation, parentRotation, 1.0f, 16, 2, vertexBuffer, indexBuffer, lineBuffer,
            lineValidityBuffer);

        ASSERT_FALSE(lineBuffer.empty());
        EXPECT_EQ(lineBuffer.size() % 2, 0u) << "lines are drawn as point pairs";
        EXPECT_EQ(lineValidityBuffer.size(), lineBuffer.size() / 2)
            << "validity is reported once per line, not once per point";

        // The cone opens along the joint frame's X. With an identity parent-local
        // rotation that is +X, whatever the parent body itself is doing.
        float furthestAlongX = 0.0f;
        for (const AZ::Vector3& point : lineBuffer)
        {
            furthestAlongX = AZ::GetMax(furthestAlongX, point.GetX());
        }
        EXPECT_GT(furthestAlongX, 0.5f) << "the drawing was rotated out of the joint frame";

        // Nothing writes the triangle buffers: the renderer never draws them.
        EXPECT_TRUE(vertexBuffer.empty());
        EXPECT_TRUE(indexBuffer.empty());
    }

    TEST_F(JoltJointHelpersTests, TheDrawnConeStaysInsideTheAnglesItWasAuthoredWith)
    {
        JoltD6JointLimitConfiguration configuration;
        configuration.m_swingLimitY = 20.0f;
        configuration.m_swingLimitZ = 40.0f;
        // Twist limits wide open, so the arc lines do not muddy the cone measurement.
        configuration.m_twistLimitLower = -1.0f;
        configuration.m_twistLimitUpper = 1.0f;

        AZStd::vector<AZ::Vector3> vertexBuffer;
        AZStd::vector<AZ::u32> indexBuffer;
        AZStd::vector<AZ::Vector3> lineBuffer;
        AZStd::vector<bool> lineValidityBuffer;

        JointLimitMath::AppendSwingConeLines(
            configuration.m_swingLimitY, configuration.m_swingLimitZ, 0.0f, 0.0f, AZ::Quaternion::CreateIdentity(),
            1.0f, 16, 2, lineBuffer, lineValidityBuffer);

        ASSERT_FALSE(lineBuffer.empty());
        for (const AZ::Vector3& point : lineBuffer)
        {
            if (point.GetLengthSq() < 1e-6f)
            {
                continue; // the apex
            }
            const AZ::Vector3 direction = point.GetNormalized();
            const float swingY = AZ::RadToDeg(atan2f(direction.GetY(), direction.GetX()));
            const float swingZ = AZ::RadToDeg(atan2f(direction.GetZ(), direction.GetX()));
            EXPECT_LE(fabsf(swingY), configuration.m_swingLimitY + Tolerance);
            EXPECT_LE(fabsf(swingZ), configuration.m_swingLimitZ + Tolerance);
        }
    }

    TEST_F(JoltJointHelpersTests, AnEllipticalConeRejectsASwingThatIsInsideBothHalfAnglesAlone)
    {
        // 15 degrees each way is inside a 20 x 20 cone taken axis by axis, but outside the
        // ellipse those two half-angles actually describe. Checking the axes independently
        // would call this fine and draw a limit the solver would then break.
        EXPECT_TRUE(JointLimitMath::IsSwingWithinLimits(19.0f, 0.0f, 20.0f, 20.0f));
        EXPECT_TRUE(JointLimitMath::IsSwingWithinLimits(0.0f, 19.0f, 20.0f, 20.0f));
        EXPECT_FALSE(JointLimitMath::IsSwingWithinLimits(15.0f, 15.0f, 20.0f, 20.0f));
    }

    TEST_F(JoltJointHelpersTests, AViolatedLimitMarksItsWholeDrawingAsViolated)
    {
        // The renderer colours per line, so a limit drawn half in the error colour would
        // read as half of it being broken rather than the joint being outside it. The
        // whole cone, or the whole arc, carries one verdict.
        JoltD6JointLimitConfiguration configuration;
        configuration.m_twistLimitLower = -30.0f;
        configuration.m_twistLimitUpper = 30.0f;
        configuration.m_swingLimitY = 45.0f;
        configuration.m_swingLimitZ = 45.0f;

        AZStd::vector<AZ::Vector3> vertexBuffer;
        AZStd::vector<AZ::u32> indexBuffer;
        AZStd::vector<AZ::Vector3> lineBuffer;
        AZStd::vector<bool> lineValidityBuffer;

        m_helpers.GenerateJointLimitVisualizationData(
            configuration, AZ::Quaternion::CreateIdentity(), Twist(80.0f), 1.0f, 16, 2, vertexBuffer, indexBuffer,
            lineBuffer, lineValidityBuffer);

        ASSERT_FALSE(lineValidityBuffer.empty());
        const size_t violated =
            static_cast<size_t>(std::count(lineValidityBuffer.begin(), lineValidityBuffer.end(), false));
        EXPECT_GT(violated, 0u) << "a twist of 80 degrees against a 30 degree limit drew nothing as violated";

        // The swing is untouched by a pure twist, so its cone stays valid - and the
        // current-twist marker is a readout rather than a limit, so it does too.
        EXPECT_LT(violated, lineValidityBuffer.size()) << "the swing cone was condemned along with the twist";
    }

    TEST_F(JoltJointHelpersTests, AJointInsideItsLimitsDrawsNothingViolated)
    {
        JoltD6JointLimitConfiguration configuration;
        configuration.m_twistLimitLower = -30.0f;
        configuration.m_twistLimitUpper = 30.0f;

        AZStd::vector<AZ::Vector3> vertexBuffer;
        AZStd::vector<AZ::u32> indexBuffer;
        AZStd::vector<AZ::Vector3> lineBuffer;
        AZStd::vector<bool> lineValidityBuffer;

        m_helpers.GenerateJointLimitVisualizationData(
            configuration, AZ::Quaternion::CreateIdentity(), Twist(10.0f), 1.0f, 16, 2, vertexBuffer, indexBuffer,
            lineBuffer, lineValidityBuffer);

        ASSERT_FALSE(lineValidityBuffer.empty());
        EXPECT_EQ(std::count(lineValidityBuffer.begin(), lineValidityBuffer.end(), false), 0);
    }

    TEST_F(JoltJointHelpersTests, AFixedJointDrawsNothingRatherThanFailing)
    {
        // It has no freedom to show. Leaving the buffers alone is the answer; a caller
        // draws whatever it finds in them.
        JoltFixedJointConfiguration configuration;

        AZStd::vector<AZ::Vector3> vertexBuffer;
        AZStd::vector<AZ::u32> indexBuffer;
        AZStd::vector<AZ::Vector3> lineBuffer;
        AZStd::vector<bool> lineValidityBuffer;

        m_helpers.GenerateJointLimitVisualizationData(
            configuration, AZ::Quaternion::CreateIdentity(), AZ::Quaternion::CreateIdentity(), 1.0f, 16, 2,
            vertexBuffer, indexBuffer, lineBuffer, lineValidityBuffer);

        EXPECT_TRUE(vertexBuffer.empty());
        EXPECT_TRUE(indexBuffer.empty());
        EXPECT_TRUE(lineBuffer.empty());
        EXPECT_TRUE(lineValidityBuffer.empty());
    }
} // namespace JoltPhysics
