#include <Joint/JoltJointLimitMath.h>

#include <AzCore/Casting/numeric_cast.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Math/Transform.h>

namespace JoltPhysics
{
    namespace JointLimitMath
    {
        namespace
        {
            //! Below this the swing quaternion carries no direction to speak of and the
            //! angles are numerically meaningless; the rotation is a pure twist.
            constexpr float SwingEpsilon = 1e-6f;

            //! A cone stops being a cone at 90 degrees, and the tangent it is built from
            //! runs away long before that. Clamped where the drawing stays readable.
            constexpr float MaximumDrawnHalfAngleDegrees = 89.0f;

            //! The same bounds PhysX clamps to, so a caller that tuned its subdivision
            //! counts against one backend gets the same detail from the other.
            constexpr AZ::u32 MinimumAngularSubdivisions = 4;
            constexpr AZ::u32 MaximumAngularSubdivisions = 32;
            constexpr AZ::u32 MinimumRadialSubdivisions = 1;
            constexpr AZ::u32 MaximumRadialSubdivisions = 4;

            //! A point on the cone at the given angle round its rim, at a fraction of the
            //! full half-angles. Normalised so every rim point is the same distance out,
            //! which is what makes the drawing read as a cone rather than a pyramid.
            AZ::Vector3 ConePoint(
                float tanY, float tanZ, float angleRadians, float fraction, const AZ::Quaternion& jointLocalRotation,
                float scale)
            {
                const AZ::Vector3 direction =
                    AZ::Vector3(1.0f, fraction * tanY * cosf(angleRadians), fraction * tanZ * sinf(angleRadians))
                        .GetNormalized();
                return jointLocalRotation.TransformVector(direction) * scale;
            }

            AZ::Vector3 TwistPoint(float degrees, const AZ::Quaternion& jointLocalRotation, float scale)
            {
                const float radians = AZ::DegToRad(degrees);
                return jointLocalRotation.TransformVector(AZ::Vector3(0.0f, cosf(radians), sinf(radians))) * scale;
            }

            void AppendLine(
                const AZ::Vector3& from,
                const AZ::Vector3& to,
                bool valid,
                AZStd::vector<AZ::Vector3>& lineBuffer,
                AZStd::vector<bool>& lineValidityBuffer)
            {
                lineBuffer.push_back(from);
                lineBuffer.push_back(to);
                lineValidityBuffer.push_back(valid);
            }
        } // namespace

        SwingTwist DecomposeSwingTwist(const AZ::Quaternion& rotation)
        {
            const AZ::Quaternion normalized = rotation.GetNormalized();

            // Twist is the projection onto the X axis. The w and x components alone
            // describe a rotation about X; renormalising them gives the twist, and what
            // remains once it is divided out is the swing.
            const float twistLengthSquared = normalized.GetW() * normalized.GetW() + normalized.GetX() * normalized.GetX();

            AZ::Quaternion twist = AZ::Quaternion::CreateIdentity();
            if (twistLengthSquared > SwingEpsilon)
            {
                const float inverseLength = 1.0f / sqrtf(twistLengthSquared);
                twist = AZ::Quaternion(normalized.GetX() * inverseLength, 0.0f, 0.0f, normalized.GetW() * inverseLength);
            }
            // Otherwise a half turn away from the X axis: the twist is unrecoverable (any
            // twist followed by that swing gives the same rotation), so it stays identity
            // and the swing carries all of it.

            const AZ::Quaternion swing = normalized * twist.GetConjugate();

            SwingTwist result;

            // Signed, and taking the shorter way round: a twist of -170 degrees should
            // read as -170 rather than +190, or a fitted range would wrap the wrong way.
            result.m_twistDegrees = AZ::RadToDeg(2.0f * atan2f(twist.GetX(), twist.GetW()));
            if (result.m_twistDegrees > 180.0f)
            {
                result.m_twistDegrees -= 360.0f;
            }
            else if (result.m_twistDegrees < -180.0f)
            {
                result.m_twistDegrees += 360.0f;
            }

            // Where the swing put the X axis. Measuring the tilt off that axis - rather
            // than reading the quaternion's components - keeps the two angles independent,
            // which is what an elliptical cone needs.
            //
            // "Swing Y" is how far X leaned *towards* Y, not a rotation about Y. The two
            // readings differ by which axis they name, and the gem's viewport cone already
            // draws its Y half-angle as a displacement towards Y - so this matches that,
            // and the drawn cone and the measured angle cannot disagree.
            const AZ::Vector3 swungAxis = swing.TransformVector(AZ::Vector3::CreateAxisX());
            result.m_swingYDegrees = AZ::RadToDeg(atan2f(swungAxis.GetY(), swungAxis.GetX()));
            result.m_swingZDegrees = AZ::RadToDeg(atan2f(swungAxis.GetZ(), swungAxis.GetX()));

            return result;
        }

        AZ::Quaternion RelativeRotationInJointFrame(
            const AZ::Quaternion& parentLocalRotation,
            const AZ::Quaternion& childRelativeToParent,
            const AZ::Quaternion& childLocalRotation)
        {
            return parentLocalRotation.GetConjugate() * childRelativeToParent * childLocalRotation;
        }

        bool IsSwingWithinLimits(float swingYDegrees, float swingZDegrees, float limitYDegrees, float limitZDegrees)
        {
            // Guard the division rather than the caller: a limit of zero is a cone with no
            // opening, which only a swing of zero fits.
            const float safeLimitY = AZ::GetMax(fabsf(limitYDegrees), SwingEpsilon);
            const float safeLimitZ = AZ::GetMax(fabsf(limitZDegrees), SwingEpsilon);
            const float yFactor = swingYDegrees / safeLimitY;
            const float zFactor = swingZDegrees / safeLimitZ;

            // The ellipse, not two independent comparisons: a swing can be inside both
            // half-angles taken one at a time and still outside the cone they describe.
            constexpr float Epsilon = 1e-4f;
            return yFactor * yFactor + zFactor * zFactor <= 1.0f + Epsilon;
        }

        SwingTwistLimits FitLimits(
            const AZStd::vector<AZ::Quaternion>& samples,
            float marginDegrees,
            float minimumExtentDegrees,
            float maximumExtentDegrees)
        {
            SwingTwistLimits limits;
            if (samples.empty())
            {
                return limits;
            }

            float maximumSwingY = 0.0f;
            float maximumSwingZ = 0.0f;
            float twistLower = AZStd::numeric_limits<float>::max();
            float twistUpper = AZStd::numeric_limits<float>::lowest();

            for (const AZ::Quaternion& sample : samples)
            {
                const SwingTwist decomposed = DecomposeSwingTwist(sample);
                maximumSwingY = AZ::GetMax(maximumSwingY, fabsf(decomposed.m_swingYDegrees));
                maximumSwingZ = AZ::GetMax(maximumSwingZ, fabsf(decomposed.m_swingZDegrees));
                twistLower = AZ::GetMin(twistLower, decomposed.m_twistDegrees);
                twistUpper = AZ::GetMax(twistUpper, decomposed.m_twistDegrees);
            }

            limits.m_swingYDegrees =
                AZ::GetClamp(maximumSwingY + marginDegrees, minimumExtentDegrees, maximumExtentDegrees);
            limits.m_swingZDegrees =
                AZ::GetClamp(maximumSwingZ + marginDegrees, minimumExtentDegrees, maximumExtentDegrees);
            limits.m_twistLowerDegrees =
                AZ::GetClamp(twistLower - marginDegrees, -maximumExtentDegrees, -minimumExtentDegrees);
            limits.m_twistUpperDegrees =
                AZ::GetClamp(twistUpper + marginDegrees, minimumExtentDegrees, maximumExtentDegrees);

            return limits;
        }

        void AppendSwingConeLines(
            float swingLimitYDegrees,
            float swingLimitZDegrees,
            float currentSwingYDegrees,
            float currentSwingZDegrees,
            const AZ::Quaternion& jointLocalRotation,
            float scale,
            AZ::u32 angularSubdivisions,
            AZ::u32 radialSubdivisions,
            AZStd::vector<AZ::Vector3>& lineBuffer,
            AZStd::vector<bool>& lineValidityBuffer)
        {
            const AZ::u32 angularSegments =
                AZ::GetClamp(angularSubdivisions, MinimumAngularSubdivisions, MaximumAngularSubdivisions);
            const AZ::u32 radialSegments =
                AZ::GetClamp(radialSubdivisions, MinimumRadialSubdivisions, MaximumRadialSubdivisions);

            const float tanY = tanf(AZ::DegToRad(AZ::GetClamp(swingLimitYDegrees, 0.0f, MaximumDrawnHalfAngleDegrees)));
            const float tanZ = tanf(AZ::DegToRad(AZ::GetClamp(swingLimitZDegrees, 0.0f, MaximumDrawnHalfAngleDegrees)));

            // One verdict for the whole cone: the renderer colours a violated limit by
            // line, so a cone drawn half in the error colour would read as half the limit
            // being broken rather than the joint being outside it.
            const bool swingValid =
                IsSwingWithinLimits(currentSwingYDegrees, currentSwingZDegrees, swingLimitYDegrees, swingLimitZDegrees);

            for (AZ::u32 ring = 1; ring <= radialSegments; ++ring)
            {
                const float fraction = aznumeric_cast<float>(ring) / aznumeric_cast<float>(radialSegments);
                AZ::Vector3 previous = ConePoint(tanY, tanZ, 0.0f, fraction, jointLocalRotation, scale);
                const AZ::Vector3 first = previous;

                for (AZ::u32 segment = 1; segment <= angularSegments; ++segment)
                {
                    const float angle = AZ::Constants::TwoPi * aznumeric_cast<float>(segment) /
                        aznumeric_cast<float>(angularSegments);
                    const AZ::Vector3 current = (segment == angularSegments)
                        ? first // close the ring exactly rather than within rounding
                        : ConePoint(tanY, tanZ, angle, fraction, jointLocalRotation, scale);
                    AppendLine(previous, current, swingValid, lineBuffer, lineValidityBuffer);
                    previous = current;
                }
            }

            // Spokes out to the rim, so the cone reads as a volume from the joint origin
            // rather than as a floating ellipse.
            constexpr AZ::u32 SpokeCount = 4;
            for (AZ::u32 spoke = 0; spoke < SpokeCount; ++spoke)
            {
                const float angle = AZ::Constants::TwoPi * aznumeric_cast<float>(spoke) / aznumeric_cast<float>(SpokeCount);
                AppendLine(
                    AZ::Vector3::CreateZero(), ConePoint(tanY, tanZ, angle, 1.0f, jointLocalRotation, scale), swingValid,
                    lineBuffer, lineValidityBuffer);
            }
        }

        void AppendTwistArcLines(
            float twistLowerDegrees,
            float twistUpperDegrees,
            float currentTwistDegrees,
            const AZ::Quaternion& jointLocalRotation,
            float scale,
            AZ::u32 angularSubdivisions,
            AZStd::vector<AZ::Vector3>& lineBuffer,
            AZStd::vector<bool>& lineValidityBuffer)
        {
            const AZ::u32 arcSegments =
                AZ::GetClamp(angularSubdivisions, MinimumAngularSubdivisions, MaximumAngularSubdivisions);

            // The arc is the limit, drawn where the limit is. Where the joint actually sits
            // is a separate line (AppendCurrentTwistLine), so an overrun shows as the
            // marker standing outside an arc the renderer has turned the error colour.
            const bool twistValid = currentTwistDegrees >= twistLowerDegrees && currentTwistDegrees <= twistUpperDegrees;

            AZ::Vector3 previous = TwistPoint(twistLowerDegrees, jointLocalRotation, scale);

            // The end spokes, so the range reads as a wedge rather than a floating arc.
            AppendLine(AZ::Vector3::CreateZero(), previous, twistValid, lineBuffer, lineValidityBuffer);

            for (AZ::u32 segment = 1; segment <= arcSegments; ++segment)
            {
                const float fraction = aznumeric_cast<float>(segment) / aznumeric_cast<float>(arcSegments);
                const float degrees = twistLowerDegrees + (twistUpperDegrees - twistLowerDegrees) * fraction;
                const AZ::Vector3 current = TwistPoint(degrees, jointLocalRotation, scale);
                AppendLine(previous, current, twistValid, lineBuffer, lineValidityBuffer);
                previous = current;
            }

            AppendLine(AZ::Vector3::CreateZero(), previous, twistValid, lineBuffer, lineValidityBuffer);
        }

        void AppendCurrentTwistLine(
            float currentTwistDegrees,
            const AZ::Quaternion& jointLocalRotation,
            float scale,
            AZStd::vector<AZ::Vector3>& lineBuffer,
            AZStd::vector<bool>& lineValidityBuffer)
        {
            // Drawn slightly longer than the arc so it stays readable where it crosses it.
            constexpr float MarkerScale = 1.25f;
            AppendLine(
                AZ::Vector3::CreateZero(), TwistPoint(currentTwistDegrees, jointLocalRotation, scale * MarkerScale),
                /*valid*/ true, lineBuffer, lineValidityBuffer);
        }
    } // namespace JointLimitMath
} // namespace JoltPhysics
