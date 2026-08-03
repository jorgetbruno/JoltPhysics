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

            AZ::u32 ClampSubdivisions(AZ::u32 subdivisions, AZ::u32 minimum, AZ::u32 maximum)
            {
                return AZ::GetClamp(subdivisions, minimum, maximum);
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
            else
            {
                // A half turn away from the X axis: the twist is unrecoverable (any twist
                // followed by that swing gives the same rotation), so call it zero and
                // let the swing carry all of it.
                twist = AZ::Quaternion::CreateIdentity();
            }

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

        void AppendSwingConeMesh(
            float swingYDegrees,
            float swingZDegrees,
            const AZ::Quaternion& jointRotation,
            float scale,
            AZ::u32 angularSubdivisions,
            AZ::u32 radialSubdivisions,
            AZStd::vector<AZ::Vector3>& vertexBuffer,
            AZStd::vector<AZ::u32>& indexBuffer)
        {
            const AZ::u32 angularSegments = ClampSubdivisions(angularSubdivisions, 4, 64);
            const AZ::u32 radialSegments = ClampSubdivisions(radialSubdivisions, 1, 16);

            const float tanY = tanf(AZ::DegToRad(AZ::GetClamp(swingYDegrees, 0.0f, MaximumDrawnHalfAngleDegrees)));
            const float tanZ = tanf(AZ::DegToRad(AZ::GetClamp(swingZDegrees, 0.0f, MaximumDrawnHalfAngleDegrees)));

            const AZ::u32 firstVertex = aznumeric_cast<AZ::u32>(vertexBuffer.size());

            // The apex, then rings out to the rim. Rings rather than a single fan so the
            // surface still shades as a curved sheet when a renderer lights it.
            vertexBuffer.push_back(AZ::Vector3::CreateZero());

            for (AZ::u32 ring = 1; ring <= radialSegments; ++ring)
            {
                const float ringFraction = aznumeric_cast<float>(ring) / aznumeric_cast<float>(radialSegments);
                for (AZ::u32 segment = 0; segment < angularSegments; ++segment)
                {
                    const float angle =
                        AZ::Constants::TwoPi * aznumeric_cast<float>(segment) / aznumeric_cast<float>(angularSegments);
                    const AZ::Vector3 direction =
                        AZ::Vector3(1.0f, ringFraction * tanY * cosf(angle), ringFraction * tanZ * sinf(angle))
                            .GetNormalized();
                    vertexBuffer.push_back(jointRotation.TransformVector(direction) * scale * ringFraction);
                }
            }

            // Apex fan onto the first ring.
            for (AZ::u32 segment = 0; segment < angularSegments; ++segment)
            {
                const AZ::u32 next = (segment + 1) % angularSegments;
                indexBuffer.push_back(firstVertex);
                indexBuffer.push_back(firstVertex + 1 + segment);
                indexBuffer.push_back(firstVertex + 1 + next);
            }

            // Quads between successive rings, as pairs of triangles.
            for (AZ::u32 ring = 1; ring < radialSegments; ++ring)
            {
                const AZ::u32 innerRingStart = firstVertex + 1 + (ring - 1) * angularSegments;
                const AZ::u32 outerRingStart = firstVertex + 1 + ring * angularSegments;
                for (AZ::u32 segment = 0; segment < angularSegments; ++segment)
                {
                    const AZ::u32 next = (segment + 1) % angularSegments;
                    indexBuffer.push_back(innerRingStart + segment);
                    indexBuffer.push_back(outerRingStart + segment);
                    indexBuffer.push_back(outerRingStart + next);

                    indexBuffer.push_back(innerRingStart + segment);
                    indexBuffer.push_back(outerRingStart + next);
                    indexBuffer.push_back(innerRingStart + next);
                }
            }
        }

        void AppendTwistArcLines(
            float twistLowerDegrees,
            float twistUpperDegrees,
            float currentTwistDegrees,
            const AZ::Quaternion& jointRotation,
            float scale,
            AZ::u32 angularSubdivisions,
            AZStd::vector<AZ::Vector3>& lineBuffer,
            AZStd::vector<bool>& lineValidityBuffer)
        {
            const AZ::u32 arcSegments = ClampSubdivisions(angularSubdivisions, 4, 64);

            // Sweep the whole span the joint could be in - the limits, widened to include
            // where it actually is. A bone dragged past its limit then draws an arc that
            // runs beyond the allowed part, with that overrun marked invalid, which is
            // the whole point of reporting validity per segment.
            const float sweepFrom = AZ::GetMin(twistLowerDegrees, currentTwistDegrees);
            const float sweepTo = AZ::GetMax(twistUpperDegrees, currentTwistDegrees);

            auto pointAt = [&jointRotation, scale](float degrees)
            {
                const float radians = AZ::DegToRad(degrees);
                return jointRotation.TransformVector(AZ::Vector3(0.0f, cosf(radians), sinf(radians))) * scale;
            };

            AZ::Vector3 previous = pointAt(sweepFrom);
            for (AZ::u32 segment = 1; segment <= arcSegments; ++segment)
            {
                const float fraction = aznumeric_cast<float>(segment) / aznumeric_cast<float>(arcSegments);
                const float degrees = sweepFrom + (sweepTo - sweepFrom) * fraction;
                const AZ::Vector3 current = pointAt(degrees);

                lineBuffer.push_back(previous);
                lineBuffer.push_back(current);

                // A segment counts as allowed when its midpoint is inside the limits, so
                // one segment straddling a limit is not silently reported as fully valid.
                const float midpoint = degrees - (sweepTo - sweepFrom) / (2.0f * aznumeric_cast<float>(arcSegments));
                lineValidityBuffer.push_back(midpoint >= twistLowerDegrees && midpoint <= twistUpperDegrees);

                previous = current;
            }

            // The two ends, as spokes back to the origin, so the range reads as a wedge
            // rather than a floating arc.
            lineBuffer.push_back(AZ::Vector3::CreateZero());
            lineBuffer.push_back(pointAt(twistLowerDegrees));
            lineValidityBuffer.push_back(true);

            lineBuffer.push_back(AZ::Vector3::CreateZero());
            lineBuffer.push_back(pointAt(twistUpperDegrees));
            lineValidityBuffer.push_back(true);
        }
    } // namespace JointLimitMath
} // namespace JoltPhysics
