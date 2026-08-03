#pragma once

#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>

namespace JoltPhysics
{
    //! Swing-twist maths for joint limits, and the geometry that visualises them.
    //!
    //! Separate from the joint components on purpose: this is what
    //! AzPhysics::JointHelpersInterface is asked for by the Animation Editor's ragdoll
    //! tools, which never touch a joint component - they work on a RagdollNodeConfiguration
    //! long before anything is simulated. Keeping it free of Jolt and of components makes
    //! it testable as the pure geometry it is.
    namespace JointLimitMath
    {
        //! A relative rotation split about the joint frame's X axis.
        //!
        //! Twist is the part about X; swing is what is left, which tilts X away from
        //! itself. The two limit kinds a ragdoll joint carries map onto exactly this
        //! split: a twist range and a swing cone.
        struct SwingTwist
        {
            float m_twistDegrees = 0.0f;  //!< Rotation about the joint frame's X axis.
            //! Tilt of the X axis *towards* Y and towards Z - not rotations about those
            //! axes, which would name the other one. This is the reading the gem's
            //! viewport cone is already drawn with, so the two cannot disagree.
            float m_swingYDegrees = 0.0f;
            float m_swingZDegrees = 0.0f;
        };

        //! Decomposes a rotation into twist about X and the swing that remains.
        //!
        //! The swing angles are signed and measured as the tilt of the rotated X axis
        //! towards Y and towards Z, which is the parametrisation an elliptical swing
        //! cone is authored in (two half-angles, one per axis) - not an axis-angle,
        //! which would collapse the two into one number.
        SwingTwist DecomposeSwingTwist(const AZ::Quaternion& rotation);

        //! The tightest limits containing every sample, plus a margin in degrees.
        //!
        //! Swing limits come back symmetric because that is what a cone can express;
        //! the twist range does not have to be, and is not forced to be. With no
        //! samples the defaults are returned unchanged - an initial guess a human can
        //! then drag, rather than a degenerate zero-width limit that pins the bone.
        struct SwingTwistLimits
        {
            float m_swingYDegrees = 45.0f;
            float m_swingZDegrees = 45.0f;
            float m_twistLowerDegrees = -45.0f;
            float m_twistUpperDegrees = 45.0f;
        };

        SwingTwistLimits FitLimits(
            const AZStd::vector<AZ::Quaternion>& samples,
            float marginDegrees = 5.0f,
            float minimumExtentDegrees = 1.0f,
            float maximumExtentDegrees = 179.0f);

        //! The swing cone as a triangle mesh: an apex at the joint origin and a fan out
        //! to an elliptical rim, so the cone reads as a solid volume rather than an
        //! outline. Appends to the buffers rather than clearing them, so a caller can
        //! build one mesh out of several limits.
        void AppendSwingConeMesh(
            float swingYDegrees,
            float swingZDegrees,
            const AZ::Quaternion& jointRotation,
            float scale,
            AZ::u32 angularSubdivisions,
            AZ::u32 radialSubdivisions,
            AZStd::vector<AZ::Vector3>& vertexBuffer,
            AZStd::vector<AZ::u32>& indexBuffer);

        //! The twist range as a line arc about the joint frame's X axis, swept from the
        //! lower limit to the upper. Each segment is flagged valid or violated: the
        //! caller draws the violated part in the error colour, which is how an
        //! over-rotated bone shows up in the Animation Editor.
        void AppendTwistArcLines(
            float twistLowerDegrees,
            float twistUpperDegrees,
            float currentTwistDegrees,
            const AZ::Quaternion& jointRotation,
            float scale,
            AZ::u32 angularSubdivisions,
            AZStd::vector<AZ::Vector3>& lineBuffer,
            AZStd::vector<bool>& lineValidityBuffer);
    } // namespace JointLimitMath
} // namespace JoltPhysics
