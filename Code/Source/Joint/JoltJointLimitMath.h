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

        //! The child's rotation relative to the parent, expressed in the joint frame.
        //!
        //! Identity in the pose the joint was authored from, so a decomposition of it
        //! reads directly as "how far has this bone moved from rest".
        AZ::Quaternion RelativeRotationInJointFrame(
            const AZ::Quaternion& parentLocalRotation,
            const AZ::Quaternion& childRelativeToParent,
            const AZ::Quaternion& childLocalRotation);

        //! Whether a swing fits inside an elliptical cone, as the ellipse equation rather
        //! than two independent comparisons - a swing can be inside both half-angles taken
        //! one at a time and still outside the cone they describe together.
        bool IsSwingWithinLimits(float swingYDegrees, float swingZDegrees, float limitYDegrees, float limitZDegrees);

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

        //! @name Joint limit visualization
        //!
        //! Everything below appends *lines* - point pairs, one validity flag per pair -
        //! and nothing writes the triangle buffers the interface also carries, because
        //! nothing renders them: AzFramework's CharacterPhysicsDebugDraw::RenderJointLimit
        //! draws the line buffer and leaves the vertex and index buffers alone, and PhysX
        //! marks its own copies of them [[maybe_unused]]. A solid cone would simply be
        //! invisible.
        //!
        //! Points are in the *joint's frame within the parent body* - the caller applies
        //! the parent's world transform itself. Pre-applying the parent's rotation here
        //! would apply it twice.
        //!
        //! Validity is decided by where the joint currently is, not by which part of the
        //! drawing a segment belongs to: the whole cone, or the whole arc, turns the error
        //! colour when the joint is outside it. That is the contract the renderer and the
        //! PhysX implementation share.
        //! @{

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
            AZStd::vector<bool>& lineValidityBuffer);

        void AppendTwistArcLines(
            float twistLowerDegrees,
            float twistUpperDegrees,
            float currentTwistDegrees,
            const AZ::Quaternion& jointLocalRotation,
            float scale,
            AZ::u32 angularSubdivisions,
            AZStd::vector<AZ::Vector3>& lineBuffer,
            AZStd::vector<bool>& lineValidityBuffer);

        //! A single spoke showing where the twist actually is inside (or outside) the arc.
        //! Always flagged valid: it is a readout, not a limit, and colouring it as a
        //! violation would say the marker itself was wrong.
        void AppendCurrentTwistLine(
            float currentTwistDegrees,
            const AZ::Quaternion& jointLocalRotation,
            float scale,
            AZStd::vector<AZ::Vector3>& lineBuffer,
            AZStd::vector<bool>& lineValidityBuffer);

        //! @}
    } // namespace JointLimitMath
} // namespace JoltPhysics
