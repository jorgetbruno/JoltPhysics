#include <Joint/JoltJointHelpers.h>

#include <AzCore/Math/MathUtils.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>

#include <Joint/JoltJointConfiguration.h>
#include <Joint/JoltJointLimitMath.h>

namespace JoltPhysics
{
    namespace
    {
        //! The joint frame as the child body holds it, given the axis the limit is
        //! centred on.
        //!
        //! The frame's X axis is the twist axis - Jolt's swing-twist and 6-DOF
        //! constraints both take X that way, and so do this gem's own joint components -
        //! so building the frame is a matter of rotating X onto the requested axis.
        //!
        //! The axis arrives in **world** space, not the child's: EMotionFX hands over a
        //! bone direction it has already resolved, so it has to be brought into the
        //! child's frame before the rotation is taken. Reading it as child-local instead
        //! puts the whole limit cone somewhere else for every bone whose body is rotated.
        AZ::Quaternion ComputeChildLocalRotation(const AZ::Quaternion& childWorldRotation, const AZ::Vector3& axis)
        {
            const AZ::Vector3 normalizedAxis =
                axis.GetLengthSq() > AZ::Constants::FloatEpsilon ? axis.GetNormalized() : AZ::Vector3::CreateAxisX();
            return AZ::Quaternion::CreateShortestArc(
                AZ::Vector3::CreateAxisX(), childWorldRotation.GetConjugate().TransformVector(normalizedAxis));
        }

        using JointLimitMath::RelativeRotationInJointFrame;

        //! Every sample brought from "child relative to parent" into the joint's own
        //! frame, which is the space the limits are written in.
        AZStd::vector<AZ::Quaternion> ToJointFrame(
            const AZStd::vector<AZ::Quaternion>& childRelativeToParentSamples,
            const AZ::Quaternion& parentLocalRotation,
            const AZ::Quaternion& childLocalRotation)
        {
            AZStd::vector<AZ::Quaternion> jointFrameSamples;
            jointFrameSamples.reserve(childRelativeToParentSamples.size());
            for (const AZ::Quaternion& sample : childRelativeToParentSamples)
            {
                jointFrameSamples.push_back(
                    RelativeRotationInJointFrame(parentLocalRotation, sample, childLocalRotation));
            }
            return jointFrameSamples;
        }
    } // namespace

    const AZStd::vector<AZ::TypeId> JoltJointHelpers::GetSupportedJointTypeIds() const
    {
        return {
            azrtti_typeid<JoltD6JointLimitConfiguration>(),
            azrtti_typeid<JoltFixedJointConfiguration>(),
            azrtti_typeid<JoltBallJointConfiguration>(),
            azrtti_typeid<JoltHingeJointConfiguration>(),
        };
    }

    AZStd::optional<const AZ::TypeId> JoltJointHelpers::GetSupportedJointTypeId(AzPhysics::JointType typeEnum) const
    {
        switch (typeEnum)
        {
        case AzPhysics::JointType::D6Joint:
            return azrtti_typeid<JoltD6JointLimitConfiguration>();
        case AzPhysics::JointType::FixedJoint:
            return azrtti_typeid<JoltFixedJointConfiguration>();
        case AzPhysics::JointType::BallJoint:
            return azrtti_typeid<JoltBallJointConfiguration>();
        case AzPhysics::JointType::HingeJoint:
            return azrtti_typeid<JoltHingeJointConfiguration>();
        default:
            // Prismatic, distance, cone, swing-twist, gear and rack-and-pinion all exist
            // in this gem, but AzPhysics::JointType has no name for them - so there is no
            // way for a caller to ask, and nothing useful to answer.
            return AZStd::nullopt;
        }
    }

    AZStd::unique_ptr<AzPhysics::JointConfiguration> JoltJointHelpers::ComputeInitialJointLimitConfiguration(
        const AZ::TypeId& jointLimitTypeId,
        const AZ::Quaternion& parentWorldRotation,
        const AZ::Quaternion& childWorldRotation,
        const AZ::Vector3& axis,
        const AZStd::vector<AZ::Quaternion>& exampleLocalRotations)
    {
        // The joint frame, written in each body's own space. Both bodies name one frame,
        // so at the pose this was computed from the joint sits at its rest position with
        // zero swing and zero twist.
        const AZ::Quaternion childLocalRotation = ComputeChildLocalRotation(childWorldRotation, axis);
        const AZ::Quaternion parentLocalRotation =
            parentWorldRotation.GetConjugate() * childWorldRotation * childLocalRotation;

        auto applyFrames = [&parentLocalRotation, &childLocalRotation](AzPhysics::JointConfiguration& configuration)
        {
            configuration.m_parentLocalRotation = parentLocalRotation;
            configuration.m_childLocalRotation = childLocalRotation;
        };

        if (jointLimitTypeId == azrtti_typeid<JoltD6JointLimitConfiguration>())
        {
            auto configuration = AZStd::make_unique<JoltD6JointLimitConfiguration>();
            applyFrames(*configuration);

            // The example poses are the child relative to the parent; bring each into the
            // joint frame before measuring, or the fit would describe rotations about the
            // parent's axes rather than the joint's.
            const JointLimitMath::SwingTwistLimits limits =
                JointLimitMath::FitLimits(ToJointFrame(exampleLocalRotations, parentLocalRotation, childLocalRotation));
            configuration->m_swingLimitY = limits.m_swingYDegrees;
            configuration->m_swingLimitZ = limits.m_swingZDegrees;
            configuration->m_twistLimitLower = limits.m_twistLowerDegrees;
            configuration->m_twistLimitUpper = limits.m_twistUpperDegrees;
            return configuration;
        }

        if (jointLimitTypeId == azrtti_typeid<JoltBallJointConfiguration>())
        {
            auto configuration = AZStd::make_unique<JoltBallJointConfiguration>();
            applyFrames(*configuration);

            // A ball joint limits swing and leaves twist free, so only the cone is fitted.
            const JointLimitMath::SwingTwistLimits limits =
                JointLimitMath::FitLimits(ToJointFrame(exampleLocalRotations, parentLocalRotation, childLocalRotation));
            configuration->m_limitProperties.m_isLimited = true;
            configuration->m_limitProperties.m_limitFirst = limits.m_swingYDegrees;
            configuration->m_limitProperties.m_limitSecond = limits.m_swingZDegrees;
            return configuration;
        }

        if (jointLimitTypeId == azrtti_typeid<JoltHingeJointConfiguration>())
        {
            auto configuration = AZStd::make_unique<JoltHingeJointConfiguration>();
            applyFrames(*configuration);

            // A hinge is twist alone - the swing the samples show is off-axis motion the
            // hinge will not permit, and is deliberately dropped rather than widened into.
            const JointLimitMath::SwingTwistLimits limits =
                JointLimitMath::FitLimits(ToJointFrame(exampleLocalRotations, parentLocalRotation, childLocalRotation));
            configuration->m_limitProperties.m_isLimited = true;
            configuration->m_limitProperties.m_limitFirst = limits.m_twistLowerDegrees;
            configuration->m_limitProperties.m_limitSecond = limits.m_twistUpperDegrees;
            return configuration;
        }

        if (jointLimitTypeId == azrtti_typeid<JoltFixedJointConfiguration>())
        {
            // Nothing to fit: a fixed joint has no freedom to bound. The frames still
            // matter, since they are where the weld holds the two bodies together.
            auto configuration = AZStd::make_unique<JoltFixedJointConfiguration>();
            applyFrames(*configuration);
            return configuration;
        }

        AZ_Warning("JoltPhysics", false,
            "Asked for an initial joint limit of type %s, which this backend does not author. "
            "GetSupportedJointTypeIds lists the types it does.",
            jointLimitTypeId.ToString<AZStd::string>().c_str());
        return nullptr;
    }

    void JoltJointHelpers::GenerateJointLimitVisualizationData(
        const AzPhysics::JointConfiguration& configuration,
        const AZ::Quaternion& parentRotation,
        const AZ::Quaternion& childRotation,
        float scale,
        AZ::u32 angularSubdivisions,
        AZ::u32 radialSubdivisions,
        [[maybe_unused]] AZStd::vector<AZ::Vector3>& vertexBufferOut,
        [[maybe_unused]] AZStd::vector<AZ::u32>& indexBufferOut,
        AZStd::vector<AZ::Vector3>& lineBufferOut,
        AZStd::vector<bool>& lineValidityBufferOut)
    {
        // Geometry goes out in the joint's frame *within the parent body* - the caller
        // applies the parent's world transform to every point itself
        // (CharacterPhysicsDebugDraw::RenderJointLimit). Folding parentRotation in here
        // as well would rotate the whole drawing twice.
        //
        // The two rotations are for one thing only: working out where the joint actually
        // is, which is what decides whether the limits are drawn as met or violated.
        const AZ::Quaternion& jointLocalRotation = configuration.m_parentLocalRotation;

        const AZ::Quaternion childRelativeToParent = parentRotation.GetConjugate() * childRotation;
        const JointLimitMath::SwingTwist current =
            JointLimitMath::DecomposeSwingTwist(JointLimitMath::RelativeRotationInJointFrame(
                configuration.m_parentLocalRotation, childRelativeToParent, configuration.m_childLocalRotation));

        // The vertex and index buffers are left alone deliberately: nothing renders them.
        // RenderJointLimit draws the line buffer and never touches the triangles, and the
        // PhysX implementation marks its own copies of them [[maybe_unused]] for the same
        // reason. A solid cone here would simply be invisible.

        if (const auto* d6 = azrtti_cast<const JoltD6JointLimitConfiguration*>(&configuration))
        {
            JointLimitMath::AppendSwingConeLines(
                d6->m_swingLimitY, d6->m_swingLimitZ, current.m_swingYDegrees, current.m_swingZDegrees,
                jointLocalRotation, scale, angularSubdivisions, radialSubdivisions, lineBufferOut,
                lineValidityBufferOut);
            JointLimitMath::AppendTwistArcLines(
                d6->m_twistLimitLower, d6->m_twistLimitUpper, current.m_twistDegrees, jointLocalRotation, scale,
                angularSubdivisions, lineBufferOut, lineValidityBufferOut);
            JointLimitMath::AppendCurrentTwistLine(
                current.m_twistDegrees, jointLocalRotation, scale, lineBufferOut, lineValidityBufferOut);
            return;
        }

        if (const auto* swingTwist = azrtti_cast<const JoltSwingTwistJointConfiguration*>(&configuration))
        {
            JointLimitMath::AppendSwingConeLines(
                swingTwist->m_normalHalfConeAngle, swingTwist->m_planeHalfConeAngle, current.m_swingYDegrees,
                current.m_swingZDegrees, jointLocalRotation, scale, angularSubdivisions, radialSubdivisions,
                lineBufferOut, lineValidityBufferOut);
            JointLimitMath::AppendTwistArcLines(
                swingTwist->m_twistLower, swingTwist->m_twistUpper, current.m_twistDegrees, jointLocalRotation, scale,
                angularSubdivisions, lineBufferOut, lineValidityBufferOut);
            JointLimitMath::AppendCurrentTwistLine(
                current.m_twistDegrees, jointLocalRotation, scale, lineBufferOut, lineValidityBufferOut);
            return;
        }

        if (const auto* ball = azrtti_cast<const JoltBallJointConfiguration*>(&configuration))
        {
            if (ball->m_limitProperties.m_isLimited)
            {
                JointLimitMath::AppendSwingConeLines(
                    ball->m_limitProperties.m_limitFirst, ball->m_limitProperties.m_limitSecond, current.m_swingYDegrees,
                    current.m_swingZDegrees, jointLocalRotation, scale, angularSubdivisions, radialSubdivisions,
                    lineBufferOut, lineValidityBufferOut);
            }
            return;
        }

        if (const auto* cone = azrtti_cast<const JoltConeJointConfiguration*>(&configuration))
        {
            JointLimitMath::AppendSwingConeLines(
                cone->m_halfConeAngle, cone->m_halfConeAngle, current.m_swingYDegrees, current.m_swingZDegrees,
                jointLocalRotation, scale, angularSubdivisions, radialSubdivisions, lineBufferOut,
                lineValidityBufferOut);
            return;
        }

        if (const auto* hinge = azrtti_cast<const JoltHingeJointConfiguration*>(&configuration))
        {
            if (hinge->m_limitProperties.m_isLimited)
            {
                JointLimitMath::AppendTwistArcLines(
                    hinge->m_limitProperties.m_limitFirst, hinge->m_limitProperties.m_limitSecond,
                    current.m_twistDegrees, jointLocalRotation, scale, angularSubdivisions, lineBufferOut,
                    lineValidityBufferOut);
                JointLimitMath::AppendCurrentTwistLine(
                    current.m_twistDegrees, jointLocalRotation, scale, lineBufferOut, lineValidityBufferOut);
            }
            return;
        }

        // A fixed joint, or a type with nothing to show. Leaving the buffers untouched is
        // the answer, not a failure - the caller draws whatever is in them.
    }
} // namespace JoltPhysics
