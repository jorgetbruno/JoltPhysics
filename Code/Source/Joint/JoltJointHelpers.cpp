#include <Joint/JoltJointHelpers.h>

#include <AzCore/Math/MathUtils.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>

#include <Joint/JoltJointConfiguration.h>
#include <Joint/JoltJointLimitMath.h>

namespace JoltPhysics
{
    namespace
    {
        //! The joint frame in world space, given the child's pose and the axis the limit
        //! is centred on.
        //!
        //! The frame's X axis is the twist axis - Jolt's swing-twist and 6-DOF
        //! constraints both take X that way, and so do this gem's own joint components -
        //! so building the frame is a matter of rotating X onto the requested axis. The
        //! axis arrives in the child body's space, which is where a ragdoll's bone
        //! direction is expressed.
        AZ::Quaternion ComputeJointWorldRotation(const AZ::Quaternion& childWorldRotation, const AZ::Vector3& axis)
        {
            const AZ::Vector3 normalizedAxis =
                axis.GetLengthSq() > AZ::Constants::FloatEpsilon ? axis.GetNormalized() : AZ::Vector3::CreateAxisX();
            const AZ::Quaternion axisRotation =
                AZ::Quaternion::CreateShortestArc(AZ::Vector3::CreateAxisX(), normalizedAxis);
            return childWorldRotation * axisRotation;
        }

        //! The child's rotation relative to the parent, expressed in the joint frame.
        //!
        //! This is the quantity the limits are written against: it is identity in the
        //! pose the joint was authored from, so a decomposition of it reads directly as
        //! "how far has this bone moved from rest".
        AZ::Quaternion RelativeRotationInJointFrame(
            const AZ::Quaternion& parentLocalRotation,
            const AZ::Quaternion& childRelativeToParent,
            const AZ::Quaternion& childLocalRotation)
        {
            return parentLocalRotation.GetConjugate() * childRelativeToParent * childLocalRotation;
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
        // The joint frame, and the same frame written in each body's own space. Both
        // bodies name one frame, so at the pose this was computed from the joint sits at
        // its rest position with zero swing and zero twist.
        const AZ::Quaternion jointWorldRotation = ComputeJointWorldRotation(childWorldRotation, axis);
        const AZ::Quaternion parentLocalRotation = parentWorldRotation.GetConjugate() * jointWorldRotation;
        const AZ::Quaternion childLocalRotation = childWorldRotation.GetConjugate() * jointWorldRotation;

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
            AZStd::vector<AZ::Quaternion> jointFrameSamples;
            jointFrameSamples.reserve(exampleLocalRotations.size());
            for (const AZ::Quaternion& example : exampleLocalRotations)
            {
                jointFrameSamples.push_back(
                    RelativeRotationInJointFrame(parentLocalRotation, example, childLocalRotation));
            }

            const JointLimitMath::SwingTwistLimits limits = JointLimitMath::FitLimits(jointFrameSamples);
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

            AZStd::vector<AZ::Quaternion> jointFrameSamples;
            jointFrameSamples.reserve(exampleLocalRotations.size());
            for (const AZ::Quaternion& example : exampleLocalRotations)
            {
                jointFrameSamples.push_back(
                    RelativeRotationInJointFrame(parentLocalRotation, example, childLocalRotation));
            }

            // A ball joint limits swing and leaves twist free, so only the cone is fitted.
            const JointLimitMath::SwingTwistLimits limits = JointLimitMath::FitLimits(jointFrameSamples);
            configuration->m_limitProperties.m_isLimited = true;
            configuration->m_limitProperties.m_limitFirst = limits.m_swingYDegrees;
            configuration->m_limitProperties.m_limitSecond = limits.m_swingZDegrees;
            return configuration;
        }

        if (jointLimitTypeId == azrtti_typeid<JoltHingeJointConfiguration>())
        {
            auto configuration = AZStd::make_unique<JoltHingeJointConfiguration>();
            applyFrames(*configuration);

            AZStd::vector<AZ::Quaternion> jointFrameSamples;
            jointFrameSamples.reserve(exampleLocalRotations.size());
            for (const AZ::Quaternion& example : exampleLocalRotations)
            {
                jointFrameSamples.push_back(
                    RelativeRotationInJointFrame(parentLocalRotation, example, childLocalRotation));
            }

            // A hinge is twist alone - the swing the samples show is off-axis motion the
            // hinge will not permit, and is deliberately dropped rather than widened into.
            const JointLimitMath::SwingTwistLimits limits = JointLimitMath::FitLimits(jointFrameSamples);
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
        AZStd::vector<AZ::Vector3>& vertexBufferOut,
        AZStd::vector<AZ::u32>& indexBufferOut,
        AZStd::vector<AZ::Vector3>& lineBufferOut,
        AZStd::vector<bool>& lineValidityBufferOut)
    {
        // Everything is drawn in the joint frame as the parent holds it: that frame is
        // where the limits are defined, and drawing there means the cone stays put while
        // the child swings inside it.
        const AZ::Quaternion jointRotation = parentRotation * configuration.m_parentLocalRotation;

        // Where the child currently is, in that same frame - what decides which part of
        // the twist arc is drawn as violated.
        const AZ::Quaternion childRelativeToParent = parentRotation.GetConjugate() * childRotation;
        const AZ::Quaternion relativeInJointFrame = RelativeRotationInJointFrame(
            configuration.m_parentLocalRotation, childRelativeToParent, configuration.m_childLocalRotation);
        const JointLimitMath::SwingTwist current = JointLimitMath::DecomposeSwingTwist(relativeInJointFrame);

        if (const auto* d6 = azrtti_cast<const JoltD6JointLimitConfiguration*>(&configuration))
        {
            JointLimitMath::AppendSwingConeMesh(
                d6->m_swingLimitY, d6->m_swingLimitZ, jointRotation, scale, angularSubdivisions, radialSubdivisions,
                vertexBufferOut, indexBufferOut);
            JointLimitMath::AppendTwistArcLines(
                d6->m_twistLimitLower, d6->m_twistLimitUpper, current.m_twistDegrees, jointRotation, scale,
                angularSubdivisions, lineBufferOut, lineValidityBufferOut);
            return;
        }

        if (const auto* swingTwist = azrtti_cast<const JoltSwingTwistJointConfiguration*>(&configuration))
        {
            JointLimitMath::AppendSwingConeMesh(
                swingTwist->m_normalHalfConeAngle, swingTwist->m_planeHalfConeAngle, jointRotation, scale,
                angularSubdivisions, radialSubdivisions, vertexBufferOut, indexBufferOut);
            JointLimitMath::AppendTwistArcLines(
                swingTwist->m_twistLower, swingTwist->m_twistUpper, current.m_twistDegrees, jointRotation, scale,
                angularSubdivisions, lineBufferOut, lineValidityBufferOut);
            return;
        }

        if (const auto* ball = azrtti_cast<const JoltBallJointConfiguration*>(&configuration))
        {
            if (ball->m_limitProperties.m_isLimited)
            {
                JointLimitMath::AppendSwingConeMesh(
                    ball->m_limitProperties.m_limitFirst, ball->m_limitProperties.m_limitSecond, jointRotation, scale,
                    angularSubdivisions, radialSubdivisions, vertexBufferOut, indexBufferOut);
            }
            return;
        }

        if (const auto* cone = azrtti_cast<const JoltConeJointConfiguration*>(&configuration))
        {
            JointLimitMath::AppendSwingConeMesh(
                cone->m_halfConeAngle, cone->m_halfConeAngle, jointRotation, scale, angularSubdivisions,
                radialSubdivisions, vertexBufferOut, indexBufferOut);
            return;
        }

        if (const auto* hinge = azrtti_cast<const JoltHingeJointConfiguration*>(&configuration))
        {
            if (hinge->m_limitProperties.m_isLimited)
            {
                JointLimitMath::AppendTwistArcLines(
                    hinge->m_limitProperties.m_limitFirst, hinge->m_limitProperties.m_limitSecond,
                    current.m_twistDegrees, jointRotation, scale, angularSubdivisions, lineBufferOut,
                    lineValidityBufferOut);
            }
            return;
        }

        // A fixed joint, or a type with nothing to show. Leaving the buffers untouched is
        // the answer, not a failure - the caller draws whatever is in them.
    }
} // namespace JoltPhysics
