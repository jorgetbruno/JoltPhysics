#include <Joint/JoltJoint.h>

#include <Joint/JoltJointConfiguration.h>
#include <Scene/JoltScene.h>
#include <Utils/Conversions.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/GearConstraint.h>
#include <Jolt/Physics/Constraints/RackAndPinionConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/SpringSettings.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace JoltPhysics
{
    JoltJoint::JoltJoint(
        JoltScene* scene,
        AzPhysics::SimulatedBodyHandle parentBody,
        AzPhysics::SimulatedBodyHandle childBody,
        JPH::Constraint* constraint)
        : m_scene(scene)
        , m_parentBody(parentBody)
        , m_childBody(childBody)
        , m_constraint(constraint)
    {
    }

    void JoltJoint::RemoveFromJoltWorld()
    {
        if (m_constraint && m_scene && m_scene->GetJoltPhysicsSystem())
        {
            m_scene->GetJoltPhysicsSystem()->RemoveConstraint(m_constraint);
        }
        m_constraint = nullptr;
    }

    AZ::Crc32 JoltJoint::GetNativeType() const
    {
        return AZ_CRC_CE("JoltJoint");
    }

    void* JoltJoint::GetNativePointer() const
    {
        return m_constraint;
    }

    AzPhysics::SimulatedBodyHandle JoltJoint::GetParentBodyHandle() const
    {
        return m_parentBody;
    }

    AzPhysics::SimulatedBodyHandle JoltJoint::GetChildBodyHandle() const
    {
        return m_childBody;
    }

    void JoltJoint::SetParentBody(AzPhysics::SimulatedBodyHandle parentBody)
    {
        // Rebinding an existing constraint is not supported; the joint must be recreated.
        m_parentBody = parentBody;
    }

    void JoltJoint::SetChildBody(AzPhysics::SimulatedBodyHandle childBody)
    {
        m_childBody = childBody;
    }

    void JoltJoint::ConfigureBreakable(bool breakable, float forceMax, float torqueMax)
    {
        m_breakable = breakable;
        m_breakForceMax = forceMax;
        m_breakTorqueMax = torqueMax;
    }

    bool JoltJoint::ExceedsBreakThreshold(float stepDeltaTime) const
    {
        if (!m_breakable || !m_constraint || stepDeltaTime <= 0.0f)
        {
            return false;
        }

        const auto square = [](float value)
        {
            return value * value;
        };

        // Accumulated solver impulses of the last step, split into the translational
        // part (N s) and the rotational part (N m s). Limit impulses count - a body
        // slamming into a limit is exactly the load that should break a joint.
        float forceImpulseSq = 0.0f;
        float torqueImpulseSq = 0.0f;
        switch (m_constraint->GetSubType())
        {
        case JPH::EConstraintSubType::Fixed:
        {
            const auto* fixed = static_cast<const JPH::FixedConstraint*>(m_constraint);
            forceImpulseSq = fixed->GetTotalLambdaPosition().LengthSq();
            torqueImpulseSq = fixed->GetTotalLambdaRotation().LengthSq();
            break;
        }
        case JPH::EConstraintSubType::Hinge:
        {
            const auto* hinge = static_cast<const JPH::HingeConstraint*>(m_constraint);
            forceImpulseSq = hinge->GetTotalLambdaPosition().LengthSq();
            const JPH::Vector<2> rotation = hinge->GetTotalLambdaRotation();
            torqueImpulseSq =
                square(rotation[0]) + square(rotation[1]) + square(hinge->GetTotalLambdaRotationLimits());
            break;
        }
        case JPH::EConstraintSubType::Slider:
        {
            const auto* slider = static_cast<const JPH::SliderConstraint*>(m_constraint);
            const JPH::Vector<2> position = slider->GetTotalLambdaPosition();
            forceImpulseSq =
                square(position[0]) + square(position[1]) + square(slider->GetTotalLambdaPositionLimits());
            torqueImpulseSq = slider->GetTotalLambdaRotation().LengthSq();
            break;
        }
        case JPH::EConstraintSubType::Distance:
        {
            const auto* distance = static_cast<const JPH::DistanceConstraint*>(m_constraint);
            forceImpulseSq = square(distance->GetTotalLambdaPosition());
            break;
        }
        case JPH::EConstraintSubType::Cone:
        {
            const auto* cone = static_cast<const JPH::ConeConstraint*>(m_constraint);
            forceImpulseSq = cone->GetTotalLambdaPosition().LengthSq();
            torqueImpulseSq = square(cone->GetTotalLambdaRotation());
            break;
        }
        case JPH::EConstraintSubType::SwingTwist:
        {
            const auto* swingTwist = static_cast<const JPH::SwingTwistConstraint*>(m_constraint);
            forceImpulseSq = swingTwist->GetTotalLambdaPosition().LengthSq();
            torqueImpulseSq = square(swingTwist->GetTotalLambdaTwist()) +
                square(swingTwist->GetTotalLambdaSwingY()) + square(swingTwist->GetTotalLambdaSwingZ());
            break;
        }
        case JPH::EConstraintSubType::Gear:
        {
            // The coupling impulse is angular on both gears; test it as torque.
            torqueImpulseSq = square(static_cast<const JPH::GearConstraint*>(m_constraint)->GetTotalLambda());
            break;
        }
        case JPH::EConstraintSubType::RackAndPinion:
        {
            // Mixed units (torque on the pinion, force on the rack); tested as the
            // torque on the pinion, which is where a real pinion strips.
            torqueImpulseSq =
                square(static_cast<const JPH::RackAndPinionConstraint*>(m_constraint)->GetTotalLambda());
            break;
        }
        default:
            return false;
        }

        const float invDtSq = 1.0f / square(stepDeltaTime);
        return (m_breakForceMax > 0.0f && forceImpulseSq * invDtSq > square(m_breakForceMax)) ||
            (m_breakTorqueMax > 0.0f && torqueImpulseSq * invDtSq > square(m_breakTorqueMax));
    }

    const JointGenericProperties* FindJointGenericProperties(const AzPhysics::JointConfiguration& configuration)
    {
        if (const auto* config = azrtti_cast<const JoltFixedJointConfiguration*>(&configuration))
        {
            return &config->m_genericProperties;
        }
        if (const auto* config = azrtti_cast<const JoltHingeJointConfiguration*>(&configuration))
        {
            return &config->m_genericProperties;
        }
        if (const auto* config = azrtti_cast<const JoltPrismaticJointConfiguration*>(&configuration))
        {
            return &config->m_genericProperties;
        }
        if (const auto* config = azrtti_cast<const JoltDistanceJointConfiguration*>(&configuration))
        {
            return &config->m_genericProperties;
        }
        if (const auto* config = azrtti_cast<const JoltConeJointConfiguration*>(&configuration))
        {
            return &config->m_genericProperties;
        }
        if (const auto* config = azrtti_cast<const JoltSwingTwistJointConfiguration*>(&configuration))
        {
            return &config->m_genericProperties;
        }
        if (const auto* config = azrtti_cast<const JoltBallJointConfiguration*>(&configuration))
        {
            return &config->m_genericProperties;
        }
        if (const auto* config = azrtti_cast<const JoltGearJointConfiguration*>(&configuration))
        {
            return &config->m_genericProperties;
        }
        if (const auto* config = azrtti_cast<const JoltRackAndPinionJointConfiguration*>(&configuration))
        {
            return &config->m_genericProperties;
        }
        return nullptr;
    }

    namespace
    {
        JPH::Vec3 AxisInLocalSpace(const AZ::Quaternion& localRotation, const AZ::Vector3& axis)
        {
            return Conversions::ToJolt(localRotation.TransformVector(axis));
        }

        void ConfigureMotor(JPH::MotorSettings& motorSettings, const JointMotorProperties& motorProperties)
        {
            // Velocity/position motor with a symmetric drive limit; the spring is left
            // disabled so the motor acts as a pure force/torque source.
            motorSettings.mMinForceLimit = -motorProperties.m_driveForceLimit;
            motorSettings.mMaxForceLimit = motorProperties.m_driveForceLimit;
        }

        void ConfigureLimitSpring(JPH::SpringSettings& springSettings, const JointLimitProperties& limitProperties)
        {
            // PhysX-style soft limits give a spring stiffness and damping coefficient
            // directly, which is Jolt's StiffnessAndDamping spring mode (k in N/m or
            // N m/rad, c in N s/m or N m s/rad) - no unit conversion needed.
            springSettings.mMode = JPH::ESpringMode::StiffnessAndDamping;
            springSettings.mStiffness = limitProperties.m_stiffness;
            springSettings.mDamping = limitProperties.m_damping;
        }

        JPH::Constraint* CreateFixedConstraint(
            const AzPhysics::JointConfiguration& configuration, JPH::Body& parentBody, JPH::Body& childBody)
        {
            JPH::FixedConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
            settings.mAutoDetectPoint = false;
            settings.mPoint1 = Conversions::ToJoltR(configuration.m_parentLocalPosition);
            settings.mAxisX1 = AxisInLocalSpace(configuration.m_parentLocalRotation, AZ::Vector3::CreateAxisX());
            settings.mAxisY1 = AxisInLocalSpace(configuration.m_parentLocalRotation, AZ::Vector3::CreateAxisY());
            settings.mPoint2 = Conversions::ToJoltR(configuration.m_childLocalPosition);
            settings.mAxisX2 = AxisInLocalSpace(configuration.m_childLocalRotation, AZ::Vector3::CreateAxisX());
            settings.mAxisY2 = AxisInLocalSpace(configuration.m_childLocalRotation, AZ::Vector3::CreateAxisY());
            return settings.Create(parentBody, childBody);
        }

        JPH::Constraint* CreateHingeConstraint(
            const JoltHingeJointConfiguration& configuration, JPH::Body& parentBody, JPH::Body& childBody)
        {
            JPH::HingeConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
            settings.mPoint1 = Conversions::ToJoltR(configuration.m_parentLocalPosition);
            settings.mHingeAxis1 = AxisInLocalSpace(configuration.m_parentLocalRotation, AZ::Vector3::CreateAxisX());
            settings.mNormalAxis1 = AxisInLocalSpace(configuration.m_parentLocalRotation, AZ::Vector3::CreateAxisY());
            settings.mPoint2 = Conversions::ToJoltR(configuration.m_childLocalPosition);
            settings.mHingeAxis2 = AxisInLocalSpace(configuration.m_childLocalRotation, AZ::Vector3::CreateAxisX());
            settings.mNormalAxis2 = AxisInLocalSpace(configuration.m_childLocalRotation, AZ::Vector3::CreateAxisY());

            if (configuration.m_limitProperties.m_isLimited)
            {
                settings.mLimitsMin = AZ::DegToRad(configuration.m_limitProperties.m_limitFirst);
                settings.mLimitsMax = AZ::DegToRad(configuration.m_limitProperties.m_limitSecond);
                if (configuration.m_limitProperties.m_isSoftLimit)
                {
                    ConfigureLimitSpring(settings.mLimitsSpringSettings, configuration.m_limitProperties);
                }
            }

            if (configuration.m_motorProperties.m_useMotor)
            {
                ConfigureMotor(settings.mMotorSettings, configuration.m_motorProperties);
            }

            return settings.Create(parentBody, childBody);
        }

        JPH::Constraint* CreateBallConstraint(
            const JoltBallJointConfiguration& configuration, JPH::Body& parentBody, JPH::Body& childBody)
        {
            JPH::SwingTwistConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
            settings.mPosition1 = Conversions::ToJoltR(configuration.m_parentLocalPosition);
            settings.mTwistAxis1 = AxisInLocalSpace(configuration.m_parentLocalRotation, AZ::Vector3::CreateAxisX());
            settings.mPlaneAxis1 = AxisInLocalSpace(configuration.m_parentLocalRotation, AZ::Vector3::CreateAxisY());
            settings.mPosition2 = Conversions::ToJoltR(configuration.m_childLocalPosition);
            settings.mTwistAxis2 = AxisInLocalSpace(configuration.m_childLocalRotation, AZ::Vector3::CreateAxisX());
            settings.mPlaneAxis2 = AxisInLocalSpace(configuration.m_childLocalRotation, AZ::Vector3::CreateAxisY());

            settings.mSwingType = JPH::ESwingType::Cone;
            if (configuration.m_limitProperties.m_isLimited)
            {
                settings.mNormalHalfConeAngle = AZ::DegToRad(configuration.m_limitProperties.m_limitFirst);
                settings.mPlaneHalfConeAngle = AZ::DegToRad(configuration.m_limitProperties.m_limitSecond);
                // Jolt's swing-twist constraint has no limit spring, so a soft cone
                // limit cannot be honoured - say so instead of silently hardening it.
                AZ_Warning("JoltPhysics", !configuration.m_limitProperties.m_isSoftLimit,
                    "Ball joint '%s': soft limits are not supported by Jolt's swing-twist constraint; "
                    "the cone limit will be hard.",
                    configuration.m_debugName.c_str());
            }
            else
            {
                // PhysX ball joints are only cone-limited when requested; twist is always free.
                settings.mNormalHalfConeAngle = JPH::JPH_PI;
                settings.mPlaneHalfConeAngle = JPH::JPH_PI;
            }
            settings.mTwistMinAngle = -JPH::JPH_PI;
            settings.mTwistMaxAngle = JPH::JPH_PI;

            return settings.Create(parentBody, childBody);
        }

        JPH::Constraint* CreatePrismaticConstraint(
            const JoltPrismaticJointConfiguration& configuration, JPH::Body& parentBody, JPH::Body& childBody)
        {
            JPH::SliderConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
            settings.mAutoDetectPoint = false;
            settings.mPoint1 = Conversions::ToJoltR(configuration.m_parentLocalPosition);
            settings.mSliderAxis1 = AxisInLocalSpace(configuration.m_parentLocalRotation, AZ::Vector3::CreateAxisX());
            settings.mNormalAxis1 = AxisInLocalSpace(configuration.m_parentLocalRotation, AZ::Vector3::CreateAxisY());
            settings.mPoint2 = Conversions::ToJoltR(configuration.m_childLocalPosition);
            settings.mSliderAxis2 = AxisInLocalSpace(configuration.m_childLocalRotation, AZ::Vector3::CreateAxisX());
            settings.mNormalAxis2 = AxisInLocalSpace(configuration.m_childLocalRotation, AZ::Vector3::CreateAxisY());

            if (configuration.m_limitProperties.m_isLimited)
            {
                settings.mLimitsMin = configuration.m_limitProperties.m_limitFirst;
                settings.mLimitsMax = configuration.m_limitProperties.m_limitSecond;
                if (configuration.m_limitProperties.m_isSoftLimit)
                {
                    ConfigureLimitSpring(settings.mLimitsSpringSettings, configuration.m_limitProperties);
                }
            }

            if (configuration.m_motorProperties.m_useMotor)
            {
                ConfigureMotor(settings.mMotorSettings, configuration.m_motorProperties);
            }

            return settings.Create(parentBody, childBody);
        }

        JPH::Constraint* CreateD6Constraint(
            const JoltD6JointLimitConfiguration& configuration, JPH::Body& parentBody, JPH::Body& childBody)
        {
            JPH::SixDOFConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
            settings.mPosition1 = Conversions::ToJoltR(configuration.m_parentLocalPosition);
            settings.mAxisX1 = AxisInLocalSpace(configuration.m_parentLocalRotation, AZ::Vector3::CreateAxisX());
            settings.mAxisY1 = AxisInLocalSpace(configuration.m_parentLocalRotation, AZ::Vector3::CreateAxisY());
            settings.mPosition2 = Conversions::ToJoltR(configuration.m_childLocalPosition);
            settings.mAxisX2 = AxisInLocalSpace(configuration.m_childLocalRotation, AZ::Vector3::CreateAxisX());
            settings.mAxisY2 = AxisInLocalSpace(configuration.m_childLocalRotation, AZ::Vector3::CreateAxisY());

            // All linear motion locked; angular limits per the configuration.
            settings.MakeFixedAxis(JPH::SixDOFConstraintSettings::EAxis::TranslationX);
            settings.MakeFixedAxis(JPH::SixDOFConstraintSettings::EAxis::TranslationY);
            settings.MakeFixedAxis(JPH::SixDOFConstraintSettings::EAxis::TranslationZ);
            settings.SetLimitedAxis(
                JPH::SixDOFConstraintSettings::EAxis::RotationX,
                AZ::DegToRad(configuration.m_twistLimitLower),
                AZ::DegToRad(configuration.m_twistLimitUpper));
            settings.SetLimitedAxis(
                JPH::SixDOFConstraintSettings::EAxis::RotationY,
                -AZ::DegToRad(configuration.m_swingLimitY),
                AZ::DegToRad(configuration.m_swingLimitY));
            settings.SetLimitedAxis(
                JPH::SixDOFConstraintSettings::EAxis::RotationZ,
                -AZ::DegToRad(configuration.m_swingLimitZ),
                AZ::DegToRad(configuration.m_swingLimitZ));

            return settings.Create(parentBody, childBody);
        }

        JPH::Constraint* CreateDistanceConstraint(
            const JoltDistanceJointConfiguration& configuration, JPH::Body& parentBody, JPH::Body& childBody)
        {
            JPH::DistanceConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
            settings.mPoint1 = Conversions::ToJoltR(configuration.m_parentLocalPosition);
            settings.mPoint2 = Conversions::ToJoltR(configuration.m_childLocalPosition);
            settings.mMinDistance = configuration.m_minDistance;
            settings.mMaxDistance = configuration.m_maxDistance;
            if (configuration.m_springFrequency > 0.0f)
            {
                settings.mLimitsSpringSettings.mMode = JPH::ESpringMode::FrequencyAndDamping;
                settings.mLimitsSpringSettings.mFrequency = configuration.m_springFrequency;
                settings.mLimitsSpringSettings.mDamping = configuration.m_springDamping;
            }
            return settings.Create(parentBody, childBody);
        }

        JPH::Constraint* CreateConeConstraint(
            const JoltConeJointConfiguration& configuration, JPH::Body& parentBody, JPH::Body& childBody)
        {
            JPH::ConeConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
            settings.mPoint1 = Conversions::ToJoltR(configuration.m_parentLocalPosition);
            settings.mTwistAxis1 = AxisInLocalSpace(configuration.m_parentLocalRotation, AZ::Vector3::CreateAxisX());
            settings.mPoint2 = Conversions::ToJoltR(configuration.m_childLocalPosition);
            settings.mTwistAxis2 = AxisInLocalSpace(configuration.m_childLocalRotation, AZ::Vector3::CreateAxisX());
            settings.mHalfConeAngle = AZ::DegToRad(configuration.m_halfConeAngle);
            return settings.Create(parentBody, childBody);
        }

        JPH::Constraint* CreateSwingTwistConstraint(
            const JoltSwingTwistJointConfiguration& configuration, JPH::Body& parentBody, JPH::Body& childBody)
        {
            JPH::SwingTwistConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
            settings.mPosition1 = Conversions::ToJoltR(configuration.m_parentLocalPosition);
            settings.mTwistAxis1 = AxisInLocalSpace(configuration.m_parentLocalRotation, AZ::Vector3::CreateAxisX());
            settings.mPlaneAxis1 = AxisInLocalSpace(configuration.m_parentLocalRotation, AZ::Vector3::CreateAxisY());
            settings.mPosition2 = Conversions::ToJoltR(configuration.m_childLocalPosition);
            settings.mTwistAxis2 = AxisInLocalSpace(configuration.m_childLocalRotation, AZ::Vector3::CreateAxisX());
            settings.mPlaneAxis2 = AxisInLocalSpace(configuration.m_childLocalRotation, AZ::Vector3::CreateAxisY());

            settings.mSwingType = JPH::ESwingType::Cone;
            settings.mNormalHalfConeAngle = AZ::DegToRad(configuration.m_normalHalfConeAngle);
            settings.mPlaneHalfConeAngle = AZ::DegToRad(configuration.m_planeHalfConeAngle);
            settings.mTwistMinAngle = AZ::DegToRad(configuration.m_twistLower);
            settings.mTwistMaxAngle = AZ::DegToRad(configuration.m_twistUpper);
            return settings.Create(parentBody, childBody);
        }

        JPH::Constraint* CreateGearConstraint(
            const JoltGearJointConfiguration& configuration, JPH::Body& parentBody, JPH::Body& childBody)
        {
            if (configuration.m_parentNumTeeth == 0 || configuration.m_childNumTeeth == 0)
            {
                AZ_Warning("JoltPhysics", false,
                    "Gear joint '%s' has a zero tooth count; the ratio would be undefined. Joint not created.",
                    configuration.m_debugName.c_str());
                return nullptr;
            }

            JPH::GearConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
            settings.mHingeAxis1 = AxisInLocalSpace(configuration.m_parentLocalRotation, AZ::Vector3::CreateAxisX());
            settings.mHingeAxis2 = AxisInLocalSpace(configuration.m_childLocalRotation, AZ::Vector3::CreateAxisX());
            settings.SetRatio(
                static_cast<int>(configuration.m_parentNumTeeth), static_cast<int>(configuration.m_childNumTeeth));
            return settings.Create(parentBody, childBody);
        }

        JPH::Constraint* CreateRackAndPinionConstraint(
            const JoltRackAndPinionJointConfiguration& configuration, JPH::Body& parentBody, JPH::Body& childBody)
        {
            if (configuration.m_pinionNumTeeth == 0 || configuration.m_rackLength <= 0.0f)
            {
                AZ_Warning("JoltPhysics", false,
                    "Rack and pinion joint '%s' needs a non-zero pinion tooth count and a positive rack length. "
                    "Joint not created.",
                    configuration.m_debugName.c_str());
                return nullptr;
            }

            JPH::RackAndPinionConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
            // Parent is the pinion (rotates), child is the rack (slides); both take the
            // joint-frame X axis, like the hinge and prismatic joints they accompany.
            settings.mHingeAxis = AxisInLocalSpace(configuration.m_parentLocalRotation, AZ::Vector3::CreateAxisX());
            settings.mSliderAxis = AxisInLocalSpace(configuration.m_childLocalRotation, AZ::Vector3::CreateAxisX());
            settings.SetRatio(
                static_cast<int>(configuration.m_rackNumTeeth),
                configuration.m_rackLength,
                static_cast<int>(configuration.m_pinionNumTeeth));
            return settings.Create(parentBody, childBody);
        }

        //! The JPH::Constraint behind a joint handle, or null when the handle names nothing
        //! in this scene.
        JPH::Constraint* FindConstraint(JoltScene& scene, AzPhysics::JointHandle jointHandle)
        {
            if (jointHandle == AzPhysics::InvalidJointHandle)
            {
                return nullptr;
            }
            if (auto* joint = azrtti_cast<JoltJoint*>(scene.GetJointFromHandle(jointHandle)))
            {
                return joint->GetConstraint();
            }
            return nullptr;
        }
    } // namespace

    JPH::Constraint* CreateJoltConstraint(
        const AzPhysics::JointConfiguration& configuration, JPH::Body& parentBody, JPH::Body& childBody)
    {
        if (const auto* hingeConfig = azrtti_cast<const JoltHingeJointConfiguration*>(&configuration))
        {
            return CreateHingeConstraint(*hingeConfig, parentBody, childBody);
        }
        if (const auto* distanceConfig = azrtti_cast<const JoltDistanceJointConfiguration*>(&configuration))
        {
            return CreateDistanceConstraint(*distanceConfig, parentBody, childBody);
        }
        if (const auto* coneConfig = azrtti_cast<const JoltConeJointConfiguration*>(&configuration))
        {
            return CreateConeConstraint(*coneConfig, parentBody, childBody);
        }
        if (const auto* swingTwistConfig = azrtti_cast<const JoltSwingTwistJointConfiguration*>(&configuration))
        {
            return CreateSwingTwistConstraint(*swingTwistConfig, parentBody, childBody);
        }
        if (const auto* ballConfig = azrtti_cast<const JoltBallJointConfiguration*>(&configuration))
        {
            return CreateBallConstraint(*ballConfig, parentBody, childBody);
        }
        if (const auto* prismaticConfig = azrtti_cast<const JoltPrismaticJointConfiguration*>(&configuration))
        {
            return CreatePrismaticConstraint(*prismaticConfig, parentBody, childBody);
        }
        if (const auto* d6Config = azrtti_cast<const JoltD6JointLimitConfiguration*>(&configuration))
        {
            return CreateD6Constraint(*d6Config, parentBody, childBody);
        }
        if (const auto* gearConfig = azrtti_cast<const JoltGearJointConfiguration*>(&configuration))
        {
            return CreateGearConstraint(*gearConfig, parentBody, childBody);
        }
        if (const auto* rackConfig = azrtti_cast<const JoltRackAndPinionJointConfiguration*>(&configuration))
        {
            return CreateRackAndPinionConstraint(*rackConfig, parentBody, childBody);
        }
        if (azrtti_cast<const JoltFixedJointConfiguration*>(&configuration))
        {
            return CreateFixedConstraint(configuration, parentBody, childBody);
        }
        return nullptr;
    }

    void LinkGearedConstraint(
        JPH::Constraint& constraint, const AzPhysics::JointConfiguration& configuration, JoltScene& scene)
    {
        // Gear and rack-and-pinion couple velocities. Position error therefore accumulates
        // as drift unless Jolt can measure the rotation each accompanying joint has really
        // travelled, which is what these references are for. Both must resolve: with only
        // one, Jolt has nothing to compare against.
        if (const auto* gearConfig = azrtti_cast<const JoltGearJointConfiguration*>(&configuration))
        {
            JPH::Constraint* parentHinge = FindConstraint(scene, gearConfig->m_parentHingeJoint);
            JPH::Constraint* childHinge = FindConstraint(scene, gearConfig->m_childHingeJoint);
            if (parentHinge && childHinge)
            {
                static_cast<JPH::GearConstraint&>(constraint).SetConstraints(parentHinge, childHinge);
            }
        }
        else if (const auto* rackConfig = azrtti_cast<const JoltRackAndPinionJointConfiguration*>(&configuration))
        {
            JPH::Constraint* pinionHinge = FindConstraint(scene, rackConfig->m_pinionHingeJoint);
            JPH::Constraint* rackSlider = FindConstraint(scene, rackConfig->m_rackSliderJoint);
            if (pinionHinge && rackSlider)
            {
                static_cast<JPH::RackAndPinionConstraint&>(constraint).SetConstraints(pinionHinge, rackSlider);
            }
        }
    }
} // namespace JoltPhysics
