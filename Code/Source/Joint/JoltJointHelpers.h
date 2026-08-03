#pragma once

#include <AzCore/Interface/Interface.h>

#include <AzFramework/Physics/Common/PhysicsJoint.h>

namespace JoltPhysics
{
    //! Answers AzPhysics::JointHelpersInterface, which is how the engine asks a physics
    //! backend what joints it can author and what their limits look like.
    //!
    //! This is the interface behind the Animation Editor's ragdoll tools: AzFramework's
    //! CharacterPhysicsDebugDraw::RenderJointLimit and EMotionFX's joint-limit widget and
    //! manipulators all reach for it. In the shipped engine only the PhysX gem registers
    //! it, so with PhysX disabled a project could simulate a ragdoll this gem built but
    //! could not author one - the runtime half was here and the authoring half was not.
    //!
    //! Registered by the system component, mirroring where PhysX registers its own.
    class JoltJointHelpers
        : public AZ::Interface<AzPhysics::JointHelpersInterface>::Registrar
    {
    public:
        AZ_RTTI(JoltPhysics::JoltJointHelpers, "{3C9E7A41-52B8-4D06-9F1E-7A4C2B5D8E63}", AzPhysics::JointHelpersInterface);

        // AzPhysics::JointHelpersInterface
        const AZStd::vector<AZ::TypeId> GetSupportedJointTypeIds() const override;
        AZStd::optional<const AZ::TypeId> GetSupportedJointTypeId(AzPhysics::JointType typeEnum) const override;

        AZStd::unique_ptr<AzPhysics::JointConfiguration> ComputeInitialJointLimitConfiguration(
            const AZ::TypeId& jointLimitTypeId,
            const AZ::Quaternion& parentWorldRotation,
            const AZ::Quaternion& childWorldRotation,
            const AZ::Vector3& axis,
            const AZStd::vector<AZ::Quaternion>& exampleLocalRotations) override;

        void GenerateJointLimitVisualizationData(
            const AzPhysics::JointConfiguration& configuration,
            const AZ::Quaternion& parentRotation,
            const AZ::Quaternion& childRotation,
            float scale,
            AZ::u32 angularSubdivisions,
            AZ::u32 radialSubdivisions,
            AZStd::vector<AZ::Vector3>& vertexBufferOut,
            AZStd::vector<AZ::u32>& indexBufferOut,
            AZStd::vector<AZ::Vector3>& lineBufferOut,
            AZStd::vector<bool>& lineValidityBufferOut) override;
    };
} // namespace JoltPhysics
