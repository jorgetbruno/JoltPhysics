#pragma once

#include <AzCore/Math/Transform.h>
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

#include <AzFramework/Physics/Ragdoll.h>

#include <Jolt/Jolt.h>
#include <Jolt/Core/Reference.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Skeleton/SkeletonMapper.h>

namespace JoltPhysics
{
    class JoltRagdoll;

    //! Describes the animation (high detail) skeleton that a ragdoll is mapped against.
    //! It must have the same hierarchy as the ragdoll skeleton but may add joints between
    //! ragdoll joints and at the root or leaves. Joints are matched to ragdoll nodes by
    //! name (a ragdoll node's name is its RagdollNodeConfiguration debug name).
    struct JoltAnimationSkeleton
    {
        AZStd::vector<AZStd::string> m_jointNames;
        //! Parent joint index per joint; -1 for a root joint. Parents must precede children.
        AZStd::vector<int> m_parentIndices;
        //! The bind pose, in model space, one entry per joint.
        AZStd::vector<AZ::Transform> m_neutralPoseModelSpace;

        bool IsValid() const
        {
            return !m_jointNames.empty() && m_parentIndices.size() == m_jointNames.size() &&
                m_neutralPoseModelSpace.size() == m_jointNames.size();
        }
    };

    //! Maps poses between a ragdoll's (low detail) skeleton and an animation (high detail)
    //! skeleton, so a simulated ragdoll can drive a rendered character and an animated
    //! pose can drive the ragdoll. Wraps JPH::SkeletonMapper.
    //!
    //! The two neutral poses should line up as closely as possible - ideally the mapped
    //! joints sit at identical positions - since the mapping is derived from them.
    class JoltSkeletonMapper
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltSkeletonMapper, AZ::SystemAllocator);

        //! Builds the mapping between the ragdoll's skeleton and the animation skeleton.
        //! Returns false if either skeleton is unusable or no joints could be matched.
        bool Initialize(const JoltRagdoll& ragdoll, const JoltAnimationSkeleton& animationSkeleton);

        bool IsInitialized() const
        {
            return m_initialized;
        }

        //! Number of joints matched one-to-one between the two skeletons.
        size_t GetMappedJointCount() const;

        //! Pins the translation of the animation joints below the first mapped joint to
        //! the neutral pose. Constraints are never perfectly rigid, so a ragdoll under
        //! stress stretches slightly; this hides that at the cost of the rendered pose no
        //! longer exactly matching the simulation. Call after Initialize.
        void LockAllTranslations();

        //! Simulated ragdoll -> animation skeleton, for rendering a ragdolling character.
        //! Joints that exist only in the animation skeleton keep their local transform
        //! from animationPoseLocalSpace; when that is null the neutral pose is used, which
        //! leaves those joints in their bind pose relative to their parent.
        bool MapRagdollStateToAnimationPose(
            const Physics::RagdollState& ragdollState,
            AZStd::vector<AZ::Transform>& outAnimationPoseModelSpace,
            const AZStd::vector<AZ::Transform>* animationPoseLocalSpace = nullptr) const;

        //! Animation skeleton -> ragdoll, for keying the ragdoll to an animated pose
        //! (feed the result to Ragdoll::SetState, DriveToPoseUsingKinematics or
        //! DriveToPoseUsingMotors). Only position and orientation are written; any other
        //! per-node fields already present in outRagdollState are left untouched.
        bool MapAnimationPoseToRagdollState(
            const AZStd::vector<AZ::Transform>& animationPoseModelSpace,
            Physics::RagdollState& outRagdollState) const;

    private:
        //! Jolt calls the ragdoll skeleton "1" and the animation skeleton "2"; the same
        //! naming is kept here so the wrapping stays easy to follow against its docs.
        JPH::Ref<JPH::Skeleton> m_animationSkeleton;
        JPH::Ref<JPH::SkeletonMapper> m_mapper;

        //! JPH::Array keeps Mat44's 16-byte alignment, which AZStd::vector does not.
        JPH::Array<JPH::Mat44> m_animationNeutralPoseModelSpace;
        JPH::Array<JPH::Mat44> m_animationNeutralPoseLocalSpace;

        size_t m_ragdollJointCount = 0;
        bool m_initialized = false;
    };
} // namespace JoltPhysics
