#include <Character/JoltSkeletonMapper.h>

#include <Character/JoltRagdoll.h>
#include <Utils/Conversions.h>

#include <Jolt/Physics/Ragdoll/Ragdoll.h>

namespace JoltPhysics
{
    namespace
    {
        //! Model-space neutral pose of the ragdoll skeleton, taken from the pose its parts
        //! were built in (JoltRagdoll builds them from the configuration's initial state).
        JPH::Array<JPH::Mat44> BuildRagdollNeutralPose(const JPH::RagdollSettings& settings)
        {
            JPH::Array<JPH::Mat44> neutralPose;
            neutralPose.reserve(settings.mParts.size());
            for (const JPH::RagdollSettings::Part& part : settings.mParts)
            {
                neutralPose.push_back(JPH::Mat44::sRotationTranslation(part.mRotation, JPH::Vec3(part.mPosition)));
            }
            return neutralPose;
        }

        //! Converts a model-space pose to local (parent-relative) space.
        JPH::Array<JPH::Mat44> ToLocalSpace(const JPH::Array<JPH::Mat44>& modelSpace, const AZStd::vector<int>& parentIndices)
        {
            JPH::Array<JPH::Mat44> localSpace(modelSpace.size(), JPH::Mat44::sIdentity());
            for (size_t i = 0; i < modelSpace.size(); ++i)
            {
                const int parentIndex = parentIndices[i];
                localSpace[i] = (parentIndex >= 0) ? modelSpace[parentIndex].Inversed() * modelSpace[i] : modelSpace[i];
            }
            return localSpace;
        }
    }

    bool JoltSkeletonMapper::Initialize(const JoltRagdoll& ragdoll, const JoltAnimationSkeleton& animationSkeleton)
    {
        m_initialized = false;

        const JPH::RagdollSettings* ragdollSettings = ragdoll.GetNativeSettings();
        if (!ragdollSettings || !ragdollSettings->GetSkeleton())
        {
            AZ_Warning("JoltPhysics", false, "JoltSkeletonMapper: the ragdoll has not been created yet.");
            return false;
        }

        if (!animationSkeleton.IsValid())
        {
            AZ_Warning("JoltPhysics", false,
                "JoltSkeletonMapper: the animation skeleton is empty or its joint names, parent indices and "
                "neutral pose have different lengths.");
            return false;
        }

        // Build the Jolt-side animation skeleton. Parents must precede children so the
        // parent index is always already known.
        m_animationSkeleton = new JPH::Skeleton();
        for (size_t i = 0; i < animationSkeleton.m_jointNames.size(); ++i)
        {
            const int parentIndex = animationSkeleton.m_parentIndices[i];
            if (parentIndex >= static_cast<int>(i))
            {
                AZ_Warning("JoltPhysics", false,
                    "JoltSkeletonMapper: joint '%s' has parent index %d, which does not precede it. Animation "
                    "skeleton joints must be ordered parents first.",
                    animationSkeleton.m_jointNames[i].c_str(), parentIndex);
                return false;
            }
            m_animationSkeleton->AddJoint(animationSkeleton.m_jointNames[i].c_str(), parentIndex);
        }
        m_animationSkeleton->CalculateParentJointIndices();

        m_animationNeutralPoseModelSpace.clear();
        m_animationNeutralPoseModelSpace.reserve(animationSkeleton.m_neutralPoseModelSpace.size());
        for (const AZ::Transform& jointTransform : animationSkeleton.m_neutralPoseModelSpace)
        {
            m_animationNeutralPoseModelSpace.push_back(Conversions::ToJolt(jointTransform));
        }
        m_animationNeutralPoseLocalSpace =
            ToLocalSpace(m_animationNeutralPoseModelSpace, animationSkeleton.m_parentIndices);

        const JPH::Array<JPH::Mat44> ragdollNeutralPose = BuildRagdollNeutralPose(*ragdollSettings);
        m_ragdollJointCount = ragdollNeutralPose.size();

        // Jolt names the ragdoll skeleton "1" and the animation skeleton "2".
        m_mapper = new JPH::SkeletonMapper();
        m_mapper->Initialize(
            ragdollSettings->GetSkeleton(), ragdollNeutralPose.data(), m_animationSkeleton,
            m_animationNeutralPoseModelSpace.data());

        if (GetMappedJointCount() == 0)
        {
            AZ_Warning("JoltPhysics", false,
                "JoltSkeletonMapper: no joints could be matched between the ragdoll and the animation skeleton. "
                "Joints are matched by name, so the ragdoll node names must match the animation joint names.");
            return false;
        }

        m_initialized = true;
        return true;
    }

    size_t JoltSkeletonMapper::GetMappedJointCount() const
    {
        return m_mapper ? m_mapper->GetMappings().size() : 0;
    }

    void JoltSkeletonMapper::LockAllTranslations()
    {
        if (!m_initialized)
        {
            return;
        }
        m_mapper->LockAllTranslations(m_animationSkeleton, m_animationNeutralPoseModelSpace.data());
    }

    bool JoltSkeletonMapper::MapRagdollStateToAnimationPose(
        const Physics::RagdollState& ragdollState,
        AZStd::vector<AZ::Transform>& outAnimationPoseModelSpace,
        const AZStd::vector<AZ::Transform>* animationPoseLocalSpace) const
    {
        if (!m_initialized)
        {
            return false;
        }

        if (ragdollState.size() < m_ragdollJointCount)
        {
            AZ_Warning("JoltPhysics", false,
                "JoltSkeletonMapper: the ragdoll state has %zu nodes but the ragdoll skeleton has %zu joints.",
                ragdollState.size(), m_ragdollJointCount);
            return false;
        }

        JPH::Array<JPH::Mat44> ragdollPoseModelSpace;
        ragdollPoseModelSpace.reserve(m_ragdollJointCount);
        for (size_t i = 0; i < m_ragdollJointCount; ++i)
        {
            ragdollPoseModelSpace.push_back(JPH::Mat44::sRotationTranslation(
                Conversions::ToJolt(ragdollState[i].m_orientation), Conversions::ToJolt(ragdollState[i].m_position)));
        }

        // Joints that only exist in the animation skeleton are positioned from their local
        // transform; without one from the caller they stay in their bind pose.
        JPH::Array<JPH::Mat44> localSpace;
        const JPH::Mat44* localSpacePose = m_animationNeutralPoseLocalSpace.data();
        if (animationPoseLocalSpace != nullptr)
        {
            if (animationPoseLocalSpace->size() != m_animationNeutralPoseLocalSpace.size())
            {
                AZ_Warning("JoltPhysics", false,
                    "JoltSkeletonMapper: the local-space animation pose has %zu joints but the animation skeleton "
                    "has %zu.",
                    animationPoseLocalSpace->size(), m_animationNeutralPoseLocalSpace.size());
                return false;
            }
            localSpace.reserve(animationPoseLocalSpace->size());
            for (const AZ::Transform& jointTransform : *animationPoseLocalSpace)
            {
                localSpace.push_back(Conversions::ToJolt(jointTransform));
            }
            localSpacePose = localSpace.data();
        }

        JPH::Array<JPH::Mat44> mappedPose(m_animationNeutralPoseModelSpace.size(), JPH::Mat44::sIdentity());
        m_mapper->Map(ragdollPoseModelSpace.data(), localSpacePose, mappedPose.data());

        outAnimationPoseModelSpace.clear();
        outAnimationPoseModelSpace.reserve(mappedPose.size());
        for (const JPH::Mat44& jointMatrix : mappedPose)
        {
            outAnimationPoseModelSpace.push_back(Conversions::FromJolt(jointMatrix));
        }
        return true;
    }

    bool JoltSkeletonMapper::MapAnimationPoseToRagdollState(
        const AZStd::vector<AZ::Transform>& animationPoseModelSpace, Physics::RagdollState& outRagdollState) const
    {
        if (!m_initialized)
        {
            return false;
        }

        if (animationPoseModelSpace.size() != m_animationNeutralPoseModelSpace.size())
        {
            AZ_Warning("JoltPhysics", false,
                "JoltSkeletonMapper: the animation pose has %zu joints but the animation skeleton has %zu.",
                animationPoseModelSpace.size(), m_animationNeutralPoseModelSpace.size());
            return false;
        }

        JPH::Array<JPH::Mat44> animationPose;
        animationPose.reserve(animationPoseModelSpace.size());
        for (const AZ::Transform& jointTransform : animationPoseModelSpace)
        {
            animationPose.push_back(Conversions::ToJolt(jointTransform));
        }

        JPH::Array<JPH::Mat44> ragdollPose(m_ragdollJointCount, JPH::Mat44::sIdentity());
        m_mapper->MapReverse(animationPose.data(), ragdollPose.data());

        if (outRagdollState.size() < m_ragdollJointCount)
        {
            outRagdollState.resize(m_ragdollJointCount);
        }
        for (size_t i = 0; i < m_ragdollJointCount; ++i)
        {
            outRagdollState[i].m_position = Conversions::FromJolt(ragdollPose[i].GetTranslation());
            outRagdollState[i].m_orientation = Conversions::FromJolt(ragdollPose[i].GetQuaternion());
        }
        return true;
    }

} // namespace JoltPhysics
