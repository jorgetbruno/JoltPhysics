#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Matrix3x3.h>

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Mat44.h>

namespace JoltPhysics
{
    namespace Conversions
    {
        inline JPH::Vec3 ToJolt(const AZ::Vector3& v)
        {
            return JPH::Vec3(v.GetX(), v.GetY(), v.GetZ());
        }

        inline JPH::RVec3 ToJoltR(const AZ::Vector3& v)
        {
            return JPH::RVec3(v.GetX(), v.GetY(), v.GetZ());
        }

        inline JPH::Quat ToJolt(const AZ::Quaternion& q)
        {
            return JPH::Quat(q.GetX(), q.GetY(), q.GetZ(), q.GetW());
        }

        inline AZ::Vector3 FromJolt(const JPH::Vec3& v)
        {
            return AZ::Vector3(v.GetX(), v.GetY(), v.GetZ());
        }

#ifdef JPH_DOUBLE_PRECISION
        inline AZ::Vector3 FromJolt(const JPH::RVec3& v)
        {
            return AZ::Vector3(
                static_cast<float>(v.GetX()),
                static_cast<float>(v.GetY()),
                static_cast<float>(v.GetZ())
            );
        }
#endif // JPH_DOUBLE_PRECISION

        inline AZ::Quaternion FromJolt(const JPH::Quat& q)
        {
            return AZ::Quaternion(q.GetX(), q.GetY(), q.GetZ(), q.GetW());
        }

        inline AZ::Transform FromJolt(const JPH::RMat44& m)
        {
            AZ::Transform transform;
            transform.SetTranslation(FromJolt(m.GetTranslation()));
            transform.SetRotation(FromJolt(m.GetQuaternion()));
            return transform;
        }

        inline JPH::Mat44 ToJolt(const AZ::Transform& t)
        {
            return JPH::Mat44::sRotationTranslation(
                ToJolt(t.GetRotation()),
                ToJolt(t.GetTranslation())
            );
        }

    } // namespace Conversions
} // namespace JoltPhysics
