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

        //! Built through the factory rather than by assembling a default-constructed
        //! transform, because AZ::Transform's default constructor is `= default` on a type
        //! with no initialisers: the scale starts as whatever was on the stack. Setting
        //! translation and rotation and leaving scale alone therefore returned a transform
        //! that was correct in position and orientation and carried garbage scale - and a
        //! caller that applied it with SetWorldTM scaled an entity by that garbage.
        //!
        //! Landing a zero there collapses the mesh to nothing and floods the log with
        //! "GetInverseFull could not calculate inverse as determinant was zero". That is
        //! what it did: a vehicle's wheel meshes, driven from GetWheelTransform, vanished
        //! the moment anything read one, while every position they reported was correct.
        //! Nothing was out of place; the meshes had no size.
        inline AZ::Transform FromJolt(const JPH::RMat44& m)
        {
            // Jolt's transforms carry no scale, so a unit scale is the honest answer.
            return AZ::Transform::CreateFromQuaternionAndTranslation(
                FromJolt(m.GetQuaternion()), FromJolt(m.GetTranslation()));
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
