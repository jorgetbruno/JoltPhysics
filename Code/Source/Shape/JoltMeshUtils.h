#pragma once

#include <AzCore/std/containers/vector.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace JoltPhysics
{
    //! Helpers for building Jolt triangle mesh shapes from raw geometry, and for
    //! (de)serializing that geometry into the "cooked" blob stored on a
    //! Physics::CookedMeshShapeConfiguration. Jolt's MeshShape needs no offline
    //! cooking pass (its BVH is built on Create()), so the "cooked" format here is
    //! simply the packed vertex/index data.
    class JoltMeshUtils
    {
    public:
        //! Packs raw vertex/index data into the blob format used by
        //! Physics::CookedMeshShapeConfiguration::SetCookedMeshData.
        static AZStd::vector<AZ::u8> PackTriangleMesh(
            const AZ::Vector3* vertices,
            AZ::u32 vertexCount,
            const AZ::u32* indices,
            AZ::u32 indexCount);

        //! Builds a Jolt mesh shape from a previously packed blob (as produced by
        //! PackTriangleMesh). Returns nullptr if the blob is malformed or empty.
        static JPH::RefConst<JPH::Shape> CreateMeshShapeFromCookedData(const AZStd::vector<AZ::u8>& cookedData);
    };

} // namespace JoltPhysics
