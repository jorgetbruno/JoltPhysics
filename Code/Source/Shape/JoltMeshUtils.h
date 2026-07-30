#pragma once

#include <AzCore/Math/Transform.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>

#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Visibility/VisibleGeometryBus.h>

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

        //! Same as above, plus a per-face material index table (blob version 2). The
        //! indices map faces onto the material slots the product asset carries; Jolt
        //! packs them as 5 bits per triangle, so valid slot indices are 0..31.
        //! materialIndices must hold exactly indexCount / 3 entries.
        static AZStd::vector<AZ::u8> PackTriangleMesh(
            const AZ::Vector3* vertices,
            AZ::u32 vertexCount,
            const AZ::u32* indices,
            AZ::u32 indexCount,
            const AZ::u8* materialIndices,
            AZ::u32 faceCount);

        //! Builds a Jolt mesh shape from a previously packed blob (as produced by
        //! PackTriangleMesh). Returns nullptr if the blob is malformed or empty.
        static JPH::RefConst<JPH::Shape> CreateMeshShapeFromCookedData(const AZStd::vector<AZ::u8>& cookedData);

        //! Packs raw convex-hull point data (vertices only, no indices) into a blob.
        //! Jolt builds the hull from the point cloud on Create(), so no offline pass is
        //! needed - this is just the packed points.
        static AZStd::vector<AZ::u8> PackConvexMesh(const AZ::Vector3* vertices, AZ::u32 vertexCount);

        //! Packs several convex-hull point clouds into one "hull group" blob (blob
        //! version 2; PackConvexMesh writes the single-hull version 1). Hulls with no
        //! points are skipped. Decode with CreateConvexShapeFromCookedData.
        static AZStd::vector<AZ::u8> PackConvexHulls(const AZStd::vector<AZStd::vector<AZ::Vector3>>& hulls);

        //! Builds a Jolt convex shape from a blob produced by PackConvexMesh (single
        //! hull) or PackConvexHulls (one hull -> bare hull shape; several -> a static
        //! compound of hulls, all points being entity-local so children sit at identity).
        //! Returns nullptr if the blob is malformed or empty.
        static JPH::RefConst<JPH::Shape> CreateConvexShapeFromCookedData(const AZStd::vector<AZ::u8>& cookedData);

        //! How CookVisibleGeometry groups convex-hull points when meshType is Convex
        //! (ignored for triangle meshes, which are always merged).
        enum class ConvexGrouping
        {
            Single, //!< One hull around all geometry.
            PerGeometryEntry, //!< One hull per VisibleGeometry entry (i.e. per render node).
        };

        //! Gathers all render geometry into one entity-local triangle soup (the input
        //! the editor's convex decomposition runs on). Returns false when the container
        //! holds no triangles.
        static bool GatherVisibleGeometrySoup(
            const AzFramework::VisibleGeometryContainer& geometryContainer,
            const AZ::Transform& entityWorldTransform,
            AZStd::vector<AZ::Vector3>& outVertices,
            AZStd::vector<AZ::u32>& outIndices);

        //! Cooks render geometry (as reported by AzFramework::VisibleGeometryRequestBus)
        //! into a cooked mesh shape configuration. The geometry entries carry local-to-world
        //! transforms; vertices are brought into the entity's local space (via the inverse of
        //! entityWorldTransform) so the resulting collider follows the entity's transform.
        //! Returns false when the container holds no triangles.
        static bool CookVisibleGeometry(
            const AzFramework::VisibleGeometryContainer& geometryContainer,
            const AZ::Transform& entityWorldTransform,
            Physics::CookedMeshShapeConfiguration::MeshType meshType,
            Physics::CookedMeshShapeConfiguration& outConfiguration,
            ConvexGrouping convexGrouping = ConvexGrouping::Single);
    };

} // namespace JoltPhysics
