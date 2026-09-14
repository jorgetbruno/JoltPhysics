#pragma once

#include <AzCore/Math/Color.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/containers/vector.h>
#include <AzFramework/Physics/SystemBus.h>

#include <Jolt/Jolt.h>
#include <Jolt/Renderer/DebugRenderer.h>

namespace JoltPhysics
{
    //! World-space points grouped by colour, for a caller that flushes them in one go
    //! per colour rather than taking a callback per primitive. Keys are AZ::Color::ToU32.
    struct JoltDebugDrawSink
    {
        AZStd::unordered_map<AZ::u32, AZStd::vector<AZ::Vector3>> m_linesByColor;
        AZStd::unordered_map<AZ::u32, AZStd::vector<AZ::Vector3>> m_trianglesByColor;

        //! Empties every buffer but keeps its capacity, so a warm frame allocates nothing.
        void Clear();
    };

    //! Jolt's debug renderer, implemented in full rather than through DebugRendererSimple.
    //!
    //! DebugRendererSimple is what this used to be, and it is what made jolt_Debug cost 50 ms
    //! a frame on a city level. Jolt caches each shape's triangles once, but the simple
    //! renderer unpacks that cache on every frame: three matrix transforms and three virtual
    //! DrawLine calls per triangle, each reaching a callback that converted the colour, built
    //! two vertices and looked the colour up in a hash map. Measured on 647 shapes: 410,424
    //! triangles became 1,232,002 lines a frame at 27 ns each in the walk, plus 14 ms to hand
    //! them to the renderer.
    //!
    //! This does the expensive part once per shape, when Jolt builds the batch: it keeps the
    //! UNIQUE edges, since every interior edge of a mesh belongs to two triangles and was
    //! being drawn twice. Per frame, DrawGeometry only transforms those edges in a tight loop
    //! into the sink's per-colour buffer.
    //!
    //! Long-lived, deliberately. Jolt's constructor asserts there is only one renderer, and
    //! Initialize() tessellates the unit box, sphere, capsule and cylinder at every level of
    //! detail - which the gem was doing again every frame by building one per draw.
    //!
    //! **Nothing else in the process may draw Jolt shapes through a different renderer
    //! type.** A shape caches the batch the first renderer built for it, and every renderer
    //! casts that batch back to its own type. This one and a DebugRendererSimple drawing the
    //! same shape would each read the other's batch as their own.
    class JoltDebugRenderer final : public JPH::DebugRenderer
    {
    public:
        JPH_OVERRIDE_NEW_DELETE

        JoltDebugRenderer();

        //! Where primitives go. A sink takes priority: it is the fast path, filled directly.
        //! Without one they go to the settings' callbacks one at a time, which is the
        //! Physics::SystemDebugRequestBus contract for callers that supply their own.
        void SetTarget(const Physics::DebugDrawSettings* settings, JoltDebugDrawSink* sink);

        void SetCameraPosition(const AZ::Vector3& cameraPosition);

        //! Counts since the last ResetCounters, for jolt_DebugDrawProfile.
        AZ::u64 GetGeometryDrawCount() const { return m_geometryDraws; }
        AZ::u64 GetGeometryTriangleCount() const { return m_geometryTriangles; }
        AZ::u64 GetLineCount() const { return m_lines; }
        void ResetCounters();

        // JPH::DebugRenderer
        void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor) override;
        void DrawTriangle(
            JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3, JPH::ColorArg inColor,
            ECastShadow inCastShadow = ECastShadow::Off) override;
        Batch CreateTriangleBatch(const Triangle* inTriangles, int inTriangleCount) override;
        Batch CreateTriangleBatch(const Vertex* inVertices, int inVertexCount, const JPH::uint32* inIndices, int inIndexCount) override;
        void DrawGeometry(
            JPH::RMat44Arg inModelMatrix, const JPH::AABox& inWorldSpaceBounds, float inLODScaleSq,
            JPH::ColorArg inModelColor, const GeometryRef& inGeometry, ECullMode inCullMode,
            ECastShadow inCastShadow, EDrawMode inDrawMode) override;
        void DrawText3D(
            [[maybe_unused]] JPH::RVec3Arg inPosition, [[maybe_unused]] const JPH::string_view& inString,
            [[maybe_unused]] JPH::ColorArg inColor, [[maybe_unused]] float inHeight) override
        {
            // Text rendering is not supported by the O3DE debug draw callbacks.
        }

    private:
        const Physics::DebugDrawSettings* m_settings = nullptr;
        JoltDebugDrawSink* m_sink = nullptr;
        JPH::Vec3 m_cameraPosition = JPH::Vec3::sZero();
        bool m_cameraPositionSet = false;

        AZ::u64 m_geometryDraws = 0;
        AZ::u64 m_geometryTriangles = 0;
        AZ::u64 m_lines = 0;
    };
} // namespace JoltPhysics
