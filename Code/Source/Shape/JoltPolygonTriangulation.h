#pragma once

#include <AzCore/Math/Vector2.h>
#include <AzCore/std/containers/vector.h>

namespace JoltPhysics
{
    //! Triangulates a simple polygon outline by ear clipping.
    //!
    //! This is what lets a CONCAVE polygon prism collide as its true outline. A prism
    //! is a 2D outline extruded along one axis, so the concavity is confined to a
    //! plane: cut the footprint into triangles here and extrude each one, and every
    //! piece is convex, the union is exactly the solid, and no 3D decomposition
    //! (V-HACD, an editor-time bake) is needed. A U-shaped blocker used to collide as
    //! a solid block - the hull of its outline - and nothing said so.
    //!
    //! O(n^2) over the vertex count, which for an authored outline of tens of points
    //! is nothing, and it runs once per shape change rather than per step.
    //!
    //! Accepts either winding. Rejects fewer than three vertices, a degenerate
    //! (zero-area) outline, and an outline it cannot clip an ear from - which is what a
    //! self-intersecting polygon looks like from here - returning an empty list so the
    //! caller can say so rather than hand Jolt garbage. Consecutive duplicate vertices
    //! and an explicitly closed outline (last vertex == first) are tolerated.
    class JoltPolygonTriangulation
    {
    public:
        //! Indices into `outline`, three per triangle, all with the same winding as the
        //! input outline. Empty on failure.
        static AZStd::vector<AZ::u32> EarClip(const AZStd::vector<AZ::Vector2>& outline);

        //! Whether the outline is convex (no reflex vertices), for callers that can take
        //! a cheaper path when it is. A collinear run does not count as reflex.
        static bool IsConvex(const AZStd::vector<AZ::Vector2>& outline);

        //! Twice the signed area of the outline; positive for counter-clockwise.
        static float SignedAreaTwice(const AZStd::vector<AZ::Vector2>& outline);
    };
} // namespace JoltPhysics
