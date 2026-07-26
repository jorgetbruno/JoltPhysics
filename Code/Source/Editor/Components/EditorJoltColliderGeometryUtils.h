#pragma once

#include <AzCore/Math/Aabb.h>
#include <AzCore/Math/Vector2.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>

namespace JPH
{
    class Shape;
}

namespace JoltPhysics::EditorColliderGeometry
{
    //! Geometry the editor colliders need to draw and to be picked in the viewport,
    //! kept out of the components so it can be tested without an entity or a viewport.

    //! Fills outLines with the triangle edges of a native shape (point pairs, shape-local
    //! space) and outBounds with the bounds of the same vertices. Both are cleared first;
    //! a null shape leaves an empty list and a null Aabb.
    void BuildShapeWireframe(const JPH::Shape* shape, AZStd::vector<AZ::Vector3>& outLines, AZ::Aabb& outBounds);

    //! A heightfield as reported by Physics::HeightfieldProviderRequestsBus: a row-major
    //! grid of heights on a regular lattice.
    struct HeightfieldGrid
    {
        AZ::u32 m_columns = 0;
        AZ::u32 m_rows = 0;
        AZ::Vector2 m_spacing = AZ::Vector2::CreateOne();
        AZStd::vector<float> m_heights;

        bool IsValid() const
        {
            return m_columns >= 2 && m_rows >= 2 &&
                m_heights.size() == static_cast<size_t>(m_columns) * static_cast<size_t>(m_rows) &&
                m_spacing.GetX() > 0.0f && m_spacing.GetY() > 0.0f;
        }
    };

    //! Entity-local position of one grid sample.
    //!
    //! Jolt heightfields are Y-up - the surface is scale * (column, height, row) - and
    //! JoltHeightfieldUtils::WrapZUp rotates them +90 degrees about X for O3DE, mapping
    //! (x, height, y) to (x, -y, height). The grid therefore runs along +X and -Y from
    //! the entity origin. This has to match, or the wireframe would sit somewhere other
    //! than the collider it describes.
    AZ::Vector3 HeightfieldSamplePosition(const HeightfieldGrid& grid, AZ::u32 column, AZ::u32 row);

    //! Bounds of the heightfield surface in entity-local space; null for an invalid grid.
    AZ::Aabb ComputeHeightfieldBounds(const HeightfieldGrid& grid);

    //! Grid wireframe as a line list (point pairs, entity-local space). Large grids are
    //! strided so neither axis draws more than maxLinesPerAxis lines - a full-resolution
    //! terrain heightfield is hundreds of thousands of segments - while the last row and
    //! column are always drawn so the wireframe reaches the edge of the collider.
    AZStd::vector<AZ::Vector3> BuildHeightfieldWireframe(const HeightfieldGrid& grid, AZ::u32 maxLinesPerAxis);
} // namespace JoltPhysics::EditorColliderGeometry
