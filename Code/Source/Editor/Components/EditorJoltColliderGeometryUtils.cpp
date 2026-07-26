#include <Editor/Components/EditorJoltColliderGeometryUtils.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include <cmath>

namespace JoltPhysics::EditorColliderGeometry
{
    namespace
    {
        //! Sample indices to draw along one axis: every stride-th sample, always ending
        //! on the last one so the wireframe reaches the edge of the collider even when
        //! the stride does not divide the grid evenly.
        AZStd::vector<AZ::u32> BuildDrawIndices(AZ::u32 samples, AZ::u32 maxLines)
        {
            AZStd::vector<AZ::u32> indices;
            if (samples == 0)
            {
                return indices;
            }

            const AZ::u32 spans = samples - 1;
            const AZ::u32 stride =
                (maxLines == 0 || spans <= maxLines) ? 1 : (spans + maxLines - 1) / maxLines;

            for (AZ::u32 i = 0; i < samples; i += stride)
            {
                indices.push_back(i);
            }
            if (indices.back() != samples - 1)
            {
                indices.push_back(samples - 1);
            }
            return indices;
        }
    } // namespace

    void BuildShapeWireframe(const JPH::Shape* shape, AZStd::vector<AZ::Vector3>& outLines, AZ::Aabb& outBounds)
    {
        outLines.clear();
        outBounds = AZ::Aabb::CreateNull();
        if (!shape)
        {
            return;
        }

        JPH::Shape::GetTrianglesContext context;
        shape->GetTrianglesStart(
            context, shape->GetLocalBounds(), JPH::Vec3::sZero(), JPH::Quat::sIdentity(), JPH::Vec3::sReplicate(1.0f));

        constexpr int batchSize = JPH::Shape::cGetTrianglesMinTrianglesRequested;
        JPH::Float3 buffer[batchSize * 3];
        int triangleCount = 0;
        while ((triangleCount = shape->GetTrianglesNext(context, batchSize, buffer)) > 0)
        {
            for (int t = 0; t < triangleCount; ++t)
            {
                const AZ::Vector3 vertices[3] = {
                    AZ::Vector3(buffer[t * 3 + 0].x, buffer[t * 3 + 0].y, buffer[t * 3 + 0].z),
                    AZ::Vector3(buffer[t * 3 + 1].x, buffer[t * 3 + 1].y, buffer[t * 3 + 1].z),
                    AZ::Vector3(buffer[t * 3 + 2].x, buffer[t * 3 + 2].y, buffer[t * 3 + 2].z),
                };

                for (int edge = 0; edge < 3; ++edge)
                {
                    outLines.push_back(vertices[edge]);
                    outLines.push_back(vertices[(edge + 1) % 3]);
                }
                for (const AZ::Vector3& vertex : vertices)
                {
                    outBounds.AddPoint(vertex);
                }
            }
        }
    }

    AZ::Vector3 HeightfieldSamplePosition(const HeightfieldGrid& grid, AZ::u32 column, AZ::u32 row)
    {
        const size_t index = static_cast<size_t>(row) * grid.m_columns + column;
        const float height = index < grid.m_heights.size() ? grid.m_heights[index] : 0.0f;
        return AZ::Vector3(
            static_cast<float>(column) * grid.m_spacing.GetX(),
            -static_cast<float>(row) * grid.m_spacing.GetY(),
            height);
    }

    AZ::Aabb ComputeHeightfieldBounds(const HeightfieldGrid& grid)
    {
        if (!grid.IsValid())
        {
            return AZ::Aabb::CreateNull();
        }

        float minHeight = std::numeric_limits<float>::max();
        float maxHeight = std::numeric_limits<float>::lowest();
        for (const float height : grid.m_heights)
        {
            // A provider may leave holes as non-finite samples; they carry no surface.
            if (std::isfinite(height))
            {
                minHeight = AZStd::min(minHeight, height);
                maxHeight = AZStd::max(maxHeight, height);
            }
        }
        if (minHeight > maxHeight)
        {
            return AZ::Aabb::CreateNull();
        }

        return AZ::Aabb::CreateFromMinMax(
            AZ::Vector3(0.0f, -static_cast<float>(grid.m_rows - 1) * grid.m_spacing.GetY(), minHeight),
            AZ::Vector3(static_cast<float>(grid.m_columns - 1) * grid.m_spacing.GetX(), 0.0f, maxHeight));
    }

    AZStd::vector<AZ::Vector3> BuildHeightfieldWireframe(const HeightfieldGrid& grid, AZ::u32 maxLinesPerAxis)
    {
        AZStd::vector<AZ::Vector3> lines;
        if (!grid.IsValid())
        {
            return lines;
        }

        const AZStd::vector<AZ::u32> columns = BuildDrawIndices(grid.m_columns, maxLinesPerAxis);
        const AZStd::vector<AZ::u32> rows = BuildDrawIndices(grid.m_rows, maxLinesPerAxis);

        // Every vertex is a real sample, so the wireframe sits on the surface even where
        // striding skips the detail between them.
        for (const AZ::u32 row : rows)
        {
            for (size_t i = 1; i < columns.size(); ++i)
            {
                lines.push_back(HeightfieldSamplePosition(grid, columns[i - 1], row));
                lines.push_back(HeightfieldSamplePosition(grid, columns[i], row));
            }
        }
        for (const AZ::u32 column : columns)
        {
            for (size_t i = 1; i < rows.size(); ++i)
            {
                lines.push_back(HeightfieldSamplePosition(grid, column, rows[i - 1]));
                lines.push_back(HeightfieldSamplePosition(grid, column, rows[i]));
            }
        }
        return lines;
    }
} // namespace JoltPhysics::EditorColliderGeometry
