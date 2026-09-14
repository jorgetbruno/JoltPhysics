#pragma once

#include <AzCore/std/containers/vector.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>

#include <Editor/Components/EditorJoltDebugDrawUtils.h>

namespace JoltPhysics
{
    //! Records the line segments a draw helper emits, so the geometry can be asserted
    //! without a viewport. Everything else DebugDisplayRequests offers is left at its
    //! default no-op; the helpers under test only draw lines.
    class RecordingDebugDisplay : public AzFramework::DebugDisplayRequests
    {
    public:
        struct Segment
        {
            AZ::Vector3 m_from;
            AZ::Vector3 m_to;
            AZ::Vector4 m_color;
        };

        void DrawLine(
            const AZ::Vector3& p1, const AZ::Vector3& p2, const AZ::Vector4& col1,
            [[maybe_unused]] const AZ::Vector4& col2) override
        {
            m_segments.push_back({ p1, p2, col1 });
        }

        //! Overridden purely so the coloured overload above does not hide it; none of the
        //! helpers under test call this one.
        void DrawLine(const AZ::Vector3& p1, const AZ::Vector3& p2) override
        {
            m_segments.push_back({ p1, p2, EditorDebugDraw::WireColor });
        }

        //! Every distinct endpoint that was drawn.
        AZStd::vector<AZ::Vector3> Points() const
        {
            AZStd::vector<AZ::Vector3> points;
            points.reserve(m_segments.size() * 2);
            for (const Segment& segment : m_segments)
            {
                points.push_back(segment.m_from);
                points.push_back(segment.m_to);
            }
            return points;
        }

        //! Whether a segment from `from` to `to` was drawn, within tolerance.
        bool HasSegment(const AZ::Vector3& from, const AZ::Vector3& to, float tolerance = 1e-3f) const
        {
            for (const Segment& segment : m_segments)
            {
                if (segment.m_from.IsClose(from, tolerance) && segment.m_to.IsClose(to, tolerance))
                {
                    return true;
                }
            }
            return false;
        }

        AZStd::vector<Segment> m_segments;
    };
} // namespace JoltPhysics
