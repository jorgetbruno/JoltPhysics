#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/Math/Vector4.h>
#include <AzCore/Math/Transform.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>

namespace JoltPhysics::EditorColliderDraw
{
    inline constexpr int WireCircleSegments = 32;
    inline const AZ::Vector4 WireColor(0.0f, 1.0f, 0.0f, 1.0f);

    inline void DrawWireLine(
        AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Vector3& from, const AZ::Vector3& to)
    {
        debugDisplay.DrawLine(from, to, WireColor, WireColor);
    }

    //! Draws a circle of the given radius in the plane perpendicular to `axis`
    //! (0 = X, 1 = Y, 2 = Z), offset along that axis, in local space transformed
    //! by `transform`.
    inline void DrawWireCircle(
        AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& transform,
        float radius, AZ::u32 axis, float axisOffset = 0.0f)
    {
        AZ::Vector3 previous = AZ::Vector3::CreateZero();
        for (int i = 0; i <= WireCircleSegments; ++i)
        {
            const float angle = AZ::Constants::TwoPi * aznumeric_cast<float>(i) / WireCircleSegments;
            const float c = radius * cosf(angle);
            const float s = radius * sinf(angle);
            AZ::Vector3 point;
            switch (axis)
            {
            case 0: point = AZ::Vector3(axisOffset, c, s); break;
            case 1: point = AZ::Vector3(c, axisOffset, s); break;
            default: point = AZ::Vector3(c, s, axisOffset); break;
            }
            point = transform.TransformPoint(point);
            if (i > 0)
            {
                DrawWireLine(debugDisplay, previous, point);
            }
            previous = point;
        }
    }

    //! Draws a half-circle arc in a vertical plane through the capsule axis (Z in
    //! shape-local space). `useX` selects the XZ plane (otherwise YZ), `top` the
    //! upper cap, `halfCylinder` the cylindrical half-height.
    inline void DrawWireArc(
        AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& transform,
        float radius, float halfCylinder, bool useX, bool top)
    {
        constexpr int ArcSegments = 16;
        AZ::Vector3 previous = AZ::Vector3::CreateZero();
        for (int i = 0; i <= ArcSegments; ++i)
        {
            const float angle = AZ::Constants::Pi * aznumeric_cast<float>(i) / ArcSegments;
            const float across = radius * cosf(angle);
            const float up = top ? halfCylinder + radius * sinf(angle) : -halfCylinder - radius * sinf(angle);
            const AZ::Vector3 local = useX ? AZ::Vector3(across, 0.0f, up) : AZ::Vector3(0.0f, across, up);
            const AZ::Vector3 point = transform.TransformPoint(local);
            if (i > 0)
            {
                DrawWireLine(debugDisplay, previous, point);
            }
            previous = point;
        }
    }
} // namespace JoltPhysics::EditorColliderDraw
