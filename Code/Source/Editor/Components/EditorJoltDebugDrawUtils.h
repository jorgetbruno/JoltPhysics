#pragma once

#include <Editor/Components/EditorJoltColliderDrawUtils.h>

#include <AzCore/Math/MathUtils.h>
#include <AzCore/Math/Quaternion.h>

namespace JoltPhysics::EditorDebugDraw
{
    using EditorColliderDraw::DrawWireArc;
    using EditorColliderDraw::DrawWireCircle;
    using EditorColliderDraw::DrawWireLine;
    using EditorColliderDraw::WireCircleSegments;
    using EditorColliderDraw::WireColor;

    //! Length of a drawn joint frame's axes, and of the cones that visualise
    //! angular limits, in metres. Joints have no size of their own, so this is a
    //! fixed readable length rather than anything derived from the scene.
    inline constexpr float JointAxisLength = 0.5f;

    inline const AZ::Vector4 AxisColorX(1.0f, 0.2f, 0.2f, 1.0f);
    inline const AZ::Vector4 AxisColorY(0.2f, 1.0f, 0.2f, 1.0f);
    inline const AZ::Vector4 AxisColorZ(0.2f, 0.4f, 1.0f, 1.0f);
    //! Limits are drawn amber so they read as annotation over the green wireframe.
    inline const AZ::Vector4 LimitColor(1.0f, 0.7f, 0.0f, 1.0f);
    //! Links to another entity (joint lead, follower) are drawn dimmer still.
    inline const AZ::Vector4 LinkColor(0.5f, 0.5f, 0.5f, 1.0f);

    //! The same axis colours as AZ::Color, for the manipulators the component modes
    //! build. Shared from here rather than restated per mode: the unity build folds the
    //! mode translation units together, where two anonymous-namespace copies collide.
    inline const AZ::Color ManipulatorAxisColorX(1.0f, 0.2f, 0.2f, 1.0f);
    inline const AZ::Color ManipulatorAxisColorY(0.2f, 1.0f, 0.2f, 1.0f);
    inline const AZ::Color ManipulatorAxisColorZ(0.2f, 0.4f, 1.0f, 1.0f);
    inline const AZ::Color ManipulatorSurfaceColor(1.0f, 1.0f, 1.0f, 0.5f);

    inline void DrawLine(
        AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Vector3& from, const AZ::Vector3& to,
        const AZ::Vector4& color)
    {
        debugDisplay.DrawLine(from, to, color, color);
    }

    //! Capsule aligned to local Z with `height` measured tip to tip, matching O3DE's
    //! capsule convention and EditorJoltCapsuleColliderComponent's own wireframe.
    inline void DrawWireCapsule(
        AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& transform, float radius, float height)
    {
        const float halfCylinder = AZ::GetMax(0.0f, height * 0.5f - radius);

        DrawWireCircle(debugDisplay, transform, radius, 2, halfCylinder);
        DrawWireCircle(debugDisplay, transform, radius, 2, -halfCylinder);
        for (bool useX : { true, false })
        {
            for (bool top : { true, false })
            {
                DrawWireArc(debugDisplay, transform, radius, halfCylinder, useX, top);
            }
        }
        // Vertical rails at the four compass points.
        for (const AZ::Vector3& across : { AZ::Vector3(radius, 0.0f, 0.0f), AZ::Vector3(-radius, 0.0f, 0.0f),
                                           AZ::Vector3(0.0f, radius, 0.0f), AZ::Vector3(0.0f, -radius, 0.0f) })
        {
            DrawWireLine(
                debugDisplay,
                transform.TransformPoint(across + AZ::Vector3(0.0f, 0.0f, -halfCylinder)),
                transform.TransformPoint(across + AZ::Vector3(0.0f, 0.0f, halfCylinder)));
        }
    }

    //! Three orthogonal great circles.
    inline void DrawWireSphere(
        AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& transform, float radius,
        const AZ::Vector4& color = WireColor)
    {
        for (AZ::u32 axis = 0; axis < 3; ++axis)
        {
            AZ::Vector3 previous = AZ::Vector3::CreateZero();
            for (int i = 0; i <= WireCircleSegments; ++i)
            {
                const float angle = AZ::Constants::TwoPi * aznumeric_cast<float>(i) / WireCircleSegments;
                const float c = radius * cosf(angle);
                const float s = radius * sinf(angle);
                AZ::Vector3 local;
                switch (axis)
                {
                case 0: local = AZ::Vector3(0.0f, c, s); break;
                case 1: local = AZ::Vector3(c, 0.0f, s); break;
                default: local = AZ::Vector3(c, s, 0.0f); break;
                }
                const AZ::Vector3 point = transform.TransformPoint(local);
                if (i > 0)
                {
                    DrawLine(debugDisplay, previous, point, color);
                }
                previous = point;
            }
        }
    }

    //! The joint frame. X is the primary axis (hinge / slider / twist / cone axis)
    //! and Y the normal or plane reference, matching the convention JoltJoint.cpp
    //! uses when it fills in mHingeAxis / mSliderAxis / mTwistAxis and mNormalAxis.
    inline void DrawJointFrame(
        AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& transform,
        float length = JointAxisLength)
    {
        const AZ::Vector3 origin = transform.GetTranslation();
        DrawLine(debugDisplay, origin, transform.TransformPoint(AZ::Vector3(length, 0.0f, 0.0f)), AxisColorX);
        DrawLine(debugDisplay, origin, transform.TransformPoint(AZ::Vector3(0.0f, length, 0.0f)), AxisColorY);
        DrawLine(debugDisplay, origin, transform.TransformPoint(AZ::Vector3(0.0f, 0.0f, length)), AxisColorZ);
    }

    //! Arc swept about the frame's X axis from `fromDegrees` to `toDegrees`, measured
    //! off +Y. Radial spokes mark the two ends. Used for hinge and twist limits.
    inline void DrawLimitArc(
        AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& transform, float radius,
        float fromDegrees, float toDegrees, const AZ::Vector4& color = LimitColor)
    {
        constexpr int ArcSegments = 32;
        const float from = AZ::DegToRad(fromDegrees);
        const float to = AZ::DegToRad(toDegrees);

        const AZ::Vector3 origin = transform.GetTranslation();
        AZ::Vector3 previous = AZ::Vector3::CreateZero();
        for (int i = 0; i <= ArcSegments; ++i)
        {
            const float angle = from + (to - from) * aznumeric_cast<float>(i) / ArcSegments;
            const AZ::Vector3 point =
                transform.TransformPoint(AZ::Vector3(0.0f, radius * cosf(angle), radius * sinf(angle)));
            if (i > 0)
            {
                DrawLine(debugDisplay, previous, point, color);
            }
            else
            {
                DrawLine(debugDisplay, origin, point, color);
            }
            previous = point;
        }
        DrawLine(debugDisplay, origin, previous, color);
    }

    //! Cone opening along +X with independent half-angles about the frame's Y and Z
    //! axes. Equal angles give a circular cone; the swing-twist and D6 joints limit
    //! swing with two different angles.
    inline void DrawLimitCone(
        AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& transform, float length,
        float halfAngleYDegrees, float halfAngleZDegrees, const AZ::Vector4& color = LimitColor)
    {
        constexpr int RimSegments = 32;
        constexpr int Spokes = 4;
        // Clamped below 90 degrees: at and beyond that the cone stops being a cone.
        const float tanY = tanf(AZ::DegToRad(AZ::GetClamp(halfAngleYDegrees, 0.0f, 89.0f)));
        const float tanZ = tanf(AZ::DegToRad(AZ::GetClamp(halfAngleZDegrees, 0.0f, 89.0f)));

        const AZ::Vector3 origin = transform.GetTranslation();
        AZ::Vector3 previous = AZ::Vector3::CreateZero();
        for (int i = 0; i <= RimSegments; ++i)
        {
            const float angle = AZ::Constants::TwoPi * aznumeric_cast<float>(i) / RimSegments;
            const AZ::Vector3 local =
                AZ::Vector3(1.0f, tanY * cosf(angle), tanZ * sinf(angle)).GetNormalized() * length;
            const AZ::Vector3 point = transform.TransformPoint(local);
            if (i > 0)
            {
                DrawLine(debugDisplay, previous, point, color);
            }
            if (i % (RimSegments / Spokes) == 0)
            {
                DrawLine(debugDisplay, origin, point, color);
            }
            previous = point;
        }
    }
} // namespace JoltPhysics::EditorDebugDraw
