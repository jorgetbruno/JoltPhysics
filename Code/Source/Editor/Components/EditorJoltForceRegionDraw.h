#pragma once

#include <Editor/Components/EditorJoltDebugDrawUtils.h>
#include <ForceRegion/JoltForceRegionForces.h>

#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>

namespace JoltPhysics::EditorDebugDraw
{
    //! Forces are drawn in cyan so they read as annotation against the green collider
    //! wireframe and the amber joint limits.
    inline const AZ::Vector4 ForceColor(0.2f, 0.9f, 1.0f, 1.0f);

    //! Length of a drawn force arrow, in metres. A force has no size of its own and its
    //! magnitude is in newtons, which has no honest mapping to metres - the direction is
    //! what an author needs to see in the viewport, and the magnitude is in the inspector.
    inline constexpr float ForceArrowLength = 1.0f;

    //! A line with a four-line head at `to`. The head scales with the shaft and is capped,
    //! so a long arrow does not grow a head the size of the region.
    inline void DrawArrow(
        AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Vector3& from, const AZ::Vector3& to,
        const AZ::Vector4& color)
    {
        const AZ::Vector3 shaft = to - from;
        const float length = shaft.GetLength();
        if (length <= 1e-4f)
        {
            return;
        }
        DrawLine(debugDisplay, from, to, color);

        const AZ::Vector3 direction = shaft / length;
        const float headLength = AZ::GetMin(0.25f * length, 0.2f);
        const float headWidth = headLength * 0.5f;
        const AZ::Vector3 side = direction.GetOrthogonalVector().GetNormalizedSafe();
        const AZ::Vector3 up = direction.Cross(side);
        const AZ::Vector3 base = to - direction * headLength;
        for (const AZ::Vector3& offset : { side * headWidth, -side * headWidth, up * headWidth, -up * headWidth })
        {
            DrawLine(debugDisplay, to, base + offset, color);
        }
    }

    //! The viewport preview of a force region: one arrow per force that has a direction
    //! of its own, from the region's origin.
    //!
    //! - A world-space force points along its axis whatever the region's rotation.
    //! - A local-space force turns with the region, exactly as the force it draws does.
    //! - A point force draws six radial arrows: outward for a repulsor, inward for a well,
    //!   because the sign of the magnitude is the whole difference and it is invisible in
    //!   a single arrow.
    //! - Drag and damping oppose a body's own velocity and have no direction until there
    //!   is a body; there is nothing truthful to draw for them, so nothing is drawn.
    //!
    //! Sign follows the force: a negative magnitude on a directional force reverses the
    //! arrow, as it reverses the push. A zero magnitude or a zero direction draws nothing,
    //! since the force does nothing.
    inline void DrawForceRegionPreview(
        AzFramework::DebugDisplayRequests& debugDisplay, const JoltForceRegion& forceRegion,
        const AZ::Transform& regionTransform)
    {
        const AZ::Vector3 origin = regionTransform.GetTranslation();

        auto drawDirectional = [&](const AZ::Vector3& worldDirection, float magnitude)
        {
            const AZ::Vector3 direction = worldDirection.GetNormalizedSafe();
            if (magnitude == 0.0f || direction.IsZero())
            {
                return;
            }
            const float sign = magnitude > 0.0f ? 1.0f : -1.0f;
            DrawArrow(debugDisplay, origin, origin + direction * (sign * ForceArrowLength), ForceColor);
        };

        for (const AZStd::shared_ptr<JoltForceRegionBaseForce>& force : forceRegion.m_forces)
        {
            if (!force)
            {
                continue;
            }

            if (const auto* worldForce = azrtti_cast<const JoltForceWorldSpace*>(force.get()))
            {
                drawDirectional(worldForce->m_direction, worldForce->m_magnitude);
            }
            else if (const auto* localForce = azrtti_cast<const JoltForceLocalSpace*>(force.get()))
            {
                // Rotation only, as JoltForceLocalSpace::CalculateForce applies it: the
                // region's scale stretches its collider, not the force.
                drawDirectional(
                    regionTransform.GetRotation().TransformVector(localForce->m_direction), localForce->m_magnitude);
            }
            else if (const auto* pointForce = azrtti_cast<const JoltForcePoint*>(force.get()))
            {
                if (pointForce->m_magnitude == 0.0f)
                {
                    continue;
                }
                const bool outward = pointForce->m_magnitude > 0.0f;
                for (const AZ::Vector3& axis :
                     { AZ::Vector3::CreateAxisX(), -AZ::Vector3::CreateAxisX(), AZ::Vector3::CreateAxisY(),
                       -AZ::Vector3::CreateAxisY(), AZ::Vector3::CreateAxisZ(), -AZ::Vector3::CreateAxisZ() })
                {
                    const AZ::Vector3 rim = origin + axis * ForceArrowLength;
                    if (outward)
                    {
                        DrawArrow(debugDisplay, origin, rim, ForceColor);
                    }
                    else
                    {
                        DrawArrow(debugDisplay, rim, origin, ForceColor);
                    }
                }
            }
            // JoltForceSimpleDrag and JoltForceLinearDamping: nothing to draw, by design.
        }
    }
} // namespace JoltPhysics::EditorDebugDraw
