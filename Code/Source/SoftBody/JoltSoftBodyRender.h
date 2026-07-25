#pragma once

#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>

#include <JoltPhysics/JoltSoftBodyBus.h>

namespace AzFramework
{
    class DebugDisplayRequests;
}

namespace JoltPhysics
{
    //! Draws a simulated soft body from its particle positions and triangle list.
    //!
    //! A soft body has no mesh asset and its shape changes every step, so this is the only
    //! way to see one. Positions are already in world space, so no transform is pushed.
    void DrawSoftBody(
        AzFramework::DebugDisplayRequests& debugDisplay,
        const AZStd::vector<AZ::Vector3>& vertexPositions,
        const AZStd::vector<AZ::u32>& triangleIndices);

    //! Draws the rest shape a soft body would be built with, for the editor viewport where
    //! nothing is simulating. Generated from the settings rather than from a live body, so
    //! it shows the effect of changing resolution or size before pressing play.
    void DrawSoftBodyPreview(
        AzFramework::DebugDisplayRequests& debugDisplay,
        const AZ::Transform& worldTransform,
        JoltSoftBodyShape shape,
        const AZ::Vector3& size);
} // namespace JoltPhysics
