#include <SoftBody/JoltSoftBodyRender.h>

#include <AzCore/Math/Color.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>

namespace JoltPhysics
{
    namespace
    {
        const AZ::Color SurfaceColor(0.85f, 0.45f, 0.25f, 0.35f);
        const AZ::Color WireColor(1.00f, 0.70f, 0.45f, 0.90f);
        const AZ::Color PreviewColor(0.85f, 0.45f, 0.25f, 0.65f);
    }

    void DrawSoftBody(
        AzFramework::DebugDisplayRequests& debugDisplay,
        const AZStd::vector<AZ::Vector3>& vertexPositions,
        const AZStd::vector<AZ::u32>& triangleIndices)
    {
        if (vertexPositions.empty() || triangleIndices.size() < 3)
        {
            return;
        }

        // The triangle list is cached per build, so a rebuild that shrank the body could
        // leave a caller holding indices past the end for a frame. Checking the largest
        // index once is cheaper than guarding every triangle, and an out-of-range list is
        // stale in its entirety rather than partly usable.
        const AZ::u32 vertexCount = static_cast<AZ::u32>(vertexPositions.size());
        for (const AZ::u32 index : triangleIndices)
        {
            if (index >= vertexCount)
            {
                return;
            }
        }

        // Translucent triangles have no depth sorting here, so depth writes stay off.
        debugDisplay.DepthWriteOff();
        debugDisplay.DrawTrianglesIndexed(vertexPositions, triangleIndices, SurfaceColor);

        // Wireframe over the top: it is what makes the deformation legible, since a flat
        // translucent fill gives no sense of how the surface is folding.
        debugDisplay.SetColor(WireColor);
        for (size_t i = 0; i + 2 < triangleIndices.size(); i += 3)
        {
            const AZ::Vector3& a = vertexPositions[triangleIndices[i]];
            const AZ::Vector3& b = vertexPositions[triangleIndices[i + 1]];
            const AZ::Vector3& c = vertexPositions[triangleIndices[i + 2]];
            debugDisplay.DrawLine(a, b);
            debugDisplay.DrawLine(b, c);
            debugDisplay.DrawLine(c, a);
        }

        debugDisplay.DepthWriteOn();
    }

    void DrawSoftBodyPreview(
        AzFramework::DebugDisplayRequests& debugDisplay,
        const AZ::Transform& worldTransform,
        JoltSoftBodyShape shape,
        const AZ::Vector3& size)
    {
        debugDisplay.PushMatrix(worldTransform);
        debugDisplay.SetColor(PreviewColor);

        switch (shape)
        {
        case JoltSoftBodyShape::Cloth:
            {
                // The sheet lies in the local XY plane, so an outline plus its diagonals
                // shows both the extent and which way it faces.
                const float halfX = size.GetX() * 0.5f;
                const float halfY = size.GetY() * 0.5f;
                const AZ::Vector3 corners[4] = {
                    AZ::Vector3(-halfX, -halfY, 0.0f),
                    AZ::Vector3(halfX, -halfY, 0.0f),
                    AZ::Vector3(halfX, halfY, 0.0f),
                    AZ::Vector3(-halfX, halfY, 0.0f),
                };
                for (int i = 0; i < 4; ++i)
                {
                    debugDisplay.DrawLine(corners[i], corners[(i + 1) % 4]);
                }
                debugDisplay.DrawLine(corners[0], corners[2]);
                debugDisplay.DrawLine(corners[1], corners[3]);
            }
            break;

        case JoltSoftBodyShape::Cube:
            {
                // Cube and Balloon are sized by the X extent alone, matching the builder.
                const AZ::Vector3 halfExtents(size.GetX() * 0.5f);
                debugDisplay.DrawWireBox(-halfExtents, halfExtents);
            }
            break;

        case JoltSoftBodyShape::Balloon:
            debugDisplay.DrawWireSphere(AZ::Vector3::CreateZero(), size.GetX() * 0.5f);
            break;
        }

        debugDisplay.PopMatrix();
    }
} // namespace JoltPhysics
