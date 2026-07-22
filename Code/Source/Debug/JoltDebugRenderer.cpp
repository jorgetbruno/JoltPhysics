#include <Debug/JoltDebugRenderer.h>

#include <Utils/Conversions.h>

namespace JoltPhysics
{
    AZ::Color JoltDebugRenderer::ToAzColor(JPH::ColorArg color) const
    {
        return AZ::Color(
            color.r / 255.0f,
            color.g / 255.0f,
            color.b / 255.0f,
            color.a / 255.0f);
    }

    void JoltDebugRenderer::DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor)
    {
        if (!m_settings || !m_settings->m_drawLineCB)
        {
            return;
        }

        const AZ::Color color = ToAzColor(inColor);
        // DebugDrawSettings' draw helpers are non-const; the settings are not modified.
        const_cast<Physics::DebugDrawSettings*>(m_settings)->DrawLine(
            Physics::DebugDrawVertex(Conversions::FromJolt(inFrom), color),
            Physics::DebugDrawVertex(Conversions::FromJolt(inTo), color),
            nullptr,
            1.0f);
    }

    void JoltDebugRenderer::DrawTriangle(
        JPH::RVec3Arg inV1,
        JPH::RVec3Arg inV2,
        JPH::RVec3Arg inV3,
        JPH::ColorArg inColor,
        [[maybe_unused]] ECastShadow inCastShadow)
    {
        if (!m_settings || !m_settings->m_drawTriBatchCB)
        {
            return;
        }

        const AZ::Color color = ToAzColor(inColor);
        const Physics::DebugDrawVertex vertices[] = {
            Physics::DebugDrawVertex(Conversions::FromJolt(inV1), color),
            Physics::DebugDrawVertex(Conversions::FromJolt(inV2), color),
            Physics::DebugDrawVertex(Conversions::FromJolt(inV3), color),
        };
        const AZ::u32 indices[] = { 0, 1, 2 };

        const_cast<Physics::DebugDrawSettings*>(m_settings)->DrawTriangleBatch(vertices, 3, indices, 3, nullptr);
    }

} // namespace JoltPhysics
