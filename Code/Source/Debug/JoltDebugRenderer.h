#pragma once

#include <AzFramework/Physics/SystemBus.h>

#include <Jolt/Jolt.h>
#include <Jolt/Renderer/DebugRenderer.h>

namespace JoltPhysics
{
    //! JPH::DebugRenderer that forwards primitives to the callbacks of a
    //! Physics::DebugDrawSettings structure (Physics::SystemDebugRequestBus::DebugDrawPhysics).
    //! The SimulatedBody argument passed to the callbacks is always nullptr.
    class JoltDebugRenderer final : public JPH::DebugRenderer
    {
    public:
        explicit JoltDebugRenderer(const Physics::DebugDrawSettings* settings)
            : m_settings(settings)
        {
        }

        void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor) override;

        Batch CreateTriangleBatch([[maybe_unused]] const Triangle* inTriangles, [[maybe_unused]] int inTriangleCount) override
        {
            // Batched geometry rendering is not used; shapes draw via DrawLine/DrawTriangle.
            return {};
        }

        Batch CreateTriangleBatch(
            [[maybe_unused]] const Vertex* inVertices,
            [[maybe_unused]] int inVertexCount,
            [[maybe_unused]] const JPH::uint32* inIndices,
            [[maybe_unused]] int inIndexCount) override
        {
            return {};
        }

        void DrawTriangle(
            JPH::RVec3Arg inV1,
            JPH::RVec3Arg inV2,
            JPH::RVec3Arg inV3,
            JPH::ColorArg inColor,
            ECastShadow inCastShadow = ECastShadow::Off) override;

        void DrawGeometry(
            JPH::RMat44Arg inModelMatrix,
            const JPH::AABox& inWorldSpaceBounds,
            float inLODScaleSq,
            JPH::ColorArg inModelColor,
            const GeometryRef& inGeometry,
            ECullMode inCullMode = ECullMode::CullBackFace,
            ECastShadow inCastShadow = ECastShadow::On,
            EDrawMode inDrawMode = EDrawMode::Solid) override;

        void DrawText3D(
            JPH::RVec3Arg inPosition,
            const std::string_view& inString,
            JPH::ColorArg inColor = JPH::Color::sWhite,
            float inHeight = 0.5f) override;

    private:
        AZ::Color ToAzColor(JPH::ColorArg color) const;

        const Physics::DebugDrawSettings* m_settings = nullptr;
    };

} // namespace JoltPhysics
