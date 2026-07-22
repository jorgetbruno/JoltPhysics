#pragma once

#include <AzFramework/Physics/SystemBus.h>

#include <Jolt/Jolt.h>
#include <Jolt/Renderer/DebugRendererSimple.h>

namespace JoltPhysics
{
    //! JPH::DebugRendererSimple that forwards primitives to the callbacks of a
    //! Physics::DebugDrawSettings structure (Physics::SystemDebugRequestBus::DebugDrawPhysics).
    //! The SimulatedBody argument passed to the callbacks is always nullptr.
    class JoltDebugRenderer final : public JPH::DebugRendererSimple
    {
    public:
        explicit JoltDebugRenderer(const Physics::DebugDrawSettings* settings)
            : m_settings(settings)
        {
        }

        void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor) override;

        void DrawTriangle(
            JPH::RVec3Arg inV1,
            JPH::RVec3Arg inV2,
            JPH::RVec3Arg inV3,
            JPH::ColorArg inColor,
            ECastShadow inCastShadow = ECastShadow::Off) override;

        void DrawText3D(
            [[maybe_unused]] JPH::RVec3Arg inPosition,
            [[maybe_unused]] const std::string_view& inString,
            [[maybe_unused]] JPH::ColorArg inColor,
            [[maybe_unused]] float inHeight) override
        {
            // Text rendering is not supported by the O3DE debug draw callbacks.
        }

    private:
        AZ::Color ToAzColor(JPH::ColorArg color) const;

        const Physics::DebugDrawSettings* m_settings = nullptr;
    };

} // namespace JoltPhysics
