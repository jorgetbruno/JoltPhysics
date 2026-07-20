#pragma once

#include <AzFramework/Physics/Configuration/CollisionConfiguration.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

namespace JoltPhysics
{
    namespace BroadPhaseLayers
    {
        static constexpr JPH::BroadPhaseLayer NonMoving(0);
        static constexpr JPH::BroadPhaseLayer Moving(1);
        static constexpr JPH::uint NumLayers = 2;
    }

    namespace ObjectLayers
    {
        static constexpr JPH::ObjectLayer NonMoving = 0;
        static constexpr JPH::ObjectLayer Moving = 1;
        static constexpr JPH::ObjectLayer NumLayers = 2;
    }

    class BroadPhaseLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
    {
    public:
        BroadPhaseLayerInterfaceImpl() = default;

        void Initialize();

        JPH::uint GetNumBroadPhaseLayers() const override;
        JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override;

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
        const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override;
#endif

    private:
        JPH::BroadPhaseLayer m_objectToBroadPhase[ObjectLayers::NumLayers];
    };

    class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter
    {
    public:
        ObjectVsBroadPhaseLayerFilterImpl() = default;

        void Initialize();

        bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override;
    };

    class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter
    {
    public:
        ObjectLayerPairFilterImpl() = default;

        void Initialize();

        bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override;
    };

} // namespace JoltPhysics
