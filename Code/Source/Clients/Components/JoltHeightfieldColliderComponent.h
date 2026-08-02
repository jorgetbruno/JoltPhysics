#pragma once

#include <Clients/Components/JoltColliderComponentBase.h>

#include <AzFramework/Physics/HeightfieldProviderBus.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace JoltPhysics
{
    //! Collider component that turns heightfield data from a Physics::HeightfieldProviderBus
    //! implementation (e.g. a terrain gem) into a Jolt HeightFieldShape. Supports runtime
    //! height updates through HeightfieldProviderNotificationBus.
    class JoltHeightfieldColliderComponent
        : public JoltColliderComponentBase
        , private Physics::HeightfieldProviderNotificationBus::Handler
        , private AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(JoltHeightfieldColliderComponent, "{E5F6A7B8-C9D0-41E2-A3B4-C5D6E7F8A9B0}", JoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // JoltColliderComponentBase
        AzPhysics::ShapeColliderPair GetShapeColliderPair() const override;

    protected:
        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // Physics::HeightfieldProviderNotificationBus
        void OnHeightfieldDataChanged(
            const AZ::Aabb& dirtyRegion,
            Physics::HeightfieldProviderNotifications::HeightfieldChangeMask changeMask) override;

        // AZ::TickBus
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        //! Pushes provider height changes into the live Jolt shape.
        //! \param dirtyRegion the world-space region the provider says changed; when it is
        //!        valid only those samples are read and written, and only bodies over that
        //!        region are woken. An invalid (null) region means "look for yourself",
        //!        which is the polling path.
        void RefreshHeightsFromProvider(const AZ::Aabb& dirtyRegion);

    private:
        bool BuildHeightfieldShape();
        void UpdateHeightsFromProvider();

        AZStd::shared_ptr<Physics::HeightfieldShapeConfiguration> m_shapeConfiguration =
            AZStd::make_shared<Physics::HeightfieldShapeConfiguration>();
        JPH::RefConst<JPH::Shape> m_nativeShape;
        //! Mirror of the provider's grid, so a change can be located without asking for
        //! the whole thing again.
        AZStd::vector<float> m_lastHeights;

        //! Frames until the next unprompted poll. A provider that mutates without firing
        //! its notification is still caught, but a full-grid read and compare is far too
        //! expensive to run every frame - a 1024x1024 terrain is 4 MB of copy and compare
        //! for a grid that usually has not changed. The editor's wireframe throttles the
        //! same call for the same reason.
        int m_ticksUntilProviderPoll = 0;
        static constexpr int TicksBetweenProviderPolls = 15;
    };

} // namespace JoltPhysics
