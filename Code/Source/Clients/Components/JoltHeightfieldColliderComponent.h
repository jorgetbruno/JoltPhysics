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

    private:
        bool BuildHeightfieldShape();

        AZStd::shared_ptr<Physics::HeightfieldShapeConfiguration> m_shapeConfiguration =
            AZStd::make_shared<Physics::HeightfieldShapeConfiguration>();
        JPH::RefConst<JPH::Shape> m_nativeShape;
    };

} // namespace JoltPhysics
