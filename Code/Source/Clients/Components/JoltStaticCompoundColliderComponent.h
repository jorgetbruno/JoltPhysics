#pragma once

#include <Clients/Components/JoltColliderComponentBase.h>

#include <AzCore/Component/TransformBus.h>

namespace JoltPhysics
{
    //! Collider component that combines the colliders of all child entities into a
    //! single compound collider (mirrors PhysX StaticCompoundColliderComponent).
    class JoltStaticCompoundColliderComponent : public JoltColliderComponentBase
    {
    public:
        AZ_COMPONENT(JoltStaticCompoundColliderComponent, "{C1D2E3F4-A5B6-47C8-9D0E-1F2A3B4C5D6E}", JoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // JoltColliderComponentBase
        AzPhysics::ShapeColliderPair GetShapeColliderPair() const override;
        AzPhysics::ShapeColliderPairList GetShapeColliderPairs() const override;

    protected:
        // AZ::Component
        void Activate() override;

        //! Collects the collider/shape pairs of all collider components on child entities.
        void GatherChildColliders();

        AzPhysics::ShapeColliderPairList m_childPairs;
    };

    //! Compound collider that supports adding and removing child collider entities at
    //! runtime (mirrors PhysX MutableCompoundColliderComponent).
    class JoltMutableCompoundColliderComponent
        : public JoltStaticCompoundColliderComponent
        , private AZ::TransformNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(JoltMutableCompoundColliderComponent, "{D2E3F4A5-B6C7-48D9-0E1F-2A3B4C5D6E7F}", JoltStaticCompoundColliderComponent);

        static void Reflect(AZ::ReflectContext* context);

    protected:
        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // AZ::TransformNotificationBus
        void OnChildAdded(AZ::EntityId child) override;
        void OnChildRemoved(AZ::EntityId child) override;
    };

} // namespace JoltPhysics
