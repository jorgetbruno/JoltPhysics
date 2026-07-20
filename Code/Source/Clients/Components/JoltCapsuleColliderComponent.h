#pragma once

#include <Clients/Components/JoltColliderComponentBase.h>

#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    //! Component that provides a capsule shape collider for the Jolt physics backend.
    //! May be used in conjunction with a Jolt Rigid Body component to create a dynamic
    //! rigid body, or with a Jolt Static Rigid Body component to create a static one.
    class JoltCapsuleColliderComponent
        : public JoltColliderComponentBase
    {
    public:
        AZ_COMPONENT(JoltCapsuleColliderComponent, "{E4F3DD04-AB7D-4E5F-CA6B-8C9D0E1F2A3B}", JoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // JoltColliderComponentBase
        AzPhysics::ShapeColliderPair GetShapeColliderPair() const override;

    private:
        AZStd::shared_ptr<Physics::CapsuleShapeConfiguration> m_shapeConfiguration =
            AZStd::make_shared<Physics::CapsuleShapeConfiguration>();
    };
} // namespace JoltPhysics
