#pragma once

#include <Clients/Components/JoltColliderComponentBase.h>

#include <Shape/JoltCylinderShapeConfiguration.h>

namespace JoltPhysics
{
    //! Component that provides a cylinder shape collider for the Jolt physics backend.
    //! May be used in conjunction with a Jolt Rigid Body component to create a dynamic
    //! rigid body, or with a Jolt Static Rigid Body component to create a static one.
    //! Jolt has a native cylinder, so unlike the PhysX backend this is not approximated
    //! by a cooked convex hull.
    class JoltCylinderColliderComponent
        : public JoltColliderComponentBase
    {
    public:
        AZ_COMPONENT(JoltCylinderColliderComponent, "{D4E5F6A7-B8C9-4203-B6C7-D8E9F0A1B2C3}", JoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // JoltColliderComponentBase
        AzPhysics::ShapeColliderPair GetShapeColliderPair() const override;

        JoltCylinderShapeConfiguration& GetShapeConfiguration()
        {
            return *m_shapeConfiguration;
        }
        const JoltCylinderShapeConfiguration& GetShapeConfiguration() const
        {
            return *m_shapeConfiguration;
        }

    private:
        AZStd::shared_ptr<JoltCylinderShapeConfiguration> m_shapeConfiguration =
            AZStd::make_shared<JoltCylinderShapeConfiguration>();
    };
} // namespace JoltPhysics
