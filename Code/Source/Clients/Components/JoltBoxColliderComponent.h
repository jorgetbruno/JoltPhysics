#pragma once

#include <Clients/Components/JoltColliderComponentBase.h>

#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    //! Component that provides a box shape collider for the Jolt physics backend.
    //! May be used in conjunction with a Jolt Rigid Body component to create a dynamic
    //! rigid body, or with a Jolt Static Rigid Body component to create a static one.
    class JoltBoxColliderComponent
        : public JoltColliderComponentBase
    {
    public:
        AZ_COMPONENT(JoltBoxColliderComponent, "{C2D1BB02-8F5B-4C3D-AE4F-6A7B8C9D0E1F}", JoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // JoltColliderComponentBase
        AzPhysics::ShapeColliderPair GetShapeColliderPair() const override;

        Physics::BoxShapeConfiguration& GetShapeConfiguration()
        {
            return *m_shapeConfiguration;
        }
        const Physics::BoxShapeConfiguration& GetShapeConfiguration() const
        {
            return *m_shapeConfiguration;
        }

    private:
        AZStd::shared_ptr<Physics::BoxShapeConfiguration> m_shapeConfiguration =
            AZStd::make_shared<Physics::BoxShapeConfiguration>();
    };
} // namespace JoltPhysics
