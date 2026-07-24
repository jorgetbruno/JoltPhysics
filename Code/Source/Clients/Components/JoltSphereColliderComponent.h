#pragma once

#include <Clients/Components/JoltColliderComponentBase.h>

#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    //! Component that provides a sphere shape collider for the Jolt physics backend.
    //! May be used in conjunction with a Jolt Rigid Body component to create a dynamic
    //! rigid body, or with a Jolt Static Rigid Body component to create a static one.
    class JoltSphereColliderComponent
        : public JoltColliderComponentBase
    {
    public:
        AZ_COMPONENT(JoltSphereColliderComponent, "{D3E2CC03-9A6C-4D4E-BF5A-7B8C9D0E1F2A}", JoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // JoltColliderComponentBase
        AzPhysics::ShapeColliderPair GetShapeColliderPair() const override;

        Physics::SphereShapeConfiguration& GetShapeConfiguration()
        {
            return *m_shapeConfiguration;
        }
        const Physics::SphereShapeConfiguration& GetShapeConfiguration() const
        {
            return *m_shapeConfiguration;
        }

    private:
        AZStd::shared_ptr<Physics::SphereShapeConfiguration> m_shapeConfiguration =
            AZStd::make_shared<Physics::SphereShapeConfiguration>();
    };
} // namespace JoltPhysics
