#pragma once

#include <Clients/Components/JoltColliderComponentBase.h>

#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    //! Component that provides a triangle-mesh or convex-hull collider for the Jolt
    //! physics backend, built from cooked mesh data baked in the editor (see
    //! EditorJoltMeshColliderComponent, which bakes the entity's render geometry).
    //! May be used with a Jolt Static Rigid Body component for world collision, or -
    //! convex hulls only - with a Jolt Rigid Body component for a dynamic body.
    class JoltMeshColliderComponent
        : public JoltColliderComponentBase
    {
    public:
        AZ_COMPONENT(JoltMeshColliderComponent, "{E5F6A7B8-C9D0-4E1F-A2B3-C4D5E6F7A8B9}", JoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // JoltColliderComponentBase
        AzPhysics::ShapeColliderPair GetShapeColliderPair() const override;

        Physics::CookedMeshShapeConfiguration& GetShapeConfiguration()
        {
            return *m_shapeConfiguration;
        }
        const Physics::CookedMeshShapeConfiguration& GetShapeConfiguration() const
        {
            return *m_shapeConfiguration;
        }

    private:
        AZStd::shared_ptr<Physics::CookedMeshShapeConfiguration> m_shapeConfiguration =
            AZStd::make_shared<Physics::CookedMeshShapeConfiguration>();
    };
} // namespace JoltPhysics
