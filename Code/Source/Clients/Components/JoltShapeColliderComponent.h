#pragma once

#include <AzCore/std/smart_ptr/shared_ptr.h>

#include <LmbrCentral/Shape/ShapeComponentBus.h>

#include <Clients/Components/JoltColliderComponentBase.h>

namespace JoltPhysics
{
    //! A collider whose geometry comes from an LmbrCentral shape component on the same
    //! entity, rather than from dimensions of its own.
    //!
    //! Mirrors PhysX's ShapeColliderComponent, which is the stock O3DE workflow for
    //! extruded level blockers, kill volumes and trigger regions: draw a Polygon Prism,
    //! add a collider, done. Without it those entities have no representation in this
    //! backend at all - the primitives can be re-authored by hand, but a prism cannot.
    //!
    //! Supported shape types: Box, Sphere, Capsule, Cylinder and Polygon Prism. A prism
    //! becomes the convex hull of its extruded outline; see DIVERGENCES for what that
    //! means for a concave one.
    class JoltShapeColliderComponent
        : public JoltColliderComponentBase
        , protected LmbrCentral::ShapeComponentNotificationsBus::Handler
    {
    public:
        AZ_COMPONENT(JoltShapeColliderComponent, "{5D3E9A7C-1F84-4B2E-9C6A-8D0B3E5F7A19}", JoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // JoltColliderComponentBase
        AzPhysics::ShapeColliderPair GetShapeColliderPair() const override;

        //! Builds a physics shape configuration from whatever shape component the entity
        //! carries. Null when there is none, or when its type is one this does not wrap.
        //!
        //! Static and entity-addressed so the editor component - a sibling, not a
        //! subclass - can call it too: both need the same answer, and one description of
        //! how a shape becomes geometry is worth more than saving an argument.
        static AZStd::shared_ptr<Physics::ShapeConfiguration> BuildShapeConfigurationForEntity(AZ::EntityId entityId);

    protected:
        void Activate() override;
        void Deactivate() override;

        // LmbrCentral::ShapeComponentNotificationsBus - the shape is the geometry, so a
        // change to it is a change to the collider.
        void OnShapeChanged(ShapeChangeReasons changeReasons) override;

    private:
        //! The extruded outline of a polygon prism, cooked as a convex hull.
        static AZStd::shared_ptr<Physics::ShapeConfiguration> BuildPolygonPrismConfiguration(AZ::EntityId entityId);
    };
} // namespace JoltPhysics
