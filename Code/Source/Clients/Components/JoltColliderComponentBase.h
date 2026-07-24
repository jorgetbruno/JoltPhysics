#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>

namespace JoltPhysics
{
    //! Base class for the Jolt collider components.
    //! Owns the collider configuration; the derived classes own the shape configuration.
    class JoltColliderComponentBase
        : public AZ::Component
        , private AzFramework::EntityDebugDisplayEventBus::Handler
    {
    public:
        AZ_RTTI(JoltColliderComponentBase, "{B1C0AA01-7E4A-4B2C-9D3E-5F6A7B8C9D0E}", AZ::Component);

        static void Reflect(AZ::ReflectContext* context);

        //! Serialized identifier for the DPE inspector (empty on plain AZ::Component).
        AZStd::string GetSerializedIdentifier() const override;

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        //! Returns the collider/shape configuration pair used when creating simulated bodies.
        virtual AzPhysics::ShapeColliderPair GetShapeColliderPair() const = 0;

        //! Returns all collider/shape configuration pairs (a compound collider returns
        //! the pairs of all its child entities' colliders).
        virtual AzPhysics::ShapeColliderPairList GetShapeColliderPairs() const
        {
            return { GetShapeColliderPair() };
        }

        //! Mutable access to the collider configuration (offset, rotation, trigger, layer...).
        Physics::ColliderConfiguration& GetColliderConfiguration()
        {
            return *m_colliderConfiguration;
        }

    protected:
        // AZ::Component
        void OnAfterEntitySet() override;
        void Activate() override;
        void Deactivate() override;

        // AzFramework::EntityDebugDisplayEventBus
        // Draws a wireframe of the collider shape(s) in the editor viewport, based on
        // whatever GetShapeColliderPairs() returns - works for every derived collider type
        // without each one needing its own drawing code. Only primitive shapes (box, sphere,
        // capsule) are drawn; heightfield/mesh shapes are skipped (heightfields are usually
        // already visualized by their terrain provider, and mesh geometry isn't cheaply
        // available here without re-reading the cached native shape).
        void DisplayEntityViewport(const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

        AZStd::string m_serializedIdentifier;

        AZStd::shared_ptr<Physics::ColliderConfiguration> m_colliderConfiguration =
            AZStd::make_shared<Physics::ColliderConfiguration>();
    };
} // namespace JoltPhysics
