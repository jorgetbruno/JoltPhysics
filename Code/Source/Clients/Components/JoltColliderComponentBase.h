#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>

#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>

namespace JoltPhysics
{
    //! Base class for the Jolt collider components.
    //! Owns the collider configuration; the derived classes own the shape configuration.
    class JoltColliderComponentBase
        : public AZ::Component
    {
    public:
        AZ_RTTI(JoltColliderComponentBase, "{B1C0AA01-7E4A-4B2C-9D3E-5F6A7B8C9D0E}", AZ::Component);

        static void Reflect(AZ::ReflectContext* context);

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
        void Activate() override;
        void Deactivate() override;

        AZStd::shared_ptr<Physics::ColliderConfiguration> m_colliderConfiguration =
            AZStd::make_shared<Physics::ColliderConfiguration>();
    };
} // namespace JoltPhysics
