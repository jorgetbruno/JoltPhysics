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
    //! Edit-time collider wireframes are drawn by EditorJoltColliderComponentBase; this
    //! runtime component intentionally does not draw (it would otherwise render the
    //! wireframes during gameplay, since runtime components are only active in game mode).
    class JoltColliderComponentBase
        : public AZ::Component
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

        //! Returns all collider/shape configuration pairs, with the entity's overall
        //! scale applied (a compound collider returns its child entities' pairs, each
        //! already scaled by that child entity's own collider component).
        virtual AzPhysics::ShapeColliderPairList GetShapeColliderPairs() const;

        //! Applies the entity's overall scale (world uniform scale times any
        //! NonUniformScale component) to each pair: assigns it as the shape
        //! configuration's scale (read by JoltShapeUtils::CreateJoltShapeFromConfig)
        //! and scales a clone of the collider offset. Heightfields are left alone -
        //! Jolt cannot scale them (nor can PhysX).
        void ApplyOverallScale(AzPhysics::ShapeColliderPairList& pairs) const;

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

        AZStd::string m_serializedIdentifier;

        AZStd::shared_ptr<Physics::ColliderConfiguration> m_colliderConfiguration =
            AZStd::make_shared<Physics::ColliderConfiguration>();
    };
} // namespace JoltPhysics
