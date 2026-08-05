#pragma once

#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/TransformBus.h>

#include <AzFramework/Physics/ColliderComponentBus.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>
#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>

namespace JoltPhysics
{
    //! Component used to register an entity as a dynamic rigid body in the Jolt simulation.
    class JoltRigidBodyComponent
        : public AZ::Component
        , public AZ::EntityBus::Handler
        , public Physics::RigidBodyRequestBus::Handler
        , public AzPhysics::SimulatedBodyComponentRequestsBus::Handler
        , public AZ::TickBus::Handler
        , private AZ::TransformNotificationBus::Handler
        , private Physics::ColliderComponentEventBus::Handler
    {
    public:
        AZ_COMPONENT(JoltRigidBodyComponent, "{F5A4EE05-BC8E-4F6A-DB7C-9D0E1F2A3B4C}");

        static void Reflect(AZ::ReflectContext* context);

        //! Serialized identifier for the DPE inspector (empty on plain AZ::Component).
        AZStd::string GetSerializedIdentifier() const override;

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // Physics::RigidBodyRequestBus + AzPhysics::SimulatedBodyComponentRequestsBus
        void EnablePhysics() override;
        void DisablePhysics() override;
        bool IsPhysicsEnabled() const override;

        AzPhysics::SceneQueryHit RayCast(const AzPhysics::RayCastRequest& request) override;
        AZ::Aabb GetAabb() const override;

        // Physics::RigidBodyRequestBus
        AZ::Vector3 GetCenterOfMassWorld() const override;
        AZ::Vector3 GetCenterOfMassLocal() const override;

        AZ::Matrix3x3 GetInertiaWorld() const override;
        AZ::Matrix3x3 GetInertiaLocal() const override;
        AZ::Matrix3x3 GetInverseInertiaWorld() const override;
        AZ::Matrix3x3 GetInverseInertiaLocal() const override;

        float GetMass() const override;
        float GetInverseMass() const override;
        void SetMass(float mass) override;
        void SetCenterOfMassOffset(const AZ::Vector3& comOffset) override;

        AZ::Vector3 GetLinearVelocity() const override;
        void SetLinearVelocity(const AZ::Vector3& velocity) override;
        AZ::Vector3 GetAngularVelocity() const override;
        void SetAngularVelocity(const AZ::Vector3& angularVelocity) override;
        AZ::Vector3 GetLinearVelocityAtWorldPoint(const AZ::Vector3& worldPoint) const override;
        void ApplyLinearImpulse(const AZ::Vector3& impulse) override;
        void ApplyLinearImpulseAtWorldPoint(const AZ::Vector3& impulse, const AZ::Vector3& worldPoint) override;
        void ApplyAngularImpulse(const AZ::Vector3& angularImpulse) override;

        float GetLinearDamping() const override;
        void SetLinearDamping(float damping) override;
        float GetAngularDamping() const override;
        void SetAngularDamping(float damping) override;

        bool IsAwake() const override;
        void ForceAsleep() override;
        void ForceAwake() override;

        bool IsKinematic() const override;
        void SetKinematic(bool kinematic) override;
        void SetKinematicTarget(const AZ::Transform& targetPosition) override;

        bool IsGravityEnabled() const override;
        void SetGravityEnabled(bool enabled) override;
        void SetSimulationEnabled(bool enabled) override;

        float GetSleepThreshold() const override;
        void SetSleepThreshold(float threshold) override;
        AzPhysics::RigidBody* GetRigidBody() override;

        // AzPhysics::SimulatedBodyComponentRequestsBus
        AzPhysics::SimulatedBody* GetSimulatedBody() override;
        AzPhysics::SimulatedBodyHandle GetSimulatedBodyHandle() const override;

        AzPhysics::RigidBodyConfiguration& GetConfiguration()
        {
            return m_configuration;
        }

    protected:
        // AZ::Component
        void OnAfterEntitySet() override;
        void Activate() override;
        void Deactivate() override;

        // AZ::EntityBus
        void OnEntityActivated(const AZ::EntityId& entityId) override;

        // AZ::TickBus
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        int GetTickOrder() override;

        // AZ::TransformNotificationBus
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        // Physics::ColliderComponentEventBus
        void OnColliderChanged() override;

    private:
        void TryCreateRigidBody();
        void CreateRigidBody();
        void DestroyRigidBody();
        void RebuildRigidBody();

        const AzPhysics::RigidBody* GetRigidBodyConst() const;

        AZStd::string m_serializedIdentifier;

        AzPhysics::RigidBodyConfiguration m_configuration;
        AzPhysics::SimulatedBodyHandle m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        AzPhysics::SceneHandle m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;

        bool m_syncingTransformFromBody = false; //!< True while the entity transform is being updated from the physics body.
        //! The last pose pushed to the entity. A sleeping body whose pose still matches
        //! this needs no write, which is what lets a scene full of settled bodies cost
        //! nothing to sync; a pose that differs while asleep (a restored snapshot, a
        //! teleport) still gets through.
        AZ::Transform m_lastSyncedTransform = AZ::Transform::CreateIdentity();

        //! Whether the entity's transform should be blended between the last two steps
        //! rather than snapped to the newest, and the pose pair that makes that possible.
        bool ShouldInterpolate(const AzPhysics::RigidBody& body) const;
        AZ::Transform InterpolatedTransform() const;

        //! Forgets the pose pair, so the next frame starts fresh from this pose. Called
        //! when something moves the body outside the simulation.
        void ResetPoseHistory(const AZ::Transform& transform);

        //! Shifts the pose pair on, once per fixed step.
        void RecordStepPose();

        //! The poses the last two physics steps left, in that order. Rendering a frame
        //! between two steps means drawing somewhere between these, and drawing the newest
        //! on every frame is what makes a body advance in a staircase at frame rates above
        //! the physics rate.
        AZ::Transform m_previousBodyTransform = AZ::Transform::CreateIdentity();
        AZ::Transform m_previousBodyTransform2 = AZ::Transform::CreateIdentity();
        bool m_hasPoseHistory = false; //!< One pose recorded.
        bool m_hasPosePair = false;    //!< Two, which is the minimum to blend between.
        //! Records a pose per fixed step. Only connected while interpolating.
        AzPhysics::SceneEvents::OnSceneSimulationFinishHandler m_sceneFinishHandler;
        bool m_rebuildPending = false; //!< True when the collider set changed and the body must be rebuilt on the next tick.
    };
} // namespace JoltPhysics
