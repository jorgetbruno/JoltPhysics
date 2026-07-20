#pragma once

#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>

namespace JoltPhysics
{
    class JoltScene;

    class JoltRigidBody : public AzPhysics::RigidBody
    {
    public:
        AZ_CLASS_ALLOCATOR_DECL
        AZ_RTTI(JoltRigidBody, "{A1B2C3D4-E5F6-7890-ABCD-EF1234567890}", AzPhysics::RigidBody)

        JoltRigidBody() = default;
        explicit JoltRigidBody(const AzPhysics::RigidBodyConfiguration& configuration);
        ~JoltRigidBody() override;

        void CreateInScene(JoltScene* scene);
        void SyncTransform();

        const JPH::BodyID& GetBodyId() const { return m_bodyId; }
        bool IsSensor() const { return m_isSensor; }

        AZ::Vector3 GetPosition() const override;
        AZ::Quaternion GetOrientation() const override;
        AZ::Aabb GetAabb() const override;
        AZ::EntityId GetEntityId() const override;

        AZ::Transform GetTransform() const override;

        void SetTransform(const AZ::Transform& transform) override;
        void SetKinematic(bool isKinematic) override;
        bool IsKinematic() const override;
        void SetKinematicTarget(const AZ::Transform& targetPosition) override;
        bool IsGravityEnabled() const override;
        void SetGravityEnabled(bool enabled) override;
        void SetSimulationEnabled(bool enabled) override;
        void SetCCDEnabled(bool enabled) override;

        AZ::Vector3 GetLinearVelocity() const override;
        void SetLinearVelocity(const AZ::Vector3& velocity) override;
        AZ::Vector3 GetAngularVelocity() const override;
        void SetAngularVelocity(const AZ::Vector3& angularVelocity) override;

        AZ::Vector3 GetLinearVelocityAtWorldPoint(const AZ::Vector3& worldPoint) const override;

        void ApplyLinearImpulse(const AZ::Vector3& impulse) override;
        void ApplyLinearImpulseAtWorldPoint(const AZ::Vector3& impulse, const AZ::Vector3& worldPoint) override;
        void ApplyAngularImpulse(const AZ::Vector3& angularImpulse) override;

        float GetMass() const override;
        float GetInverseMass() const override;
        void SetMass(float mass) override;
        void SetCenterOfMassOffset(const AZ::Vector3& comOffset) override;
        AZ::Matrix3x3 GetInertiaLocal() const override;
        AZ::Matrix3x3 GetInertiaWorld() const override;
        AZ::Matrix3x3 GetInverseInertiaLocal() const override;
        AZ::Matrix3x3 GetInverseInertiaWorld() const override;

        void SetLinearDamping(float damping) override;
        float GetLinearDamping() const override;
        void SetAngularDamping(float damping) override;
        float GetAngularDamping() const override;

        bool IsAwake() const override;
        void ForceAsleep() override;
        void ForceAwake() override;

        float GetSleepThreshold() const override;
        void SetSleepThreshold(float threshold) override;

        AZ::Vector3 GetCenterOfMassWorld() const override;
        AZ::Vector3 GetCenterOfMassLocal() const override;

        void AddShape(AZStd::shared_ptr<Physics::Shape> shape) override;
        void RemoveShape(AZStd::shared_ptr<Physics::Shape> shape) override;

        void UpdateMassProperties(AzPhysics::MassComputeFlags flags = AzPhysics::MassComputeFlags::DEFAULT,
            const AZ::Vector3& centerOfMassOffsetOverride = AZ::Vector3::CreateZero(),
            const AZ::Matrix3x3& inertiaTensorOverride = AZ::Matrix3x3::CreateIdentity(),
            const float massOverride = 1.0f) override;

        AZ::Crc32 GetNativeType() const override;
        void* GetNativePointer() const override;
        AzPhysics::SceneQueryHit RayCast(const AzPhysics::RayCastRequest& request) override;

    private:
        AzPhysics::RigidBodyConfiguration m_configuration;
        JoltScene* m_scene = nullptr;
        JPH::BodyID m_bodyId;
        AZ::EntityId m_entityId;
        bool m_isKinematic = false;
        bool m_isSensor = false;
    };

} // namespace JoltPhysics
