#pragma once

#include <AzFramework/Physics/Character.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>

namespace JoltPhysics
{
    class JoltScene;

    //! Physics::Character implementation backed by JPH::CharacterVirtual.
    //! The character is not a Jolt body; it moves via collision queries in Move().
    //! An optional inner kinematic body (mirroring the character shape) makes the
    //! character visible to the rest of the simulation: dynamic bodies collide with
    //! it, and sensors fire trigger events for it.
    class JoltCharacter final : public Physics::Character
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltCharacter, AZ::SystemAllocator);
        AZ_RTTI(JoltCharacter, "{C7A11E50-1B2C-4D3E-9F00-A1B2C3D4E5F6}", Physics::Character);

        explicit JoltCharacter(const Physics::CharacterConfiguration& configuration);
        ~JoltCharacter() override = default;

        void CreateInScene(JoltScene* scene);

        //! Jolt body id of the inner kinematic body (invalid when none was created).
        JPH::BodyID GetInnerBodyId() const;

        // Physics::Character
        AZ::Vector3 GetBasePosition() const override;
        void SetBasePosition(const AZ::Vector3& position) override;
        void SetRotation(const AZ::Quaternion& rotation) override;
        AZ::Vector3 GetCenterPosition() const override;
        float GetStepHeight() const override;
        void SetStepHeight(float stepHeight) override;
        AZ::Vector3 GetUpDirection() const override;
        void SetUpDirection(const AZ::Vector3& upDirection) override;
        float GetSlopeLimitDegrees() const override;
        void SetSlopeLimitDegrees(float slopeLimitDegrees) override;
        float GetMaximumSpeed() const override;
        void SetMaximumSpeed(float maximumSpeed) override;
        AZ::Vector3 GetVelocity() const override;
        void SetCollisionLayer(const AzPhysics::CollisionLayer& layer) override;
        void SetCollisionGroup(const AzPhysics::CollisionGroup& group) override;
        AzPhysics::CollisionLayer GetCollisionLayer() const override;
        AzPhysics::CollisionGroup GetCollisionGroup() const override;
        AZ::Crc32 GetColliderTag() const override;
        void AddVelocityForTick(const AZ::Vector3& velocity) override;
        void AddVelocityForPhysicsTimestep(const AZ::Vector3& velocity) override;
        void ApplyRequestedVelocity(float deltaTime) override;
        void ResetRequestedVelocityForTick() override;
        void ResetRequestedVelocityForPhysicsTimestep() override;
        void Move(const AZ::Vector3& requestedMovement, float deltaTime) override;
        void AttachShape(AZStd::shared_ptr<Physics::Shape> shape) override;

        // AzPhysics::SimulatedBody
        AZ::EntityId GetEntityId() const override;
        AZ::Transform GetTransform() const override;
        void SetTransform(const AZ::Transform& transform) override;
        AZ::Vector3 GetPosition() const override;
        AZ::Quaternion GetOrientation() const override;
        AZ::Aabb GetAabb() const override;
        AzPhysics::SceneQueryHit RayCast(const AzPhysics::RayCastRequest& request) override;
        AZ::Crc32 GetNativeType() const override;
        void* GetNativePointer() const override;

        //! Ground state passthroughs (used by the component and gameplay code).
        bool IsOnGround() const;
        AZ::Vector3 GetGroundNormal() const;

    private:
        //! Signed distance from the character position (shape center) down to the
        //! shape's bottom, used for base<->center conversions.
        float GetBottomOffset() const;

        Physics::CharacterConfiguration m_configuration;

        AZStd::unique_ptr<JPH::CharacterVirtual> m_character;
        JPH::RefConst<JPH::Shape> m_shape;
        JoltScene* m_scene = nullptr;

        AZ::Quaternion m_orientation = AZ::Quaternion::CreateIdentity();
        AZ::Vector3 m_requestedVelocityForTick = AZ::Vector3::CreateZero();
        AZ::Vector3 m_requestedVelocityForPhysicsTimestep = AZ::Vector3::CreateZero();
        AZ::Vector3 m_observedVelocity = AZ::Vector3::CreateZero();
        float m_stepHeight = 0.5f;
        float m_slopeLimitDegrees = 30.0f;
        float m_maximumSpeed = 100.0f;
        AzPhysics::CollisionLayer m_collisionLayer;
        AzPhysics::CollisionGroup m_collisionGroup;
    };
} // namespace JoltPhysics
