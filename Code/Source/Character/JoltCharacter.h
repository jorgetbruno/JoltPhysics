#pragma once

#include <AzFramework/Physics/Character.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Character/Character.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>

namespace JoltPhysics
{
    class JoltScene;

    //! Jolt character configuration extended with the backend choice. Passed by the
    //! character controller component; JoltCharacter reads the flag via RTTI.
    class JoltCharacterConfiguration : public Physics::CharacterConfiguration
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltCharacterConfiguration, AZ::SystemAllocator);
        AZ_RTTI(JoltCharacterConfiguration, "{6E9A2B4C-7D1F-4E30-9A2B-5C6D7E8F90A1}", Physics::CharacterConfiguration);

        //! When true the character is backed by a real rigid body in the simulation
        //! (JPH::Character): cheaper, and most accurate collision response with dynamic
        //! bodies. When false the character is a JPH::CharacterVirtual (updated by queries).
        bool m_rigidBodyCharacter = false;
    };

    //! Physics::Character implementation backed by either JPH::CharacterVirtual (the
    //! default) or JPH::Character (a real rigid body, when the configuration requests it).
    //! For the virtual backend the character is not a Jolt body; it moves via collision
    //! queries in Move(), and an optional inner kinematic body makes it visible to the
    //! simulation. For the rigid-body backend the character IS a body: it moves during
    //! the physics step and its ground state is refreshed by PostSimulation().
    class JoltCharacter final : public Physics::Character
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltCharacter, AZ::SystemAllocator);
        AZ_RTTI(JoltCharacter, "{C7A11E50-1B2C-4D3E-9F00-A1B2C3D4E5F6}", Physics::Character);

        explicit JoltCharacter(const Physics::CharacterConfiguration& configuration);
        ~JoltCharacter() override;

        void CreateInScene(JoltScene* scene);

        //! Removes the character's body from the physics system (rigid-body backend only;
        //! the virtual character's inner body is cleaned up by its own destructor).
        void RemoveFromScene();

        //! Refreshes the ground state after the physics step (rigid-body backend only).
        void PostSimulation();

        //! True when this character is backed by a real rigid body (JPH::Character).
        bool IsRigidBodyCharacter() const { return m_rigidBodyCharacter; }

        //! Jolt body id of the character's body: the rigid-body character's own body, or
        //! the virtual character's inner kinematic body (invalid when neither exists).
        JPH::BodyID GetInnerBodyId() const;

        //! Writes/reads the native character's runtime state (position, velocities and
        //! the cached ground) through a Jolt state recorder. A CharacterVirtual lives
        //! outside the body system, so PhysicsSystem::SaveState does not cover it - a
        //! scene snapshot has to include the characters explicitly, which is what
        //! JoltScene::SaveSimulationState does with these.
        void SaveNativeState(JPH::StateRecorder& recorder) const;
        void RestoreNativeState(JPH::StateRecorder& recorder);

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

        //! Shapes attached to the character beyond its own, in the order they arrived.
        //! Exposed so the component can tell whether anything is attached at all.
        const AZStd::vector<AZStd::shared_ptr<Physics::Shape>>& GetAttachedShapes() const
        {
            return m_attachedShapes;
        }

        //! Keeps the attachment body under the character. Called after the character has
        //! moved; deltaTime > 0 drives it there as a kinematic body (so it pushes what it
        //! touches), and 0 teleports it, which is what an explicit position set means.
        void SyncAttachmentBody(float deltaTime);

        //! Ground state passthroughs (used by the component and gameplay code).
        bool IsOnGround() const;
        AZ::Vector3 GetGroundNormal() const;

    private:
        //! Signed distance from the character position (shape center) down to the
        //! shape's bottom, used for base<->center conversions.
        float GetBottomOffset() const;

        //! Converts a base (feet) position to the shape centre Jolt positions bodies by.
        //!
        //! O3DE places a character by its base, not its centre: Physics::Character makes
        //! SetBasePosition writable while GetCenterPosition is read-only, and the PhysX
        //! backend maps the entity transform onto PxController's foot position. Jolt has
        //! no such notion - a CharacterVirtual sits at its shape centre - so every
        //! entity-facing position crosses this conversion.
        AZ::Vector3 BaseToCenter(const AZ::Vector3& basePosition) const;

        //! Adds the colliders the configuration arrived with, once there is a scene to
        //! put them in.
        void AttachConfiguredColliders();

        //! Rebuilds the attachment body from m_attachedShapes: creates it on the first
        //! shape, re-shapes it on later ones, and removes it when the last one goes.
        void RefreshAttachmentBody();

        void DestroyAttachmentBody();

        //! The attached shapes as one native shape - the shape itself when there is only
        //! one at the origin, a static compound otherwise.
        JPH::RefConst<JPH::Shape> BuildAttachmentShape() const;

        Physics::CharacterConfiguration m_configuration;
        bool m_rigidBodyCharacter = false;

        AZStd::unique_ptr<JPH::CharacterVirtual> m_character; //!< Virtual backend.
        JPH::Ref<JPH::Character> m_rigidBody;                 //!< Rigid-body backend.
        JPH::RefConst<JPH::Shape> m_shape;

        //! The object layer resolved from the configured collision layer and group.
        //! CharacterVirtual sweeps its own movement rather than going through the
        //! simulation, so this has to be handed to ExtendedUpdate as a filter or the
        //! character's collision layer would only ever affect its inner body.
        JPH::ObjectLayer m_objectLayer = 0;
        JoltScene* m_scene = nullptr;

        //! Colliders attached to the character, and the kinematic body that carries them.
        //!
        //! They are deliberately not part of the character's own shape: that shape is what
        //! the character sweeps its movement against, so folding a weapon or a hitbox into
        //! it would have the character refuse to walk through gaps its body fits. PhysX
        //! keeps them apart the same way, on what it calls a shadow body.
        //!
        //! Created on demand rather than always, unlike PhysX: a character with nothing
        //! attached is the common case and it costs a body, a shape and a sync per step.
        AZStd::vector<AZStd::shared_ptr<Physics::Shape>> m_attachedShapes;
        JPH::BodyID m_attachmentBodyId;

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
