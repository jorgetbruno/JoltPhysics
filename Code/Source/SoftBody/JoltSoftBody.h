#pragma once

#include <AzCore/Math/Aabb.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/RTTI/TypeInfo.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/parallel/mutex.h>

#include <AzFramework/Physics/Collision/CollisionGroups.h>
#include <AzFramework/Physics/Collision/CollisionLayers.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <AzFramework/Physics/Configuration/SimulatedBodyConfiguration.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Collision/CollisionGroup.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>

#include <JoltPhysics/JoltSoftBodyBus.h>
#include <JoltPhysics/Pipeline/JoltMeshAsset.h>

namespace JPH
{
    class PhysicsSystem;
}

namespace JoltPhysics
{
    class JoltScene;

    //! Everything describing a soft body. The fields split into two kinds, and the split
    //! matters: those marked "baked" go into the particle layout when the body is built
    //! and cannot be changed on a live body, while the rest are forwarded straight to
    //! Jolt's motion properties and can change every frame.
    struct JoltSoftBodySettings
    {
        //! Edit-context helpers: a soft body's grid shows every field for every shape, so
        //! half the inspector is inert depending on what is selected - and a silently
        //! ignored edit (resizing a mesh body, pinning a cube) reads as a bug. Same
        //! per-type visibility idiom the vehicle configuration already uses.
        AZ::Crc32 GetProceduralShapeVisibility() const;  //!< Cloth, Cube, Balloon
        AZ::Crc32 GetMeshShapeVisibility() const;        //!< Mesh only
        AZ::Crc32 GetClothOnlyVisibility() const;        //!< Cloth only (pinning presets)
        AZ::Crc32 GetClosedShapeVisibility() const;      //!< Cube, Balloon, Mesh (pressure)

        AZ_CLASS_ALLOCATOR(JoltSoftBodySettings, AZ::SystemAllocator);
        AZ_TYPE_INFO(JoltSoftBodySettings, "{7C1E9A4B-2D63-4F8A-B5C7-1E9A4B2D63F8}");

        //! Baked: which geometry is generated.
        JoltSoftBodyShape m_shape = JoltSoftBodyShape::Cloth;
        //! Baked: the surface for the Mesh shape - the first triangle mesh of a cooked
        //! .joltmesh asset, welded so split render vertices simulate as one particle.
        //! Ignored by the procedural shapes; must be loaded before the body is built.
        AZ::Data::Asset<Pipeline::JoltMeshAsset> m_meshAsset{ AZ::Data::AssetLoadBehavior::QueueLoad };
        //! Baked: how a cloth is held up. Ignored by the other shapes (a mesh has no
        //! canonical corners or top edge; pin its particles at runtime instead).
        JoltSoftBodyPinning m_pinning = JoltSoftBodyPinning::Corners;
        //! Baked: extents in metres. A Cube and a Balloon use the X component only, since
        //! Jolt's cube helper takes a single spacing and the balloon is a sphere.
        AZ::Vector3 m_size = AZ::Vector3(2.0f, 2.0f, 2.0f);
        //! Baked: particles along each axis. Cost grows sharply - a cloth is the square of
        //! this and a cube the cube of it.
        AZ::u32 m_resolution = 8;
        //! Baked: total mass in kg, divided evenly over the unpinned particles - pinned
        //! ones have infinite mass by definition, so they take no share.
        float m_mass = 5.0f;
        //! Baked: edge stiffness. 0 is inextensible, larger values stretch.
        float m_compliance = 0.0f;
        //! Baked: tethers every free particle to its closest pinned one, which keeps cloth
        //! from stretching regardless of iteration count. Needs pinned particles.
        JoltSoftBodyLraType m_lraType = JoltSoftBodyLraType::None;

        //! Live: solver iterations per step.
        AZ::u32 m_numIterations = 5;
        //! Live: velocity lost per second.
        float m_linearDamping = 0.1f;
        //! Live: internal pressure. Only does anything on a closed shape.
        float m_pressure = 0.0f;
        //! Live: per-body gravity multiplier.
        float m_gravityFactor = 1.0f;
        //! Live: how strongly the scene's wind pushes on this body's faces. 1 is a plain
        //! aerodynamic response, 0 opts out. On by default because a cloth that ignores
        //! the level's wind regions while everything else respects them reads as broken -
        //! and with no wind regions authored, the wind is zero and this costs nothing.
        float m_windInfluence = 1.0f;
        //! Live: surface friction when sliding over other bodies. A soft body has no
        //! per-shape physics material, so this stands in for the material's friction.
        float m_friction = 0.2f;
        //! Live: bounciness when colliding. 0 lands dead.
        float m_restitution = 0.0f;
        //! Live: particle radius in metres. A little padding keeps the surface from
        //! z-fighting with whatever it rests on.
        float m_vertexRadius = 0.0f;
        //! Live: cap on any particle's speed (m/s), Jolt's guard against explosion.
        float m_maxLinearVelocity = 500.0f;
        //! Live: whether the body's position follows its particles. Off for something
        //! anchored to the static world, so the broadphase entry stays put.
        bool m_updatePosition = true;
        //! Live: whether faces collide and raycast from both sides. Jolt defaults this
        //! off, but a thin cloth that can only be hit from the front reads as broken, so
        //! this gem defaults it on.
        bool m_doubleSidedFaces = true;
        //! Baked: whether the body may go to sleep once it settles.
        bool m_allowSleeping = true;

        //! Live: which collision layer the body is on, and which layers it collides with.
        //! Live rather than baked because Jolt can move a body between object layers
        //! (`BodyInterface::SetObjectLayer`) without rebuilding it — and rebuilding a soft
        //! body would throw away whatever deformation it had settled into.
        AzPhysics::CollisionLayer m_collisionLayer;
        AzPhysics::CollisionGroups::Id m_collisionGroupId;
    };

    //! What JoltScene::AddSimulatedBody needs to build a soft body: the settings plus the
    //! world placement, since a soft body's particles are generated in world space at
    //! creation rather than following an entity transform afterwards.
    struct JoltSoftBodyConfiguration : public AzPhysics::SimulatedBodyConfiguration
    {
        AZ_CLASS_ALLOCATOR(JoltSoftBodyConfiguration, AZ::SystemAllocator);
        AZ_RTTI(JoltSoftBodyConfiguration, "{1BFA7A03-9E43-4A2C-A6F0-5D62E7B7A9C4}",
            AzPhysics::SimulatedBodyConfiguration);
        static void Reflect(AZ::ReflectContext* context);

        JoltSoftBodySettings m_settings;
    };

    //! A Jolt soft body - cloth, a wobbling cube or a pressurised balloon - in one of this
    //! gem's scenes.
    //!
    //! Body lifetime and locking here are the opposite of the JoltBuoyancy gem's water
    //! volume, and both halves are load-bearing. That volume runs inside a step listener,
    //! where every body mutex is already held, so it must use the no-lock interface and may
    //! not add or remove bodies. This class *creates and destroys* bodies, which is only
    //! legal outside the step, and it reads particle positions while the solver may be
    //! running, so it takes a real body lock. Creating a body from a step listener would
    //! deadlock, and reading particles without a lock would tear.
    //!
    //! This lived in a separate JoltSoftBody gem first. It moved in because a soft body
    //! *is* a body: it needs object layers, collision filtering and eventually scene handles
    //! and collision events, all of which are this gem's internals. Buoyancy stays outside
    //! because it only perturbs bodies that something else owns.
    class JoltSoftBody final : public AzPhysics::SimulatedBody
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltSoftBody, AZ::SystemAllocator);
        AZ_RTTI(JoltSoftBody, "{2C6B4E18-70D5-49A7-8F31-9E0C5A7B4D62}", AzPhysics::SimulatedBody);

        JoltSoftBody() = default;
        explicit JoltSoftBody(const JoltSoftBodyConfiguration& configuration);
        ~JoltSoftBody() override;

        JoltSoftBody(const JoltSoftBody&) = delete;
        JoltSoftBody& operator=(const JoltSoftBody&) = delete;

        //! Builds the body in the scene that owns it. Called by JoltScene::AddSimulatedBody.
        void CreateInScene(JoltScene* scene);

        //! Removes the body from the physics system, leaving the object intact for the
        //! scene's deferred deletion.
        void RemoveFromJoltWorld();

        //! Builds the body and adds it to the given scene. Returns false when the scene is
        //! not backed by Jolt, which is the case under any other physics backend.
        bool Attach(AzPhysics::SceneHandle sceneHandle);

        //! Builds the body in a Jolt physics system directly, for callers that own one
        //! rather than reaching it through an AzPhysics scene. Takes an explicit object
        //! layer, which is also the seam the tests attach through.
        bool AttachToPhysicsSystem(JPH::PhysicsSystem* physicsSystem, JPH::ObjectLayer objectLayer);

        //! Removes the body from the scene and destroys it. Safe to call when detached.
        void Detach();

        bool IsAttached() const;

        // AzPhysics::SimulatedBody
        AzPhysics::SceneQueryHit RayCast(const AzPhysics::RayCastRequest& request) override;
        AZ::Crc32 GetNativeType() const override;
        void* GetNativePointer() const override;
        AZ::EntityId GetEntityId() const override;
        AZ::Transform GetTransform() const override;
        AZ::Vector3 GetPosition() const override;
        AZ::Quaternion GetOrientation() const override;
        //! Bounds of the simulated particles, which is where the body actually is - a
        //! deformed cloth can be nowhere near its creation transform.
        AZ::Aabb GetAabb() const override;

        //! The body's world placement. Applied when the body is built; moving an existing
        //! soft body would fight the solver, so this only takes effect on the next build.
        void SetTransform(const AZ::Transform& worldTransform) override;

        JPH::BodyID GetBodyId() const;

        //! Replaces the settings. Live fields are pushed to the body immediately; changing
        //! a baked field rebuilds the body in place, keeping it in the same scene.
        void SetSettings(const JoltSoftBodySettings& settings);
        JoltSoftBodySettings GetSettings() const;

        //! Pushes the scene's wind onto the body's faces for one fixed step.
        //!
        //! Called from the scene's simulation-start event, which is the one moment that is
        //! both per-step (wind is a force, so it scales with time actually simulated, not
        //! with render frames) and outside the solver (this takes a body write lock).
        //! Samples Physics::WindRequests over the body's bounds; each face takes a pressure
        //! along its normal proportional to the squared normal component of the relative
        //! wind, which is what fills a sail face-on and lets it luff edge-on.
        void ApplyWind(float deltaTime);

        //! The total impulse the last ApplyWind put into the body, in world space and
        //! newton-seconds. This is the sail's pull: what an attachment transfers to the
        //! rigid body the cloth is rigged to, so the boat is driven by the same wind its
        //! sail visibly catches rather than by a second, made-up force. Zero when the air
        //! was still.
        AZ::Vector3 GetLastWindImpulse() const;

        void SetWindInfluence(float influence);

        void SetPressure(float pressure);
        void SetLinearDamping(float damping);
        void SetGravityFactor(float factor);
        void SetNumIterations(AZ::u32 iterations);
        void SetFriction(float friction);
        void SetRestitution(float restitution);
        void SetCollisionLayer(const AzPhysics::CollisionLayer& layer);
        void SetCollisionGroupId(const AzPhysics::CollisionGroups::Id& groupId);

        //! The Jolt object layer the body currently occupies. Exposed so tests and tools
        //! can confirm the layer/group pair actually reached the body.
        JPH::ObjectLayer GetObjectLayer() const;

        AZ::u32 GetVertexCount() const;
        AZ::Vector3 GetVertexPosition(AZ::u32 index) const;
        AZ::Aabb GetWorldBounds() const;

        //! Pins one particle where it currently is, or releases it with the same share of
        //! the body's mass every other free particle carries. Runtime pins live on the
        //! particles themselves, so any rebuild reverts to the authored pinning. Returns
        //! false for an out-of-range index or a detached body.
        bool SetVertexPinned(AZ::u32 index, bool pinned);
        bool IsVertexPinned(AZ::u32 index) const;

        //! One particle's velocity, in world space. Setting the velocity of a pinned
        //! particle is refused (returns false), as it would have no effect.
        bool SetVertexVelocity(AZ::u32 index, const AZ::Vector3& velocity);
        AZ::Vector3 GetVertexVelocity(AZ::u32 index) const;

        //! Baked: replaces the geometry and switches the shape to Custom. Positions are
        //! entity-local, the same space the Mesh shape's cooked vertices are in. Rebuilds
        //! the body; empty geometry clears it. Shared positions weld into one particle, so
        //! already-unique geometry keeps its vertex count and order.
        void SetCustomGeometry(
            const AZStd::vector<AZ::Vector3>& vertices, const AZStd::vector<AZ::u32>& indices);
        bool HasCustomGeometry() const;

        //! Baked: ties particles to an animated skeleton through Jolt's skinned
        //! constraints. jointInvBinds holds each joint's inverse bind transform in the
        //! soft body's local frame (it takes a particle's rest position to joint-local
        //! space); skinnedVertices lists which particles are skinned and how. Rebuilds
        //! the body. Not supported for the Cube shape, whose layout Jolt pre-optimises.
        //! Passing empty data clears the skinning. C++ only - this is the seam an
        //! actor/Atom integration drives, not a per-frame scripting surface.
        void SetSkinningData(
            const AZStd::vector<AZ::Transform>& jointInvBinds,
            const AZStd::vector<JoltSoftBodySkinnedVertex>& skinnedVertices);
        bool HasSkinningData() const;

        //! Per-frame: recomputes the skinned target positions from the current joint
        //! transforms, expressed in the soft body's local frame in the same order as the
        //! inverse binds. hardSkinAll snaps every skinned particle exactly onto its
        //! target, which is how a soft body is reset onto a newly-posed skeleton.
        bool UpdateSkinnedJoints(const AZStd::vector<AZ::Transform>& jointTransforms, bool hardSkinAll = false);

        //! The frame Jolt does its skinning maths in: the body's centre-of-mass
        //! transform. Joint transforms handed to UpdateSkinnedJoints and the bind
        //! transforms behind SetSkinningData are relative to this, so anything working in
        //! world space has to divide it out first. Identity for a detached body.
        AZ::Transform GetSkinningFrame() const;

        //! Live: turns the skinned constraints on or off without losing the data.
        void SetSkinConstraintsEnabled(bool enabled);

        //! Jolt's fine-grained collision group - group/sub-group ids plus an optional
        //! GroupFilter - the same mechanism ragdolls use to stop touching limbs
        //! colliding. Applied live; survives rebuilds. C++ only: the coarse layer/group
        //! pair on the settings covers script and editor filtering.
        void SetCollisionGroup(const JPH::CollisionGroup& group);
        JPH::CollisionGroup GetCollisionGroup() const;

        //! Copies every particle position, in world space, into outPositions. Returns false
        //! when the body is not in a scene, leaving outPositions empty. One locked pass
        //! rather than GetVertexPosition per particle, which is what rendering needs.
        bool CopyVertexPositions(AZStd::vector<AZ::Vector3>& outPositions) const;

        //! Triangle indices into the particle array, for drawing the surface. Fixed for the
        //! lifetime of a built body, so a renderer can read it once per rebuild.
        const AZStd::vector<AZ::u32>& GetTriangleIndices() const
        {
            return m_triangleIndices;
        }

        //! Increments on every rebuild, so a renderer can tell that the triangle list it
        //! cached is stale without comparing the contents.
        AZ::u32 GetBuildGeneration() const;

    private:
        //! Generates the particle layout for the current settings. Separated from Attach so
        //! that a settings change can rebuild without re-resolving the scene.
        //! outPerVertexInvMass is the inverse mass every free particle was given, kept so a
        //! runtime unpin can restore exactly that share.
        JPH::Ref<JPH::SoftBodySharedSettings> BuildSharedSettings(
            AZStd::vector<AZ::u32>& outTriangleIndices, float& outPerVertexInvMass) const;

        //! Says which entity lost faces and how many, once per build, for every shape.
        void ReportDegenerateFaces(size_t dropped, size_t total, size_t orphans) const;

        //! Creates and adds the body. Assumes the caller holds m_mutex and that any
        //! previous body has been removed.
        bool CreateBody();

        //! Removes and destroys the body. Assumes the caller holds m_mutex.
        void DestroyBody();

        //! Pushes the live settings onto an existing body. Assumes the caller holds m_mutex.
        void ApplyLiveSettings();

        //! Re-resolves the object layer from the settings' collision layer and group, and
        //! moves an existing body onto it. Assumes the caller holds m_mutex.
        void RefreshObjectLayer();

        //! Guards every member below: gameplay writes settings from the main thread while
        //! rendering reads particle positions.
        mutable AZStd::mutex m_mutex;

        AZ::EntityId m_entityId;
        JoltScene* m_scene = nullptr;
        JPH::PhysicsSystem* m_physicsSystem = nullptr;
        JPH::ObjectLayer m_objectLayer = 0;
        JPH::BodyID m_bodyId;

        //! Copies the skinning data onto the shared settings. Assumes the settings'
        //! vertices and faces are final and Optimize has not run yet.
        void ApplySkinningData(JPH::SoftBodySharedSettings& settings) const;

        JoltSoftBodySettings m_settings;
        AZ::Transform m_worldTransform = AZ::Transform::CreateIdentity();
        AZStd::vector<AZ::u32> m_triangleIndices;
        AZ::u32 m_buildGeneration = 0;
        //! The inverse mass a free particle carries in the current build, so unpinning a
        //! particle at runtime restores the same share instead of a guess.
        float m_perVertexInvMass = 1.0f;

        //! Geometry for the Custom shape, entity-local. Not part of the settings: it comes
        //! from a live mesh rather than from anything an author serialises, and copying it
        //! through every GetSettings/SetSettings round trip would cost a megabyte a call on
        //! a real character.
        AZStd::vector<AZ::Vector3> m_customVertices;
        AZStd::vector<AZ::u32> m_customIndices;

        //! What the last wind application added, in world space. Read by attachments to
        //! push the body the cloth is rigged to.
        AZ::Vector3 m_lastWindImpulse = AZ::Vector3::CreateZero();

        //! Skinning data, baked into the shared settings on every rebuild.
        AZStd::vector<AZ::Transform> m_jointInvBinds;
        AZStd::vector<JoltSoftBodySkinnedVertex> m_skinnedVertices;
        bool m_skinConstraintsEnabled = true;

        //! Jolt collision group, applied at creation and re-applied on rebuilds.
        JPH::CollisionGroup m_collisionGroup;
    };
} // namespace JoltPhysics
