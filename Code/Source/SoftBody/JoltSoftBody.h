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
#include <AzFramework/Physics/Common/PhysicsTypes.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>

#include <JoltPhysics/JoltSoftBodyBus.h>

namespace JPH
{
    class PhysicsSystem;
}

namespace JoltPhysics
{
    //! Everything describing a soft body. The fields split into two kinds, and the split
    //! matters: those marked "baked" go into the particle layout when the body is built
    //! and cannot be changed on a live body, while the rest are forwarded straight to
    //! Jolt's motion properties and can change every frame.
    struct JoltSoftBodySettings
    {
        AZ_CLASS_ALLOCATOR(JoltSoftBodySettings, AZ::SystemAllocator);
        AZ_TYPE_INFO(JoltSoftBodySettings, "{7C1E9A4B-2D63-4F8A-B5C7-1E9A4B2D63F8}");

        //! Baked: which geometry is generated.
        JoltSoftBodyShape m_shape = JoltSoftBodyShape::Cloth;
        //! Baked: how a cloth is held up. Ignored by the other shapes.
        JoltSoftBodyPinning m_pinning = JoltSoftBodyPinning::Corners;
        //! Baked: extents in metres. A Cube and a Balloon use the X component only, since
        //! Jolt's cube helper takes a single spacing and the balloon is a sphere.
        AZ::Vector3 m_size = AZ::Vector3(2.0f, 2.0f, 2.0f);
        //! Baked: particles along each axis. Cost grows sharply - a cloth is the square of
        //! this and a cube the cube of it.
        AZ::u32 m_resolution = 8;
        //! Baked: total mass in kg, divided evenly over the particles.
        float m_mass = 5.0f;
        //! Baked: edge stiffness. 0 is inextensible, larger values stretch.
        float m_compliance = 0.0f;

        //! Live: solver iterations per step.
        AZ::u32 m_numIterations = 5;
        //! Live: velocity lost per second.
        float m_linearDamping = 0.1f;
        //! Live: internal pressure. Only does anything on a closed shape.
        float m_pressure = 0.0f;
        //! Live: per-body gravity multiplier.
        float m_gravityFactor = 1.0f;
        //! Baked: whether the body may go to sleep once it settles.
        bool m_allowSleeping = true;

        //! Live: which collision layer the body is on, and which layers it collides with.
        //! Live rather than baked because Jolt can move a body between object layers
        //! (`BodyInterface::SetObjectLayer`) without rebuilding it — and rebuilding a soft
        //! body would throw away whatever deformation it had settled into.
        AzPhysics::CollisionLayer m_collisionLayer;
        AzPhysics::CollisionGroups::Id m_collisionGroupId;
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
    class JoltSoftBody final
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltSoftBody, AZ::SystemAllocator);

        JoltSoftBody() = default;
        ~JoltSoftBody();

        JoltSoftBody(const JoltSoftBody&) = delete;
        JoltSoftBody& operator=(const JoltSoftBody&) = delete;

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

        //! The body's world placement. Applied when the body is built; moving an existing
        //! soft body would fight the solver, so this only takes effect on the next build.
        void SetTransform(const AZ::Transform& worldTransform);

        //! Replaces the settings. Live fields are pushed to the body immediately; changing
        //! a baked field rebuilds the body in place, keeping it in the same scene.
        void SetSettings(const JoltSoftBodySettings& settings);
        JoltSoftBodySettings GetSettings() const;

        void SetPressure(float pressure);
        void SetLinearDamping(float damping);
        void SetGravityFactor(float factor);
        void SetNumIterations(AZ::u32 iterations);
        void SetCollisionLayer(const AzPhysics::CollisionLayer& layer);
        void SetCollisionGroupId(const AzPhysics::CollisionGroups::Id& groupId);

        //! The Jolt object layer the body currently occupies. Exposed so tests and tools
        //! can confirm the layer/group pair actually reached the body.
        JPH::ObjectLayer GetObjectLayer() const;

        AZ::u32 GetVertexCount() const;
        AZ::Vector3 GetVertexPosition(AZ::u32 index) const;
        AZ::Aabb GetWorldBounds() const;

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
        JPH::Ref<JPH::SoftBodySharedSettings> BuildSharedSettings(AZStd::vector<AZ::u32>& outTriangleIndices) const;

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

        JPH::PhysicsSystem* m_physicsSystem = nullptr;
        JPH::ObjectLayer m_objectLayer = 0;
        JPH::BodyID m_bodyId;

        JoltSoftBodySettings m_settings;
        AZ::Transform m_worldTransform = AZ::Transform::CreateIdentity();
        AZStd::vector<AZ::u32> m_triangleIndices;
        AZ::u32 m_buildGeneration = 0;
    };
} // namespace JoltPhysics
