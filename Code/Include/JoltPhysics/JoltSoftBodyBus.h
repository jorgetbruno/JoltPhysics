#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/limits.h>
#include <AzCore/Math/Aabb.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>

#include <AzFramework/Physics/Collision/CollisionGroups.h>
#include <AzFramework/Physics/Collision/CollisionLayers.h>

namespace JoltPhysics
{
    //! Which shape the soft body is built from. Cloth, Cube and Balloon are generated
    //! procedurally; Mesh takes its surface from a cooked .joltmesh asset.
    enum class JoltSoftBodyShape : AZ::u8
    {
        Cloth = 0, //!< A flat grid in the entity's local XY plane. Drapes and folds.
        Cube = 1, //!< A solid grid with volume constraints. Wobbles and keeps its bulk.
        Balloon = 2, //!< A closed grid driven by internal pressure. Inflates and bounces.
        Mesh = 3, //!< The triangle surface of a .joltmesh asset, welded and simulated as cloth.
        Custom = 4, //!< Triangles handed over at runtime through SetCustomGeometry.
    };

    //! How a cloth is held up. A cloth with nothing pinned simply falls.
    enum class JoltSoftBodyPinning : AZ::u8
    {
        None = 0, //!< Nothing pinned: the whole sheet falls.
        Corners = 1, //!< The four corner particles are fixed.
        TopEdge = 2, //!< The entire +Y edge is fixed, like a hanging curtain.
    };

    //! Long range attachment constraints: every free particle is tethered to its closest
    //! pinned particle at the rest-pose distance, which is Jolt's cheap route to cloth
    //! that does not stretch no matter how few solver iterations run. Needs pinned
    //! particles to tether to, so it does nothing on an unpinned body.
    enum class JoltSoftBodyLraType : AZ::u8
    {
        None = 0, //!< No tethers; stiffness comes from compliance and iterations alone.
        EuclideanDistance = 1, //!< Tether length is the straight-line rest distance.
        GeodesicDistance = 2, //!< Tether length follows the edges, so a draped sheet keeps its slack.
    };

    //! One joint's pull on a skinned soft body particle.
    struct JoltSoftBodySkinInfluence
    {
        AZ_TYPE_INFO(JoltSoftBodySkinInfluence, "{6C2E9A17-4B85-4D3F-9E01-7A5C8B0D2F64}");

        //! Index into the inverse-bind / joint-transform arrays.
        AZ::u32 m_jointIndex = 0;
        //! Blend weight; the weights of a vertex are normalized when the body is built.
        float m_weight = 1.0f;
    };

    //! How many joints Jolt lets pull on one skinned particle. Influences past this are
    //! dropped when the body is built, so a caller that ranks its own weights should cut
    //! them here rather than let the backend choose which to lose.
    static constexpr AZ::u32 MaxSoftBodySkinInfluences = 4;

    //! Ties one particle to the skinned position its joints compute.
    //!
    //! The three distances are Jolt's skinned-constraint fields, and they line up with
    //! how O3DE already asks cloth authors to think: the max distance is a motion
    //! constraint (a sphere around the skinned position the particle may move within),
    //! and the backstop is a sphere behind the vertex that it is pushed out of, which
    //! stands in for the volume of the body underneath.
    struct JoltSoftBodySkinnedVertex
    {
        AZ_TYPE_INFO(JoltSoftBodySkinnedVertex, "{0D8A5F31-2C74-4E96-B1A8-3F60C7D9E425}");

        //! Index into the soft body's particle array.
        AZ::u32 m_vertexIndex = 0;
        //! Up to four joint influences; more are ignored.
        AZStd::vector<JoltSoftBodySkinInfluence> m_influences;
        //! How far simulation may drift from the skinned position. 0 hard-skins the
        //! particle; FLT_MAX turns the constraint off for it.
        float m_maxDistance = 0.1f;
        //! Where the backstop sphere starts, behind the particle along the surface
        //! normal. Disabled when it is at or beyond m_maxDistance, which is the default.
        float m_backstopDistance = AZStd::numeric_limits<float>::max();
        //! Radius of that sphere. Large values approximate a plane, which is Jolt's own
        //! default and what a flat surface behind the cloth wants.
        float m_backstopRadius = 40.0f;
    };

    //! Runtime control of a soft body. Every setting that Jolt can change on a live body
    //! is settable here; the ones baked into the particle layout at creation time
    //! (shape, resolution, size) are not, and need the component rebuilt.
    class JoltSoftBodyRequests
        : public AZ::ComponentBus
    {
    public:
        virtual ~JoltSoftBodyRequests() = default;

        //! Internal pressure. Zero for cloth. Positive values inflate a closed body, so
        //! this is what makes a balloon hold its shape instead of collapsing.
        virtual void SetPressure(float pressure) = 0;
        virtual float GetPressure() const = 0;

        //! Fraction of velocity removed per second. Higher values settle the body sooner.
        virtual void SetLinearDamping(float damping) = 0;
        virtual float GetLinearDamping() const = 0;

        //! Multiplier on gravity for this body alone. Zero makes it float in place.
        virtual void SetGravityFactor(float factor) = 0;
        virtual float GetGravityFactor() const = 0;

        //! Solver iterations per step. Higher is stiffer and more expensive.
        virtual void SetNumIterations(AZ::u32 iterations) = 0;
        virtual AZ::u32 GetNumIterations() const = 0;

        //! Friction of the body's surface when sliding over other bodies. A soft body has
        //! no per-shape physics material, so this plays the role the material's friction
        //! plays on a rigid body.
        virtual void SetFriction(float friction) = 0;
        virtual float GetFriction() const = 0;

        //! Bounciness when colliding. 0 lands dead, 1 keeps all its energy.
        virtual void SetRestitution(float restitution) = 0;
        virtual float GetRestitution() const = 0;

        //! Which collision layer the body sits on, and which layers it collides with.
        //! Changing either moves the body between Jolt object layers without rebuilding
        //! it, so a soft body keeps whatever deformation it had.
        virtual void SetCollisionLayer(const AzPhysics::CollisionLayer& layer) = 0;
        virtual AzPhysics::CollisionLayer GetCollisionLayer() const = 0;
        virtual void SetCollisionGroupId(const AzPhysics::CollisionGroups::Id& groupId) = 0;
        virtual AzPhysics::CollisionGroups::Id GetCollisionGroupId() const = 0;

        //! Removes the body from the simulation, or rebuilds and re-adds it. A disabled
        //! soft body keeps its settings but occupies nothing in the scene, so re-enabling
        //! rebuilds from the rest pose: whatever deformation the body had is gone. That is
        //! inherent - the particles only exist while the body does.
        virtual void SetEnabled(bool enabled) = 0;
        virtual bool IsEnabled() const = 0;

        //! Number of particles in the body, or zero when it is not in a scene.
        virtual AZ::u32 GetVertexCount() const = 0;

        //! World-space bounds of the simulated particles, or a null Aabb when the body is
        //! not in a scene. This is how far the body has actually deformed, which the
        //! entity transform alone does not tell you.
        virtual AZ::Aabb GetWorldBounds() const = 0;

        //! World-space position of one particle. Returns a zero vector for an index past
        //! GetVertexCount, so a caller polling a body that has just been disabled reads
        //! zeroes rather than reading out of bounds.
        virtual AZ::Vector3 GetVertexPosition(AZ::u32 index) const = 0;

        //! Every particle position in world space in one call - one body lock instead of
        //! one per particle, which is the difference between script-driven rendering being
        //! feasible and not. Empty when the body is not in a scene.
        virtual AZStd::vector<AZ::Vector3> GetVertexPositions() const = 0;

        //! The same positions, copied into a vector the caller owns and keeps.
        //!
        //! This is the one to use every frame. GetVertexPositions returns by value, so a
        //! renderer polling a cloth of fifty thousand particles allocates and frees that
        //! vector sixty times a second; reusing one buffer costs nothing after the first
        //! call. Returns false and leaves the buffer empty when the body is not in a scene.
        virtual bool CopyVertexPositions(AZStd::vector<AZ::Vector3>& outPositions) const = 0;

        //! Triangle indices into the particle array, three per triangle, for building a
        //! surface from GetVertexPositions. Fixed until the body rebuilds.
        virtual AZStd::vector<AZ::u32> GetTriangleIndices() const = 0;

        //! Pins a particle where it currently is (grabbing cloth, anchoring a corner at
        //! runtime) or releases it, restoring the mass it would have had. Runtime pins
        //! live on the particles, so anything that rebuilds the body - a baked-setting
        //! change, SetEnabled off and on, moving the entity - reverts to the authored
        //! pinning. Returns false for an out-of-range index.
        virtual bool SetVertexPinned(AZ::u32 index, bool pinned) = 0;
        virtual bool IsVertexPinned(AZ::u32 index) const = 0;

        //! Sets one particle's velocity in world space - popping a balloon outward,
        //! flicking a cloth. A pinned particle keeps velocity zero. Returns false for an
        //! out-of-range index.
        virtual bool SetVertexVelocity(AZ::u32 index, const AZ::Vector3& velocity) = 0;
        virtual AZ::Vector3 GetVertexVelocity(AZ::u32 index) const = 0;

        //! @name Geometry
        //! @{

        //! Replaces the body's geometry with the given triangles and switches the shape to
        //! Custom, which is how a caller that already has a mesh - a character's cloth
        //! mesh, geometry built at runtime - simulates that surface rather than one of the
        //! generated shapes. Rebuilds the body. Passing empty geometry clears it, leaving a
        //! Custom body with nothing to build.
        //!
        //! Positions are **entity-local**, the same space the Mesh shape's cooked vertices
        //! are in, because that is the space a model's vertex buffer is already in. The
        //! entity transform places the result.
        //!
        //! Vertices that share a position are welded into one particle, since a render mesh
        //! splits vertices along every normal and UV seam and a sheet simulated unwelded
        //! tears along each one. Geometry whose positions are already unique keeps its
        //! vertex count **and its order**, so a caller that welded the mesh itself - which
        //! anything mapping simulation back onto render vertices has to - can treat
        //! particle indices and its own vertex indices as the same thing.
        virtual void SetCustomGeometry(
            const AZStd::vector<AZ::Vector3>& vertices, const AZStd::vector<AZ::u32>& indices) = 0;

        //! @}

        //! @name Skinning
        //!
        //! Ties particles to an animated skeleton through Jolt's skinned constraints.
        //! Public because the thing that drives it cannot live in this gem: an actor's
        //! pose comes from EMotionFX, which depends on Atom, and this gem depends on no
        //! renderer. The JoltCloth sibling gem is what calls these.
        //!
        //! Not reflected to script. SetSkinningData rebuilds the body, and
        //! UpdateSkinnedJoints wants the whole skeleton every frame - neither is a
        //! per-frame scripting surface, and both would be a foot-gun as one.
        //! @{

        //! Baked: jointBindTransforms holds where each joint sat, **in world space**, at
        //! the moment the binding was computed; skinnedVertices lists which particles are
        //! skinned and how. Rebuilds the body. Not supported for the Cube shape, whose
        //! layout Jolt pre-optimises. Passing empty data clears the skinning.
        virtual void SetSkinningData(
            const AZStd::vector<AZ::Transform>& jointBindTransforms,
            const AZStd::vector<JoltSoftBodySkinnedVertex>& skinnedVertices) = 0;
        virtual bool HasSkinningData() const = 0;

        //! Per-frame: recomputes the skinned target positions from the current joint
        //! transforms, **in world space**, in the same order as the bind transforms.
        //! hardSkinAll snaps every skinned particle exactly onto its target, which is how
        //! a soft body is reset onto a newly-posed skeleton.
        //!
        //! World space on purpose. Jolt wants these relative to the body's centre of
        //! mass, and a caller that converted into the body's *position* instead - the
        //! obvious mistake - would get cloth that hangs at an offset rather than anything
        //! that reads as a bug. The conversion happens on this side of the bus.
        virtual bool UpdateSkinnedJoints(
            const AZStd::vector<AZ::Transform>& jointTransforms, bool hardSkinAll) = 0;

        //! Live: turns the skinned constraints on or off without losing the data.
        virtual void SetSkinConstraintsEnabled(bool enabled) = 0;

        //! @}
    };

    using JoltSoftBodyRequestBus = AZ::EBus<JoltSoftBodyRequests>;

    //! One particle of a soft body touching another body. The index is into the same
    //! particle array GetVertexPositions reads, so a listener can react on the exact
    //! piece of cloth that touched.
    struct JoltSoftBodyParticleContact
    {
        AZ_TYPE_INFO(JoltSoftBodyParticleContact, "{93B1E4C7-5A2D-4F8B-9C6E-1D7A3B5F9E2C}");

        //! Index into the soft body's particle array.
        AZ::u32 m_vertexIndex = 0;
        //! Contact position in world space.
        AZ::Vector3 m_position = AZ::Vector3::CreateZero();
        //! Contact normal in world space, as Jolt reports it (pointing from the other
        //! body towards the soft body).
        AZ::Vector3 m_normal = AZ::Vector3::CreateZero();
    };

    //! Raised on the soft body's entity while its particles touch another body - once per
    //! touched body per simulation step, carrying every contacting particle. Jolt has no
    //! removal callback for soft body contacts, so there is no End notification here; a
    //! listener that stops receiving the event knows contact was lost (the generic
    //! collision events still report Begin/End per body pair).
    class JoltSoftBodyNotifications
        : public AZ::ComponentBus
    {
    public:
        virtual ~JoltSoftBodyNotifications() = default;

        //! otherEntity is the entity of the touched body, invalid for bodies without one.
        virtual void OnSoftBodyContact(
            AZ::EntityId otherEntity, const AZStd::vector<JoltSoftBodyParticleContact>& contacts) = 0;
    };

    using JoltSoftBodyNotificationBus = AZ::EBus<JoltSoftBodyNotifications>;

} // namespace JoltPhysics
