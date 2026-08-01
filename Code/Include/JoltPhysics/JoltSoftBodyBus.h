#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Math/Aabb.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>

#include <AzFramework/Physics/Collision/CollisionGroups.h>
#include <AzFramework/Physics/Collision/CollisionLayers.h>

namespace JoltPhysics
{
    //! Which shape the soft body is built from. Jolt simulates particles and constraints,
    //! not a mesh asset, so the geometry is generated rather than loaded.
    enum class JoltSoftBodyShape : AZ::u8
    {
        Cloth = 0, //!< A flat grid in the entity's local XY plane. Drapes and folds.
        Cube = 1, //!< A solid grid with volume constraints. Wobbles and keeps its bulk.
        Balloon = 2, //!< A closed grid driven by internal pressure. Inflates and bounces.
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
    };

    using JoltSoftBodyRequestBus = AZ::EBus<JoltSoftBodyRequests>;

} // namespace JoltPhysics
