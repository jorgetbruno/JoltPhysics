#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Math/Aabb.h>
#include <AzCore/Math/Vector3.h>

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

        //! Which collision layer the body sits on, and which layers it collides with.
        //! Changing either moves the body between Jolt object layers without rebuilding
        //! it, so a soft body keeps whatever deformation it had.
        virtual void SetCollisionLayer(const AzPhysics::CollisionLayer& layer) = 0;
        virtual AzPhysics::CollisionLayer GetCollisionLayer() const = 0;
        virtual void SetCollisionGroupId(const AzPhysics::CollisionGroups::Id& groupId) = 0;
        virtual AzPhysics::CollisionGroups::Id GetCollisionGroupId() const = 0;

        //! Removes the body from the simulation, or rebuilds and re-adds it. A disabled
        //! soft body keeps its settings but occupies nothing in the scene.
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
    };

    using JoltSoftBodyRequestBus = AZ::EBus<JoltSoftBodyRequests>;

} // namespace JoltPhysics
