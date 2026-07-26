#pragma once

#include <AzFramework/Physics/HeightfieldProviderBus.h>

#include <Editor/Components/EditorJoltColliderComponentBase.h>
#include <Editor/Components/EditorJoltColliderGeometryUtils.h>

namespace JoltPhysics
{
    //! Editor Jolt Heightfield Collider: edit-time counterpart of
    //! JoltHeightfieldColliderComponent. Spawns the runtime component via
    //! BuildGameEntity; the heightfield itself comes from the entity's
    //! Physics::HeightfieldProviderBus implementation, which this component reads to
    //! draw the collision surface in the viewport and to answer selection queries.
    class EditorJoltHeightfieldColliderComponent
        : public EditorJoltColliderComponentBase
        , private Physics::HeightfieldProviderNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(EditorJoltHeightfieldColliderComponent, "{A7B8C9D0-E1F2-4345-F6A7-B8C9D0E1F2A3}", EditorJoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    protected:
        // EditorJoltColliderComponentBase
        void DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const override;
        AZ::Aabb GetLocalShapeBounds() const override;

    private:
        // Physics::HeightfieldProviderNotificationBus
        void OnHeightfieldDataChanged(
            const AZ::Aabb& dirtyRegion,
            Physics::HeightfieldProviderNotifications::HeightfieldChangeMask changeMask) override;

        //! Re-reads the grid from the provider and rebuilds the wireframe and bounds.
        //! Cached rather than pulled per frame: GetHeights copies the whole grid, which
        //! is megabytes for a terrain-sized heightfield. Refreshed when the provider
        //! announces a change; a provider that mutates its data silently (the runtime
        //! component polls on tick for exactly that case) leaves the wireframe stale
        //! until the next announcement.
        void RefreshFromProvider() const;

        //! Enough to read a terrain's shape without turning the wireframe into a solid
        //! block of lines. Terrain heightfields run to 512 samples a side and more.
        static constexpr AZ::u32 MaxWireframeLinesPerAxis = 64;

        mutable EditorColliderGeometry::HeightfieldGrid m_grid;
        mutable AZStd::vector<AZ::Vector3> m_wireframeLines; //!< Entity-local point pairs.
        mutable AZStd::vector<AZ::Vector3> m_wireframeLinesWorld; //!< Per-frame transformed scratch.
        mutable AZ::Aabb m_localBounds = AZ::Aabb::CreateNull();
        mutable bool m_dirty = true;
    };
} // namespace JoltPhysics
