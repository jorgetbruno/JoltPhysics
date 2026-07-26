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

        //! Pulls the grid description out of the provider.
        void ReadGridFromProvider(EditorColliderGeometry::HeightfieldGrid& outGrid) const;

        //! Rebuilds the wireframe and bounds from whatever is currently in m_grid.
        void RebuildFromGrid() const;

        //! Re-reads the grid and rebuilds unconditionally.
        void RefreshFromProvider() const;

        //! Brings the wireframe up to date if it needs it.
        //!
        //! The grid is cached rather than pulled per frame: GetHeights copies the whole
        //! thing, which is megabytes for a terrain-sized heightfield. A provider that
        //! announces its changes refreshes the cache immediately through
        //! HeightfieldProviderNotificationBus; one that mutates silently is caught by a
        //! periodic re-read, which is the only way to notice - the provider interface
        //! has no revision to compare against, so "did this change?" and "give me the
        //! data" are the same question.
        void RefreshIfStale() const;

        //! Enough to read a terrain's shape without turning the wireframe into a solid
        //! block of lines. Terrain heightfields run to 512 samples a side and more.
        static constexpr AZ::u32 MaxWireframeLinesPerAxis = 64;

        //! One full re-read every this many draws, roughly four times a second at 60 fps.
        //! Counted in draws rather than seconds so the cost is proportional to what is
        //! actually on screen - a heightfield nobody is looking at is never re-read -
        //! and so the behaviour does not depend on a clock.
        static constexpr AZ::u32 DrawsBetweenProviderPolls = 15;

        mutable EditorColliderGeometry::HeightfieldGrid m_grid;
        mutable AZStd::vector<AZ::Vector3> m_wireframeLines; //!< Entity-local point pairs.
        mutable AZStd::vector<AZ::Vector3> m_wireframeLinesWorld; //!< Per-frame transformed scratch.
        mutable AZ::Aabb m_localBounds = AZ::Aabb::CreateNull();
        mutable bool m_dirty = true;
        mutable AZ::u32 m_drawsSincePoll = 0;
    };
} // namespace JoltPhysics
