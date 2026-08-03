#pragma once

#include <Editor/Components/EditorJoltColliderComponentBase.h>

#include <LmbrCentral/Shape/ShapeComponentBus.h>

namespace JoltPhysics
{
    //! Editor Shape Collider: collision geometry taken from the shape component on the
    //! same entity, drawn in the Edit viewport and spawned as a runtime
    //! JoltShapeColliderComponent on export.
    //!
    //! No manipulators of its own - the shape component already owns the handles that
    //! resize it, and duplicating them would give an author two ways to change one thing.
    class EditorJoltShapeColliderComponent
        : public EditorJoltColliderComponentBase
        , private LmbrCentral::ShapeComponentNotificationsBus::Handler
    {
    public:
        AZ_COMPONENT(EditorJoltShapeColliderComponent, "{7E4B2A96-3D51-4C08-B7F2-9A6E1D0C5B34}", EditorJoltColliderComponentBase);

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    protected:
        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // EditorJoltColliderComponentBase
        void DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const override;
        AZ::Aabb GetLocalShapeBounds() const override;
        AzPhysics::ShapeColliderPairList GetEditorShapeColliderPairs() const override;

        // LmbrCentral::ShapeComponentNotificationsBus
        void OnShapeChanged(ShapeChangeReasons changeReasons) override;

    private:
        //! Cached wireframe of whatever the shape component currently describes, rebuilt
        //! when the shape reports a change rather than every frame.
        void RebuildDebugLines() const;

        mutable AZStd::vector<AZ::Vector3> m_debugLines;
        mutable AZStd::vector<AZ::Vector3> m_debugLinesWorld;
        mutable AZ::Aabb m_debugBounds = AZ::Aabb::CreateNull();
        mutable bool m_debugLinesDirty = true;
    };
} // namespace JoltPhysics
