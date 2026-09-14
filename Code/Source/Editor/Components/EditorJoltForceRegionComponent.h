#pragma once

#include <AzCore/std/string/string.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

#include <ForceRegion/JoltForceRegionForces.h>

namespace JoltPhysics
{
    //! Editor Jolt Force Region: edit-time counterpart of JoltForceRegionComponent
    //! (PhysX-style editor/runtime split). Holds the forces and the wind tag, spawns the
    //! runtime component via BuildGameEntity, and draws the forces in the viewport.
    //!
    //! The preview is the point of this component. The runtime one already composed on
    //! editor entities - the editor colliders provide the service it needs - and sat
    //! inert there, since the default scene it looks for does not exist in edit mode. But
    //! a fan, a current or a wind zone was a wireframe box with nothing in it saying which
    //! way it pushed, and the only way to find out was to enter game mode and drop
    //! something in.
    class EditorJoltForceRegionComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , private AzFramework::EntityDebugDisplayEventBus::Handler
    {
    public:
        AZ_COMPONENT(EditorJoltForceRegionComponent, "{5B7E2C91-4D3A-4F86-9E1B-7C2A8D4F6E10}", AzToolsFramework::Components::EditorComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

        const JoltForceRegion& GetForceRegion() const
        {
            return m_forceRegion;
        }

        //! Tests only: the fields are authored through the inspector, not in code.
        void SetForceRegionForTesting(const JoltForceRegion& forceRegion)
        {
            m_forceRegion = forceRegion;
        }
        void SetWindTagForTesting(const AZStd::string& windTag)
        {
            m_windTag = windTag;
        }

    protected:
        void Activate() override;
        void Deactivate() override;

        // AzFramework::EntityDebugDisplayEvents
        void DisplayEntityViewport(
            const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

    private:
        JoltForceRegion m_forceRegion;
        AZStd::string m_windTag;
    };
} // namespace JoltPhysics
