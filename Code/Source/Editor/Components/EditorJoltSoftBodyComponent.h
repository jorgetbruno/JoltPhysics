#pragma once

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

#include <SoftBody/JoltSoftBody.h>

namespace JoltPhysics
{
    //! Editor Soft Body: previews the rest shape in the Edit viewport and spawns the runtime
    //! JoltSoftBodyComponent via BuildGameEntity, following the same editor/runtime split as
    //! the gem's other components.
    //!
    //! The preview is the rest shape only. Nothing is simulated in the Edit viewport, so
    //! what a cloth actually does is only visible in game mode.
    class EditorJoltSoftBodyComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , private AzFramework::EntityDebugDisplayEventBus::Handler
    {
    public:
        AZ_COMPONENT(EditorJoltSoftBodyComponent, "{4A9B2D6E-8C3F-4B7A-9E2D-6C8F3A1B7E2D}",
            AzToolsFramework::Components::EditorComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // EditorComponentBase
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        // AzFramework::EntityDebugDisplayEvents
        void DisplayEntityViewport(
            const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

        JoltSoftBodySettings m_settings;
        bool m_visible = true;
    };
} // namespace JoltPhysics
