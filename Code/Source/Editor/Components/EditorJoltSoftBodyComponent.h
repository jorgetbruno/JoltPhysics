#pragma once

#include <AzCore/Component/TransformBus.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

#include <SoftBody/JoltSoftBody.h>

namespace JoltPhysics
{
    //! Editor Soft Body: previews the soft body in the Edit viewport and spawns the runtime
    //! JoltSoftBodyComponent via BuildGameEntity, following the same editor/runtime split as
    //! the gem's other components.
    //!
    //! With Live preview off the viewport shows the rest shape. With it on, a real soft body
    //! simulates in the edit-mode physics scene (EditorWorldBus) - the same scene the editor
    //! colliders put their static bodies in, so a curtain drapes over the level's actual
    //! geometry before play is ever pressed.
    class EditorJoltSoftBodyComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , private AzFramework::EntityDebugDisplayEventBus::Handler
        , private AZ::TransformNotificationBus::Handler
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

        //! The authored settings, the same accessor the runtime component offers. Changes
        //! only reach a live preview body on the next RefreshPreviewBody (the property
        //! editor's ChangeNotify, or toggling the preview).
        JoltSoftBodySettings& GetSettings()
        {
            return m_settings;
        }

        //! The same switch the Live preview checkbox drives; public so tools and tests can
        //! turn the edit-mode simulation on without a property editor.
        void SetLivePreviewEnabled(bool enabled);

        //! Whether a preview body is currently simulating in the editor scene. False when
        //! the preview is off or there is no editor world to host it.
        bool HasLivePreviewBody() const;

        //! The simulating preview body, for tests that assert on what the preview does.
        const JoltSoftBody* GetPreviewBody() const
        {
            return m_previewBody.get();
        }

    private:
        // AzFramework::EntityDebugDisplayEvents
        void DisplayEntityViewport(
            const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

        // AZ::TransformNotificationBus
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        //! ChangeNotify for the settings and the Live preview toggle.
        AZ::u32 OnConfigurationChanged();

        //! Creates, updates or destroys the preview body to match the current settings and
        //! the Live preview toggle.
        void RefreshPreviewBody();
        void DestroyPreviewBody();

        JoltSoftBodySettings m_settings;
        bool m_visible = true;
        //! Simulate in the edit-mode scene instead of showing the rest shape.
        bool m_livePreview = false;

        //! This component owns the preview body directly rather than handing it to the
        //! scene: it is editor-only furniture, and BuildGameEntity must not carry it over.
        AZStd::unique_ptr<JoltSoftBody> m_previewBody;
        //! Reused every frame so drawing the preview does not allocate per draw.
        AZStd::vector<AZ::Vector3> m_previewPositions;
    };
} // namespace JoltPhysics
