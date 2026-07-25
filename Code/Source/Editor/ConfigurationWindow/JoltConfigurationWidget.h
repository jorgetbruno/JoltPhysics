#pragma once

#if !defined(Q_MOC_RUN)
#include <AzCore/Memory/SystemAllocator.h>
#include <AzFramework/Physics/Configuration/SceneConfiguration.h>
#include <AzToolsFramework/UI/PropertyEditor/PropertyEditorAPI_Internals.h>

#include <JoltPhysics/Configuration/JoltConfiguration.h>

#include <Editor/ConfigurationWindow/JoltConfigurationWindowBus.h>
#endif

#include <QWidget>

namespace AzQtComponents
{
    class TabWidget;
    class SegmentControl;
}

namespace AzToolsFramework
{
    class ReflectedPropertyEditor;
}

namespace JoltPhysics::Editor
{
    class CollisionLayersWidget;
    class CollisionGroupsWidget;

    //! The Jolt Physics Configuration view pane: edits the system configuration
    //! (timestep, buffers, Jolt capacities), the default scene configuration
    //! (gravity), and the collision layers and groups. Every change is applied to the
    //! live physics system and saved to the project's settings registry immediately —
    //! there is no separate save step, matching the PhysX Configuration window.
    class JoltConfigurationWidget
        : public QWidget
        , private AzToolsFramework::IPropertyEditorNotify
        , private JoltConfigurationWindowRequestBus::Handler
    {
        Q_OBJECT

    public:
        AZ_CLASS_ALLOCATOR(JoltConfigurationWidget, AZ::SystemAllocator);

        explicit JoltConfigurationWidget(QWidget* parent = nullptr);
        ~JoltConfigurationWidget() override;

        // JoltConfigurationWindowRequestBus
        void ShowGlobalSettingsTab() override;
        void ShowCollisionLayersTab() override;
        void ShowCollisionGroupsTab() override;

    private:
        void LoadFromPhysicsSystem();
        void ApplyAndSave();

        // AzToolsFramework::IPropertyEditorNotify (global settings tab)
        void BeforePropertyModified(AzToolsFramework::InstanceDataNode* node) override;
        void AfterPropertyModified(AzToolsFramework::InstanceDataNode* node) override;
        void SetPropertyEditingActive(AzToolsFramework::InstanceDataNode* node) override;
        void SetPropertyEditingComplete(AzToolsFramework::InstanceDataNode* node) override;
        void SealUndoStack() override;

        AzQtComponents::TabWidget* m_tabs = nullptr;
        AzToolsFramework::ReflectedPropertyEditor* m_globalEditor = nullptr;
        AzQtComponents::SegmentControl* m_filteringTabs = nullptr;
        CollisionLayersWidget* m_layersWidget = nullptr;
        CollisionGroupsWidget* m_groupsWidget = nullptr;

        JoltSystemConfiguration m_systemConfig;
        AzPhysics::SceneConfiguration m_defaultSceneConfig;
    };
} // namespace JoltPhysics::Editor
