#include <Editor/ConfigurationWindow/JoltConfigurationWidget.h>

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzQtComponents/Components/Widgets/SegmentControl.h>
#include <AzQtComponents/Components/Widgets/TabWidget.h>
#include <AzToolsFramework/UI/PropertyEditor/ReflectedPropertyEditor.hxx>

#include <Editor/ConfigurationWindow/CollisionGroupsWidget.h>
#include <Editor/ConfigurationWindow/CollisionLayersWidget.h>
#include <System/JoltSystem.h>

#include <QVBoxLayout>

namespace JoltPhysics::Editor
{
    namespace
    {
        constexpr int GlobalTabIndex = 0;
        constexpr int FilteringTabIndex = 1;
        constexpr int LayersSegmentIndex = 0;
        constexpr int GroupsSegmentIndex = 1;
    } // namespace

    JoltConfigurationWidget::JoltConfigurationWidget(QWidget* parent)
        : QWidget(parent)
    {
        AZ::SerializeContext* serializeContext = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(
            serializeContext, &AZ::ComponentApplicationBus::Events::GetSerializeContext);
        AZ_Assert(serializeContext, "JoltConfigurationWidget: no serialize context");

        m_tabs = new AzQtComponents::TabWidget(this);

        // Global configuration: one property editor over both configuration objects.
        m_globalEditor = new AzToolsFramework::ReflectedPropertyEditor(m_tabs);
        m_globalEditor->Setup(serializeContext, this, /*enableScrollbars*/ true);
        m_tabs->addTab(m_globalEditor, tr("Global Configuration"));

        // Collision filtering: layers and groups behind a segment control, like the
        // PhysX window.
        m_layersWidget = new CollisionLayersWidget();
        m_groupsWidget = new CollisionGroupsWidget();
        m_filteringTabs = new AzQtComponents::SegmentControl(m_tabs);
        m_filteringTabs->addTab(m_layersWidget, tr("Layers"));
        m_filteringTabs->addTab(m_groupsWidget, tr("Groups"));
        m_tabs->addTab(m_filteringTabs, tr("Collision Filtering"));

        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->addWidget(m_tabs);

        connect(m_layersWidget, &CollisionLayersWidget::onValueChanged, this,
            [this](const AzPhysics::CollisionLayers& layers)
            {
                m_systemConfig.m_collisionConfig.m_collisionLayers = layers;
                // Layer names label the group matrix's columns, so renames must
                // propagate.
                m_groupsWidget->SetValue(m_systemConfig.m_collisionConfig.m_collisionGroups, layers);
                ApplyAndSave();
            });

        connect(m_groupsWidget, &CollisionGroupsWidget::onValueChanged, this,
            [this](const AzPhysics::CollisionGroups& groups)
            {
                m_systemConfig.m_collisionConfig.m_collisionGroups = groups;
                ApplyAndSave();
            });

        LoadFromPhysicsSystem();

        JoltConfigurationWindowRequestBus::Handler::BusConnect();
    }

    JoltConfigurationWidget::~JoltConfigurationWidget()
    {
        JoltConfigurationWindowRequestBus::Handler::BusDisconnect();
    }

    void JoltConfigurationWidget::LoadFromPhysicsSystem()
    {
        if (JoltSystem* joltSystem = GetJoltSystem())
        {
            m_systemConfig = joltSystem->GetJoltConfiguration();
            m_defaultSceneConfig = joltSystem->GetDefaultSceneConfiguration();
        }

        m_globalEditor->ClearInstances();
        m_globalEditor->AddInstance(&m_systemConfig);
        m_globalEditor->AddInstance(&m_defaultSceneConfig);
        m_globalEditor->InvalidateAll();
        m_globalEditor->ExpandAll();

        m_layersWidget->SetValue(m_systemConfig.m_collisionConfig.m_collisionLayers);
        m_groupsWidget->SetValue(
            m_systemConfig.m_collisionConfig.m_collisionGroups,
            m_systemConfig.m_collisionConfig.m_collisionLayers);
    }

    void JoltConfigurationWidget::ApplyAndSave()
    {
        JoltSystem* joltSystem = GetJoltSystem();
        if (joltSystem == nullptr)
        {
            return;
        }

        joltSystem->UpdateConfiguration(&m_systemConfig);
        joltSystem->UpdateDefaultSceneConfiguration(m_defaultSceneConfig);

        [[maybe_unused]] const bool saved =
            joltSystem->GetSettingsRegistryManager().SaveConfiguration(m_systemConfig, m_defaultSceneConfig);
        AZ_Warning("JoltPhysics", saved, "Failed to save the Jolt Physics configuration to the project registry");
    }

    void JoltConfigurationWidget::ShowGlobalSettingsTab()
    {
        m_tabs->setCurrentIndex(GlobalTabIndex);
    }

    void JoltConfigurationWidget::ShowCollisionLayersTab()
    {
        m_tabs->setCurrentIndex(FilteringTabIndex);
        m_filteringTabs->setCurrentIndex(LayersSegmentIndex);
    }

    void JoltConfigurationWidget::ShowCollisionGroupsTab()
    {
        m_tabs->setCurrentIndex(FilteringTabIndex);
        m_filteringTabs->setCurrentIndex(GroupsSegmentIndex);
    }

    void JoltConfigurationWidget::BeforePropertyModified(AzToolsFramework::InstanceDataNode* /*node*/)
    {
    }

    void JoltConfigurationWidget::AfterPropertyModified(AzToolsFramework::InstanceDataNode* /*node*/)
    {
        ApplyAndSave();
    }

    void JoltConfigurationWidget::SetPropertyEditingActive(AzToolsFramework::InstanceDataNode* /*node*/)
    {
    }

    void JoltConfigurationWidget::SetPropertyEditingComplete(AzToolsFramework::InstanceDataNode* /*node*/)
    {
    }

    void JoltConfigurationWidget::SealUndoStack()
    {
    }
} // namespace JoltPhysics::Editor
