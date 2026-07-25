#include <Editor/PropertyHandlers/CollisionLayerWidget.h>

#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Physics/PropertyTypes.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Editor/ConfigurationWindow/JoltConfigurationWindowBus.h>
#include <Editor/PropertyHandlers/CollisionPropertyUtils.h>

#include <QComboBox>
#include <QSignalBlocker>
#include <QToolButton>

namespace JoltPhysics::Editor
{
    AZ::u32 CollisionLayerWidget::GetHandlerName() const
    {
        return Physics::Edit::CollisionLayerSelector;
    }

    bool CollisionLayerWidget::IsDefaultHandler() const
    {
        return true;
    }

    QWidget* CollisionLayerWidget::CreateGUI(QWidget* parent)
    {
        auto* picker = aznew widget_t(parent);

        picker->GetComboBox()->setToolTip("Which collision layer this object is on.");

        // The pencil button jumps to where layers are authored, like PhysX's version
        // of this widget does for its configuration window.
        picker->GetEditButton()->setVisible(true);
        picker->GetEditButton()->setToolTip("Edit collision layers");
        connect(picker->GetEditButton(), &QToolButton::clicked, this, []()
            {
                AzToolsFramework::EditorRequests::Bus::Broadcast(
                    &AzToolsFramework::EditorRequests::OpenViewPane, ConfigurationWindowName);
                JoltConfigurationWindowRequestBus::Broadcast(
                    &JoltConfigurationWindowRequests::ShowCollisionLayersTab);
            });

        connect(picker, &widget_t::valueChanged, this, [picker]()
            {
                AzToolsFramework::PropertyEditorGUIMessages::Bus::Broadcast(
                    &AzToolsFramework::PropertyEditorGUIMessages::RequestWrite, picker);
            });

        return picker;
    }

    void CollisionLayerWidget::ConsumeAttribute(
        widget_t* widget, AZ::u32 attrib, AzToolsFramework::PropertyAttributeReader* attrValue, const char* debugName)
    {
        if (attrib == AZ::Edit::Attributes::ReadOnly)
        {
            bool readOnly = false;
            if (attrValue->Read<bool>(readOnly))
            {
                widget->setEnabled(!readOnly);
            }
        }

        AZ_UNUSED(debugName);
    }

    void CollisionLayerWidget::WriteGUIValuesIntoProperty(
        [[maybe_unused]] size_t index,
        widget_t* GUI,
        property_t& instance,
        [[maybe_unused]] AzToolsFramework::InstanceDataNode* node)
    {
        instance = GetLayerFromName(GUI->value());
    }

    bool CollisionLayerWidget::ReadValuesIntoGUI(
        [[maybe_unused]] size_t index,
        widget_t* GUI,
        const property_t& instance,
        [[maybe_unused]] AzToolsFramework::InstanceDataNode* node)
    {
        QSignalBlocker blocker(GUI);

        // Rebuilt on every read rather than cached: layer names are editable at any
        // time, and one handler instance serves every panel showing the property.
        GUI->clearElements();
        for (const AZStd::string& layerName : GetLayerNames())
        {
            GUI->Add(layerName);
        }
        GUI->setValue(GetNameFromLayer(instance));

        return false;
    }

    AzPhysics::CollisionLayer CollisionLayerWidget::GetLayerFromName(const AZStd::string& layerName)
    {
        AzPhysics::CollisionLayer layer = AzPhysics::CollisionLayer::Default;
        if (const AzPhysics::CollisionConfiguration* collisionConfiguration = GetCollisionConfiguration())
        {
            collisionConfiguration->m_collisionLayers.TryGetLayer(layerName, layer);
        }
        return layer;
    }

    AZStd::string CollisionLayerWidget::GetNameFromLayer(const AzPhysics::CollisionLayer& layer)
    {
        if (const AzPhysics::CollisionConfiguration* collisionConfiguration = GetCollisionConfiguration())
        {
            return collisionConfiguration->m_collisionLayers.GetName(layer);
        }
        return {};
    }

    AZStd::vector<AZStd::string> CollisionLayerWidget::GetLayerNames()
    {
        AZStd::vector<AZStd::string> layerNames;
        if (const AzPhysics::CollisionConfiguration* collisionConfiguration = GetCollisionConfiguration())
        {
            // Unnamed slots are the unused tail of the fixed 64-entry array, not gaps:
            // showing them would offer layers that cannot be referred to by name.
            for (const AZStd::string& layerName : collisionConfiguration->m_collisionLayers.GetNames())
            {
                if (!layerName.empty())
                {
                    layerNames.push_back(layerName);
                }
            }
        }
        return layerNames;
    }
} // namespace JoltPhysics::Editor
