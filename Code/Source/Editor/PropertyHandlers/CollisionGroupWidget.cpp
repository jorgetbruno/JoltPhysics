#include <Editor/PropertyHandlers/CollisionGroupWidget.h>

#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Physics/PropertyTypes.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <JoltPhysics/Configuration/JoltConfiguration.h>

#include <Editor/ConfigurationWindow/JoltConfigurationWindowBus.h>
#include <Editor/PropertyHandlers/CollisionPropertyUtils.h>

#include <QComboBox>
#include <QSignalBlocker>
#include <QToolButton>

namespace JoltPhysics::Editor
{
    AZ::u32 CollisionGroupWidget::GetHandlerName() const
    {
        return Physics::Edit::CollisionGroupSelector;
    }

    bool CollisionGroupWidget::IsDefaultHandler() const
    {
        return true;
    }

    QWidget* CollisionGroupWidget::CreateGUI(QWidget* parent)
    {
        auto* picker = aznew widget_t(parent);

        picker->GetComboBox()->setToolTip("Which collision layers this object collides with.");

        picker->GetEditButton()->setVisible(true);
        picker->GetEditButton()->setToolTip("Edit collision groups");
        connect(picker->GetEditButton(), &QToolButton::clicked, this, []()
            {
                AzToolsFramework::EditorRequests::Bus::Broadcast(
                    &AzToolsFramework::EditorRequests::OpenViewPane, ConfigurationWindowName);
                JoltConfigurationWindowRequestBus::Broadcast(
                    &JoltConfigurationWindowRequests::ShowCollisionGroupsTab);
            });

        connect(picker, &widget_t::valueChanged, this, [picker]()
            {
                // RequestWrite opens the property-modification undo batch;
                // OnEditingFinished closes it. A combo box edit is atomic, so both are
                // sent together (as the engine's GenericComboBoxHandler does). Without
                // the second the batch stays open: undo warns "Cannot Undo while
                // Recording" and the level can no longer be saved.
                AzToolsFramework::PropertyEditorGUIMessages::Bus::Broadcast(
                    &AzToolsFramework::PropertyEditorGUIMessages::RequestWrite, picker);
                AzToolsFramework::PropertyEditorGUIMessages::Bus::Broadcast(
                    &AzToolsFramework::PropertyEditorGUIMessages::OnEditingFinished, picker);
            });

        return picker;
    }

    void CollisionGroupWidget::ConsumeAttribute(
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

    void CollisionGroupWidget::WriteGUIValuesIntoProperty(
        [[maybe_unused]] size_t index,
        widget_t* GUI,
        property_t& instance,
        [[maybe_unused]] AzToolsFramework::InstanceDataNode* node)
    {
        instance = GetGroupFromName(GUI->value());
    }

    bool CollisionGroupWidget::ReadValuesIntoGUI(
        [[maybe_unused]] size_t index,
        widget_t* GUI,
        const property_t& instance,
        [[maybe_unused]] AzToolsFramework::InstanceDataNode* node)
    {
        QSignalBlocker blocker(GUI);

        GUI->clearElements();
        for (const AZStd::string& groupName : GetGroupNames())
        {
            GUI->Add(groupName);
        }
        GUI->setValue(GetNameFromGroup(instance));

        return false;
    }

    AzPhysics::CollisionGroups::Id CollisionGroupWidget::GetGroupFromName(const AZStd::string& groupName)
    {
        if (const AzPhysics::CollisionConfiguration* collisionConfiguration = GetCollisionConfiguration())
        {
            return collisionConfiguration->m_collisionGroups.FindGroupIdByName(groupName);
        }
        return AzPhysics::CollisionGroups::Id();
    }

    AZStd::string CollisionGroupWidget::GetNameFromGroup(const AzPhysics::CollisionGroups::Id& groupId)
    {
        const AzPhysics::CollisionConfiguration* collisionConfiguration = GetCollisionConfiguration();
        if (!collisionConfiguration)
        {
            return {};
        }

        AZStd::string name = collisionConfiguration->m_collisionGroups.FindGroupNameById(groupId);
        if (!name.empty())
        {
            return name;
        }

        // An id that resolves to nothing - an unset one on a collider that was never
        // given a group, or one left behind by a deleted preset - is not an error:
        // AzPhysics answers CollisionGroup::All for any id it cannot find, so that is
        // what the object really collides with and that is what has to be shown.
        //
        // Showing a blank instead let the control disagree with the simulation, and
        // worse, it made picking the entry the control appeared to be on count as no
        // change, so nothing was written and the choice silently did nothing.
        return collisionConfiguration->m_collisionGroups.FindGroupNameById(JoltSystemConfiguration::AllGroupId);
    }

    AZStd::vector<AZStd::string> CollisionGroupWidget::GetGroupNames()
    {
        AZStd::vector<AZStd::string> groupNames;
        if (const AzPhysics::CollisionConfiguration* collisionConfiguration = GetCollisionConfiguration())
        {
            for (const AzPhysics::CollisionGroups::Preset& preset : collisionConfiguration->m_collisionGroups.GetPresets())
            {
                groupNames.push_back(preset.m_name);
            }
        }
        return groupNames;
    }
} // namespace JoltPhysics::Editor
