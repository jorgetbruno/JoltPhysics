#include <Editor/PropertyHandlers/CollisionGroupWidget.h>

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
                AzToolsFramework::PropertyEditorGUIMessages::Bus::Broadcast(
                    &AzToolsFramework::PropertyEditorGUIMessages::RequestWrite, picker);
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
        if (const AzPhysics::CollisionConfiguration* collisionConfiguration = GetCollisionConfiguration())
        {
            return collisionConfiguration->m_collisionGroups.FindGroupNameById(groupId);
        }
        return {};
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
