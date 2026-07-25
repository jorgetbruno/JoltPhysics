#pragma once

#if !defined(Q_MOC_RUN)
#include <AzCore/Memory/SystemAllocator.h>
#include <AzFramework/Physics/Collision/CollisionGroups.h>
#include <AzToolsFramework/UI/PropertyEditor/PropertyEditorAPI.h>
#include <AzToolsFramework/UI/PropertyEditor/PropertyStringComboBoxCtrl.hxx>
#endif

namespace JoltPhysics::Editor
{
    //! Renders AzPhysics::CollisionGroups::Id ("Collides With") as a preset-name
    //! dropdown instead of a raw Uuid. See CollisionLayerWidget for why this handler
    //! exists and where the name Physics::Edit::CollisionGroupSelector comes from.
    class CollisionGroupWidget
        : public QObject
        , public AzToolsFramework::PropertyHandler<AzPhysics::CollisionGroups::Id, AzToolsFramework::PropertyStringComboBoxCtrl>
    {
        Q_OBJECT

    public:
        AZ_CLASS_ALLOCATOR(CollisionGroupWidget, AZ::SystemAllocator);

        CollisionGroupWidget() = default;

        AZ::u32 GetHandlerName() const override;
        QWidget* CreateGUI(QWidget* parent) override;
        bool IsDefaultHandler() const override;

        void ConsumeAttribute(
            widget_t* widget,
            AZ::u32 attrib,
            AzToolsFramework::PropertyAttributeReader* attrValue,
            const char* debugName) override;

        void WriteGUIValuesIntoProperty(
            size_t index, widget_t* GUI, property_t& instance, AzToolsFramework::InstanceDataNode* node) override;
        bool ReadValuesIntoGUI(
            size_t index, widget_t* GUI, const property_t& instance, AzToolsFramework::InstanceDataNode* node) override;

    private:
        static AzPhysics::CollisionGroups::Id GetGroupFromName(const AZStd::string& groupName);
        static AZStd::string GetNameFromGroup(const AzPhysics::CollisionGroups::Id& groupId);
        static AZStd::vector<AZStd::string> GetGroupNames();
    };
} // namespace JoltPhysics::Editor
