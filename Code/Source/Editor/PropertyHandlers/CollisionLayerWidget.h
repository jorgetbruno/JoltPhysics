#pragma once

#if !defined(Q_MOC_RUN)
#include <AzCore/Memory/SystemAllocator.h>
#include <AzFramework/Physics/Collision/CollisionLayers.h>
#include <AzToolsFramework/UI/PropertyEditor/PropertyEditorAPI.h>
#include <AzToolsFramework/UI/PropertyEditor/PropertyStringComboBoxCtrl.hxx>
#endif

namespace JoltPhysics::Editor
{
    //! Renders AzPhysics::CollisionLayer as a name dropdown instead of a raw index.
    //!
    //! The handler name is Physics::Edit::CollisionLayerSelector, declared by
    //! AzFramework rather than by any backend: AzFramework's own reflection (for
    //! example Physics::CharacterConfiguration) asks for a handler by that name, and
    //! whichever physics gem is enabled is expected to supply one. Without this the
    //! field falls back to the default struct rendering, which is why the dropdown is
    //! missing in projects that have no PhysX gem.
    //!
    //! The layer names come from AzPhysics::SystemInterface, so nothing here is
    //! specific to Jolt.
    class CollisionLayerWidget
        : public QObject
        , public AzToolsFramework::PropertyHandler<AzPhysics::CollisionLayer, AzToolsFramework::PropertyStringComboBoxCtrl>
    {
        Q_OBJECT

    public:
        AZ_CLASS_ALLOCATOR(CollisionLayerWidget, AZ::SystemAllocator);

        CollisionLayerWidget() = default;

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
        static AzPhysics::CollisionLayer GetLayerFromName(const AZStd::string& layerName);
        static AZStd::string GetNameFromLayer(const AzPhysics::CollisionLayer& layer);
        static AZStd::vector<AZStd::string> GetLayerNames();
    };
} // namespace JoltPhysics::Editor
