#pragma once

#if !defined(Q_MOC_RUN)
#include <AzCore/Memory/SystemAllocator.h>
#include <AzFramework/Physics/Collision/CollisionLayers.h>
#include <AzToolsFramework/UI/PropertyEditor/PropertyEditorAPI_Internals.h>
#endif

#include <QWidget>

namespace AzToolsFramework
{
    class ReflectedPropertyEditor;
}

namespace JoltPhysics::Editor
{
    //! Edits the 64 collision layer names via a reflected property editor, keeping
    //! them unique (collision layers are referred to by name everywhere else) and
    //! layer 0's name fixed.
    class CollisionLayersWidget
        : public QWidget
        , private AzToolsFramework::IPropertyEditorNotify
    {
        Q_OBJECT

    public:
        AZ_CLASS_ALLOCATOR(CollisionLayersWidget, AZ::SystemAllocator);

        explicit CollisionLayersWidget(QWidget* parent = nullptr);

        void SetValue(const AzPhysics::CollisionLayers& layers);
        const AzPhysics::CollisionLayers& GetValue() const;

    signals:
        void onValueChanged(const AzPhysics::CollisionLayers& newValue);

    private:
        // AzToolsFramework::IPropertyEditorNotify
        void BeforePropertyModified(AzToolsFramework::InstanceDataNode* node) override;
        void AfterPropertyModified(AzToolsFramework::InstanceDataNode* node) override;
        void SetPropertyEditingActive(AzToolsFramework::InstanceDataNode* node) override;
        void SetPropertyEditingComplete(AzToolsFramework::InstanceDataNode* node) override;
        void SealUndoStack() override;

        //! Returns true if any name had to be adjusted.
        bool EnforceValidLayerNames();

        AzToolsFramework::ReflectedPropertyEditor* m_propertyEditor = nullptr;
        AzPhysics::CollisionLayers m_value;

        //! Layer 0 is the engine-wide default layer; renaming it would orphan every
        //! collider that never chose a layer, so its name is pinned to what it was
        //! when the widget was populated.
        AZStd::string m_defaultLayerName;
    };
} // namespace JoltPhysics::Editor
