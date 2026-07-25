#include <Editor/ConfigurationWindow/CollisionLayersWidget.h>

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/std/string/string.h>
#include <AzToolsFramework/UI/PropertyEditor/ReflectedPropertyEditor.hxx>

#include <QVBoxLayout>

namespace JoltPhysics::Editor
{
    CollisionLayersWidget::CollisionLayersWidget(QWidget* parent)
        : QWidget(parent)
    {
        AZ::SerializeContext* serializeContext = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(
            serializeContext, &AZ::ComponentApplicationBus::Events::GetSerializeContext);
        AZ_Assert(serializeContext, "CollisionLayersWidget: no serialize context");

        m_propertyEditor = new AzToolsFramework::ReflectedPropertyEditor(this);
        m_propertyEditor->Setup(serializeContext, this, /*enableScrollbars*/ true);

        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->addWidget(m_propertyEditor);

        m_propertyEditor->AddInstance(&m_value);
        m_propertyEditor->InvalidateAll();
        m_propertyEditor->ExpandAll();
    }

    void CollisionLayersWidget::SetValue(const AzPhysics::CollisionLayers& layers)
    {
        m_value = layers;
        m_defaultLayerName = m_value.GetNames()[AzPhysics::CollisionLayer::Default.GetIndex()];
        m_propertyEditor->InvalidateValues();
    }

    const AzPhysics::CollisionLayers& CollisionLayersWidget::GetValue() const
    {
        return m_value;
    }

    void CollisionLayersWidget::BeforePropertyModified(AzToolsFramework::InstanceDataNode* /*node*/)
    {
    }

    void CollisionLayersWidget::AfterPropertyModified(AzToolsFramework::InstanceDataNode* /*node*/)
    {
        if (EnforceValidLayerNames())
        {
            m_propertyEditor->InvalidateValues();
        }
        emit onValueChanged(m_value);
    }

    void CollisionLayersWidget::SetPropertyEditingActive(AzToolsFramework::InstanceDataNode* /*node*/)
    {
    }

    void CollisionLayersWidget::SetPropertyEditingComplete(AzToolsFramework::InstanceDataNode* /*node*/)
    {
    }

    void CollisionLayersWidget::SealUndoStack()
    {
    }

    bool CollisionLayersWidget::EnforceValidLayerNames()
    {
        bool adjusted = false;
        const auto& names = m_value.GetNames();

        const AZ::u8 defaultIndex = AzPhysics::CollisionLayer::Default.GetIndex();
        if (names[defaultIndex] != m_defaultLayerName)
        {
            m_value.SetName(static_cast<AZ::u64>(defaultIndex), m_defaultLayerName);
            adjusted = true;
        }

        // Duplicate names would make name-to-layer lookups ambiguous; suffix any
        // repeat until it is unique. Empty names are fine: they mark unused slots and
        // are hidden from the layer dropdowns.
        for (AZ::u64 i = 0; i < names.size(); ++i)
        {
            if (names[i].empty())
            {
                continue;
            }

            auto isTakenByEarlierLayer = [&names, i](const AZStd::string& candidate)
            {
                for (AZ::u64 j = 0; j < i; ++j)
                {
                    if (!names[j].empty() && names[j] == candidate)
                    {
                        return true;
                    }
                }
                return false;
            };

            if (isTakenByEarlierLayer(names[i]))
            {
                AZStd::string unique;
                for (int suffix = 1;; ++suffix)
                {
                    unique = AZStd::string::format("%s_%d", names[i].c_str(), suffix);
                    if (!isTakenByEarlierLayer(unique))
                    {
                        break;
                    }
                }
                m_value.SetName(i, unique);
                adjusted = true;
            }
        }

        return adjusted;
    }
} // namespace JoltPhysics::Editor
