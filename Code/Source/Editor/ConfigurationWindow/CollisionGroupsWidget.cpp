#include <Editor/ConfigurationWindow/CollisionGroupsWidget.h>

#include <QHBoxLayout>
#include <QHeaderView>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>

namespace JoltPhysics::Editor
{
    namespace
    {
        constexpr int GroupNameColumn = 0;
        constexpr int FirstLayerColumn = 1;
    } // namespace

    CollisionGroupsWidget::CollisionGroupsWidget(QWidget* parent)
        : QWidget(parent)
    {
        m_table = new QTableWidget(this);
        m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
        m_table->setSelectionMode(QAbstractItemView::SingleSelection);
        m_table->verticalHeader()->setVisible(false);
        connect(m_table, &QTableWidget::itemChanged, this, &CollisionGroupsWidget::OnItemChanged);

        auto* addButton = new QPushButton(tr("Add Group"), this);
        connect(addButton, &QPushButton::clicked, this, &CollisionGroupsWidget::OnAddGroup);

        auto* removeButton = new QPushButton(tr("Remove Selected"), this);
        connect(removeButton, &QPushButton::clicked, this, &CollisionGroupsWidget::OnRemoveSelectedGroup);

        auto* buttonLayout = new QHBoxLayout();
        buttonLayout->addWidget(addButton);
        buttonLayout->addWidget(removeButton);
        buttonLayout->addStretch();

        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->addLayout(buttonLayout);
        layout->addWidget(m_table);
    }

    void CollisionGroupsWidget::SetValue(
        const AzPhysics::CollisionGroups& groups, const AzPhysics::CollisionLayers& layers)
    {
        m_groups = groups;
        m_layers = layers;
        RebuildTable();
    }

    const AzPhysics::CollisionGroups& CollisionGroupsWidget::GetValue() const
    {
        return m_groups;
    }

    void CollisionGroupsWidget::RebuildTable()
    {
        m_rebuilding = true;

        m_columnLayers.clear();
        const auto& layerNames = m_layers.GetNames();
        for (AZ::u64 i = 0; i < layerNames.size(); ++i)
        {
            if (!layerNames[i].empty())
            {
                m_columnLayers.push_back(AzPhysics::CollisionLayer(static_cast<AZ::u8>(i)));
            }
        }

        m_table->clear();
        m_table->setColumnCount(FirstLayerColumn + static_cast<int>(m_columnLayers.size()));
        QStringList headers;
        headers << tr("Group Name");
        for (const auto& layer : m_columnLayers)
        {
            headers << QString::fromUtf8(m_layers.GetName(layer).c_str());
        }
        m_table->setHorizontalHeaderLabels(headers);

        const auto& presets = m_groups.GetPresets();
        m_rowGroups.clear();
        m_table->setRowCount(static_cast<int>(presets.size()));

        for (int row = 0; row < static_cast<int>(presets.size()); ++row)
        {
            const auto& preset = presets[row];
            m_rowGroups.push_back(preset.m_id);

            auto* nameItem = new QTableWidgetItem(QString::fromUtf8(preset.m_name.c_str()));
            Qt::ItemFlags nameFlags = Qt::ItemIsSelectable | Qt::ItemIsEnabled;
            if (!preset.m_readOnly)
            {
                nameFlags |= Qt::ItemIsEditable;
            }
            nameItem->setFlags(nameFlags);
            m_table->setItem(row, GroupNameColumn, nameItem);

            for (int column = 0; column < static_cast<int>(m_columnLayers.size()); ++column)
            {
                auto* checkItem = new QTableWidgetItem();
                Qt::ItemFlags checkFlags = Qt::ItemIsSelectable | Qt::ItemIsUserCheckable;
                if (!preset.m_readOnly)
                {
                    checkFlags |= Qt::ItemIsEnabled;
                }
                checkItem->setFlags(checkFlags);
                checkItem->setCheckState(
                    preset.m_group.IsSet(m_columnLayers[column]) ? Qt::Checked : Qt::Unchecked);
                m_table->setItem(row, FirstLayerColumn + column, checkItem);
            }
        }

        m_table->resizeColumnsToContents();

        m_rebuilding = false;
    }

    void CollisionGroupsWidget::OnItemChanged(QTableWidgetItem* item)
    {
        if (m_rebuilding || item == nullptr)
        {
            return;
        }

        const int row = item->row();
        if (row < 0 || row >= static_cast<int>(m_rowGroups.size()))
        {
            return;
        }
        const AzPhysics::CollisionGroups::Id groupId = m_rowGroups[row];

        if (item->column() == GroupNameColumn)
        {
            const AZStd::string newName(item->text().toUtf8().constData());
            const AZStd::string oldName = m_groups.FindGroupNameById(groupId);
            if (newName == oldName)
            {
                return;
            }

            // Groups are referenced by name from script and code, so an empty or
            // duplicate name is rejected rather than repaired.
            if (newName.empty() || IsGroupNameTaken(newName))
            {
                RebuildTable();
                return;
            }

            m_groups.SetGroupName(groupId, newName);
        }
        else
        {
            const int layerColumn = item->column() - FirstLayerColumn;
            if (layerColumn < 0 || layerColumn >= static_cast<int>(m_columnLayers.size()))
            {
                return;
            }
            m_groups.SetLayer(groupId, m_columnLayers[layerColumn], item->checkState() == Qt::Checked);
        }

        emit onValueChanged(m_groups);
    }

    void CollisionGroupsWidget::OnAddGroup()
    {
        // New groups start colliding with everything: the common case is carving a few
        // layers out, not building up from nothing.
        m_groups.CreateGroup(MakeUniqueGroupName("NewGroup"), AzPhysics::CollisionGroup::All);
        RebuildTable();
        emit onValueChanged(m_groups);
    }

    void CollisionGroupsWidget::OnRemoveSelectedGroup()
    {
        const int row = m_table->currentRow();
        if (row < 0 || row >= static_cast<int>(m_rowGroups.size()))
        {
            return;
        }

        const AzPhysics::CollisionGroups::Id groupId = m_rowGroups[row];
        for (const auto& preset : m_groups.GetPresets())
        {
            if (preset.m_id == groupId)
            {
                if (preset.m_readOnly)
                {
                    return;
                }
                break;
            }
        }

        m_groups.DeleteGroup(groupId);
        RebuildTable();
        emit onValueChanged(m_groups);
    }

    AZStd::string CollisionGroupsWidget::MakeUniqueGroupName(const AZStd::string& base) const
    {
        if (!IsGroupNameTaken(base))
        {
            return base;
        }
        for (int suffix = 1;; ++suffix)
        {
            AZStd::string candidate = AZStd::string::format("%s%d", base.c_str(), suffix);
            if (!IsGroupNameTaken(candidate))
            {
                return candidate;
            }
        }
    }

    bool CollisionGroupsWidget::IsGroupNameTaken(const AZStd::string& name) const
    {
        for (const auto& preset : m_groups.GetPresets())
        {
            if (preset.m_name == name)
            {
                return true;
            }
        }
        return false;
    }
} // namespace JoltPhysics::Editor
