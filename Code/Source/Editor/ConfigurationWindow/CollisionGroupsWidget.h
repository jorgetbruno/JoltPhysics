#pragma once

#if !defined(Q_MOC_RUN)
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/std/containers/vector.h>
#include <AzFramework/Physics/Collision/CollisionGroups.h>
#include <AzFramework/Physics/Collision/CollisionLayers.h>
#endif

#include <QWidget>

class QTableWidget;
class QTableWidgetItem;

namespace JoltPhysics::Editor
{
    //! Edits collision group presets as a matrix: one row per group, one checkbox
    //! column per named collision layer, mirroring the PhysX Configuration window's
    //! Groups view. Read-only presets (such as All and None) are shown but locked.
    class CollisionGroupsWidget : public QWidget
    {
        Q_OBJECT

    public:
        AZ_CLASS_ALLOCATOR(CollisionGroupsWidget, AZ::SystemAllocator);

        explicit CollisionGroupsWidget(QWidget* parent = nullptr);

        //! The layers are needed alongside the groups to label and index the
        //! checkbox columns.
        void SetValue(const AzPhysics::CollisionGroups& groups, const AzPhysics::CollisionLayers& layers);
        const AzPhysics::CollisionGroups& GetValue() const;

    signals:
        void onValueChanged(const AzPhysics::CollisionGroups& newValue);

    private:
        void RebuildTable();
        void OnItemChanged(QTableWidgetItem* item);
        void OnAddGroup();
        void OnRemoveSelectedGroup();

        AZStd::string MakeUniqueGroupName(const AZStd::string& base) const;
        bool IsGroupNameTaken(const AZStd::string& name) const;

        QTableWidget* m_table = nullptr;

        AzPhysics::CollisionGroups m_groups;
        AzPhysics::CollisionLayers m_layers;

        //! Column index -> collision layer, for the checkbox columns (column 0 is the
        //! group name).
        AZStd::vector<AzPhysics::CollisionLayer> m_columnLayers;

        //! Row index -> group id.
        AZStd::vector<AzPhysics::CollisionGroups::Id> m_rowGroups;

        //! Suppresses itemChanged handling while the table is being repopulated.
        bool m_rebuilding = false;
    };
} // namespace JoltPhysics::Editor
