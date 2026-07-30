#include <Editor/Pipeline/JoltMeshBehavior.h>
#include <Editor/Pipeline/JoltMeshGroup.h>

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <SceneAPI/SceneCore/Containers/Scene.h>
#include <SceneAPI/SceneCore/Containers/SceneGraph.h>
#include <SceneAPI/SceneCore/Containers/Utilities/Filters.h>
#include <SceneAPI/SceneCore/Containers/Utilities/SceneGraphUtilities.h>
#include <SceneAPI/SceneCore/Containers/Views/FilterIterator.h>
#include <SceneAPI/SceneCore/DataTypes/DataTypeUtilities.h>
#include <SceneAPI/SceneCore/DataTypes/GraphData/IMeshData.h>
#include <SceneAPI/SceneCore/Events/GraphMetaInfoBus.h>
#include <SceneAPI/SceneCore/Utilities/SceneGraphSelector.h>
#include <SceneAPI/SceneData/Rules/CoordinateSystemRule.h>

namespace JoltPhysics::Pipeline
{
    void JoltMeshBehavior::Activate()
    {
        AZ::SceneAPI::Events::ManifestMetaInfoBus::Handler::BusConnect();
        AZ::SceneAPI::Events::AssetImportRequestBus::Handler::BusConnect();
    }

    void JoltMeshBehavior::Deactivate()
    {
        AZ::SceneAPI::Events::AssetImportRequestBus::Handler::BusDisconnect();
        AZ::SceneAPI::Events::ManifestMetaInfoBus::Handler::BusDisconnect();
    }

    void JoltMeshBehavior::Reflect(AZ::ReflectContext* context)
    {
        JoltMeshGroup::Reflect(context);

        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltMeshBehavior, BehaviorComponent>()->Version(1);
        }
    }

    void JoltMeshBehavior::GetCategoryAssignments(CategoryRegistrationList& categories, const AZ::SceneAPI::Containers::Scene& scene)
    {
        if (AZ::SceneAPI::Utilities::DoesSceneGraphContainDataLike<AZ::SceneAPI::DataTypes::IMeshData>(scene, false))
        {
            categories.emplace_back("Jolt Physics", JoltMeshGroup::TYPEINFO_Uuid(), s_meshBehaviorPreferredTabOrder);
        }
    }

    void JoltMeshBehavior::GetAvailableModifiers(
        AZ::SceneAPI::Events::ManifestMetaInfo::ModifiersList& modifiers,
        [[maybe_unused]] const AZ::SceneAPI::Containers::Scene& scene,
        const AZ::SceneAPI::DataTypes::IManifestObject& target)
    {
        if (!target.RTTI_IsTypeOf(JoltMeshGroup::TYPEINFO_Uuid()))
        {
            return;
        }

        const JoltMeshGroup* group = azrtti_cast<const JoltMeshGroup*>(&target);
        const AZ::SceneAPI::Containers::RuleContainer& rules = group->GetRuleContainerConst();

        AZStd::unordered_set<AZ::Uuid> existingRules;
        const size_t ruleCount = rules.GetRuleCount();
        for (size_t i = 0; i < ruleCount; ++i)
        {
            existingRules.insert(rules.GetRule(i)->RTTI_GetType());
        }
        if (existingRules.find(azrtti_typeid<AZ::SceneAPI::SceneData::CoordinateSystemRule>()) == existingRules.end())
        {
            modifiers.push_back(azrtti_typeid<AZ::SceneAPI::SceneData::CoordinateSystemRule>());
        }
    }

    void JoltMeshBehavior::InitializeObject(const AZ::SceneAPI::Containers::Scene& scene, AZ::SceneAPI::DataTypes::IManifestObject& target)
    {
        if (!target.RTTI_IsTypeOf(JoltMeshGroup::TYPEINFO_Uuid()))
        {
            return;
        }

        JoltMeshGroup* group = azrtti_cast<JoltMeshGroup*>(&target);
        group->SetName(AZ::SceneAPI::DataTypes::Utilities::CreateUniqueName<JoltMeshGroup>(scene.GetName(), scene.GetManifest()));

        const AZ::SceneAPI::Containers::SceneGraph& graph = scene.GetGraph();

        // Gather the nodes that should be selected by default. Nodes tagged with the
        // "PhysicsMesh" virtual type are the ones the scene's own behavior marked as
        // collision geometry (e.g. a "_phys" render mesh), so they are the sensible
        // default selection for a physics mesh group.
        AZ::SceneAPI::DataTypes::ISceneNodeSelectionList& nodeSelectionList = group->GetSceneNodeSelectionList();
        AZ::SceneAPI::Utilities::SceneGraphSelector::UnselectAll(graph, nodeSelectionList);
        AZ::SceneAPI::Containers::SceneGraph::ContentStorageConstData graphContent = graph.GetContentStorage();
        auto view = AZ::SceneAPI::Containers::Views::MakeFilterView(graphContent,
            AZ::SceneAPI::Containers::DerivedTypeFilter<AZ::SceneAPI::DataTypes::IMeshData>());
        for (auto iter = view.begin(), iterEnd = view.end(); iter != iterEnd; ++iter)
        {
            AZ::SceneAPI::Containers::SceneGraph::NodeIndex nodeIndex = graph.ConvertToNodeIndex(iter.GetBaseIterator());

            AZ::SceneAPI::Events::GraphMetaInfo::VirtualTypesSet types;
            AZ::SceneAPI::Events::GraphMetaInfoBus::Broadcast(&AZ::SceneAPI::Events::GraphMetaInfoBus::Events::GetVirtualTypes, types, scene, nodeIndex);

            if (types.count(AZ_CRC_CE("PhysicsMesh")) == 1)
            {
                nodeSelectionList.AddSelectedNode(graph.GetNodeName(nodeIndex).GetPath());
            }
        }

        // Update list of materials slots after the group's node selection list has been gathered.
        group->SetSceneGraph(&graph);
        group->UpdateMaterialSlots();
    }

    AZ::SceneAPI::Events::ProcessingResult JoltMeshBehavior::UpdateManifest(AZ::SceneAPI::Containers::Scene& scene, ManifestAction action,
        RequestingApplication /*requester*/)
    {
        if (action == ManifestAction::ConstructDefault)
        {
            return BuildDefault(scene);
        }
        else if (action == ManifestAction::Update)
        {
            return UpdateJoltMeshGroups(scene);
        }
        else
        {
            return AZ::SceneAPI::Events::ProcessingResult::Ignored;
        }
    }

    AZ::SceneAPI::Events::ProcessingResult JoltMeshBehavior::BuildDefault(AZ::SceneAPI::Containers::Scene& scene) const
    {
        if (!AZ::SceneAPI::Utilities::DoesSceneGraphContainDataLike<AZ::SceneAPI::DataTypes::IMeshData>(scene, true))
        {
            return AZ::SceneAPI::Events::ProcessingResult::Ignored;
        }

        AZStd::shared_ptr<JoltMeshGroup> group = AZStd::make_shared<JoltMeshGroup>();

        // This is a group that's generated automatically so may not be saved to disk but would need to be recreated
        //      in the same way again. To guarantee the same uuid, generate a stable one instead.
        group->OverrideId(AZ::SceneAPI::DataTypes::Utilities::CreateStableUuid(scene, JoltMeshGroup::TYPEINFO_Uuid()));

        group->SetSceneGraph(&scene.GetGraph());

        AZ::SceneAPI::Events::ManifestMetaInfoBus::Broadcast(&AZ::SceneAPI::Events::ManifestMetaInfoBus::Events::InitializeObject, scene, *group);
        scene.GetManifest().AddEntry(AZStd::move(group));

        return AZ::SceneAPI::Events::ProcessingResult::Success;
    }

    AZ::SceneAPI::Events::ProcessingResult JoltMeshBehavior::UpdateJoltMeshGroups(AZ::SceneAPI::Containers::Scene& scene) const
    {
        bool updated = false;
        AZ::SceneAPI::Containers::SceneManifest& manifest = scene.GetManifest();
        auto valueStorage = manifest.GetValueStorage();
        auto view = AZ::SceneAPI::Containers::MakeDerivedFilterView<JoltMeshGroup>(valueStorage);
        for (JoltMeshGroup& group : view)
        {
            if (group.GetName().empty())
            {
                group.SetName(AZ::SceneAPI::DataTypes::Utilities::CreateUniqueName<JoltMeshGroup>(scene.GetName(), scene.GetManifest()));
            }

            AZ::SceneAPI::Utilities::SceneGraphSelector::UpdateNodeSelection(scene.GetGraph(), group.GetSceneNodeSelectionList());

            // Update list of materials slots after the group's node selection list has been updated.
            group.SetSceneGraph(&scene.GetGraph());
            group.UpdateMaterialSlots();

            updated = true;
        }

        return updated ? AZ::SceneAPI::Events::ProcessingResult::Success : AZ::SceneAPI::Events::ProcessingResult::Ignored;
    }

} // namespace JoltPhysics::Pipeline
