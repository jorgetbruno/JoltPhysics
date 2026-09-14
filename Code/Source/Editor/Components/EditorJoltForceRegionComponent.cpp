#include <Editor/Components/EditorJoltForceRegionComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Editor/Components/EditorJoltForceRegionDraw.h>
#include <ForceRegion/JoltForceRegionComponent.h>

namespace JoltPhysics
{
    void EditorJoltForceRegionComponent::Reflect(AZ::ReflectContext* context)
    {
        // Idempotent: the runtime component reflects it too, in whichever order the
        // descriptors come. See the guard in JoltForceRegion::Reflect.
        JoltForceRegion::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltForceRegionComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(1)
                ->Field("ForceRegion", &EditorJoltForceRegionComponent::m_forceRegion)
                ->Field("WindTag", &EditorJoltForceRegionComponent::m_windTag)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltForceRegionComponent>("Jolt Force Region",
                    "Applies forces to bodies inside a trigger collider (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltForceRegionComponent::m_forceRegion,
                        "Forces", "Forces applied to every body inside the region.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltForceRegionComponent::m_windTag,
                        "Wind tag", "Publishes this region through the engine's wind interface under this tag. "
                        "Match it against the global or local wind tag in the Jolt configuration; leave empty "
                        "for a region that is not wind.")
                    ;
            }
        }
    }

    void EditorJoltForceRegionComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltForceRegionService"));
    }

    void EditorJoltForceRegionComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltForceRegionService"));
    }

    void EditorJoltForceRegionComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        // A region is a trigger volume, so it needs geometry to be a volume of.
        required.push_back(AZ_CRC_CE("JoltColliderService"));
    }

    void EditorJoltForceRegionComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<JoltForceRegionComponent>(m_forceRegion, m_windTag);
    }

    void EditorJoltForceRegionComponent::Activate()
    {
        AzToolsFramework::Components::EditorComponentBase::Activate();
        AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(GetEntityId());
    }

    void EditorJoltForceRegionComponent::Deactivate()
    {
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();
        AzToolsFramework::Components::EditorComponentBase::Deactivate();
    }

    void EditorJoltForceRegionComponent::DisplayEntityViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        EditorDebugDraw::DrawForceRegionPreview(debugDisplay, m_forceRegion, worldTransform);
    }
} // namespace JoltPhysics
