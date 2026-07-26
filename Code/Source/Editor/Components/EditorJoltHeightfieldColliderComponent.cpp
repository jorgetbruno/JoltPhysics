#include <Editor/Components/EditorJoltHeightfieldColliderComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltHeightfieldColliderComponent.h>

namespace JoltPhysics
{
    void EditorJoltHeightfieldColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltColliderComponentBase::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltHeightfieldColliderComponent, EditorJoltColliderComponentBase>()
                ->Version(1)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltHeightfieldColliderComponent>(
                    "Jolt Heightfield Collider",
                    "Heightfield collider fed by a Physics::HeightfieldProviderBus implementation (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void EditorJoltHeightfieldColliderComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltHeightfieldColliderComponent>())
        {
            component->GetColliderConfiguration() = m_colliderConfiguration;
        }
    }

} // namespace JoltPhysics
