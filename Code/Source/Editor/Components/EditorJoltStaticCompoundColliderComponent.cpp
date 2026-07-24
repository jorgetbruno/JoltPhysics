#include <Editor/Components/EditorJoltStaticCompoundColliderComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltStaticCompoundColliderComponent.h>

namespace JoltPhysics
{
    void EditorJoltStaticCompoundColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltColliderComponentBase::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltStaticCompoundColliderComponent, EditorJoltColliderComponentBase>()
                ->Version(1)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltStaticCompoundColliderComponent>(
                    "Jolt Static Compound Collider",
                    "Combines the colliders of all child entities into a single compound collider (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void EditorJoltStaticCompoundColliderComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltStaticCompoundColliderComponent>())
        {
            component->GetColliderConfiguration() = *m_colliderConfiguration;
        }
    }

} // namespace JoltPhysics
