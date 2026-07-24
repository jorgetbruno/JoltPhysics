#include <Editor/Components/EditorJoltMutableCompoundColliderComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltStaticCompoundColliderComponent.h>

namespace JoltPhysics
{
    void EditorJoltMutableCompoundColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltColliderComponentBase::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltMutableCompoundColliderComponent, EditorJoltColliderComponentBase>()
                ->Version(1)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltMutableCompoundColliderComponent>(
                    "Jolt Mutable Compound Collider",
                    "Compound collider that supports adding and removing child collider entities at runtime (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void EditorJoltMutableCompoundColliderComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltMutableCompoundColliderComponent>())
        {
            component->GetColliderConfiguration() = *m_colliderConfiguration;
        }
    }

} // namespace JoltPhysics
