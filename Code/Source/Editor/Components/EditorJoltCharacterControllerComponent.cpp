#include <Editor/Components/EditorJoltCharacterControllerComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltCharacterControllerComponent.h>
#include <Editor/Components/EditorJoltDebugDrawUtils.h>

namespace JoltPhysics
{
    void EditorJoltCharacterControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->RegisterGenericType<AZStd::shared_ptr<Physics::ShapeConfiguration>>();

            serializeContext->Class<EditorJoltCharacterControllerComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(1)
                ->Field("CharacterConfiguration", &EditorJoltCharacterControllerComponent::m_characterConfig)
                ->Field("ShapeConfiguration", &EditorJoltCharacterControllerComponent::m_shapeConfig)
                ->Field("Height", &EditorJoltCharacterControllerComponent::m_height)
                ->Field("Radius", &EditorJoltCharacterControllerComponent::m_radius)
                ->Field("RigidBodyCharacter", &EditorJoltCharacterControllerComponent::m_rigidBodyCharacter)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                // The Physics::CharacterConfiguration field-level edit context comes from
                // AzFramework; the capsule Height/Radius are exposed here as plain floats.
                editContext->Class<EditorJoltCharacterControllerComponent>(
                    "Jolt Character Controller", "Character controller simulated by the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltCharacterControllerComponent::m_height,
                        "Height", "Total capsule height, including the hemispherical caps.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.01f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltCharacterControllerComponent::m_radius,
                        "Radius", "Capsule radius.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.01f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltCharacterControllerComponent::m_rigidBodyCharacter,
                        "Rigid body character",
                        "When set, the character is a real rigid body in the simulation (cheaper, most "
                        "accurate response with dynamic bodies). When clear, it is a virtual character.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltCharacterControllerComponent::m_characterConfig,
                        "Character Configuration", "Configuration of the character controller")
                    ;
            }
        }
    }

    void EditorJoltCharacterControllerComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("PhysicsWorldBodyService"));
        // A character controller acts as a dynamic kinematic rigid body.
        provided.push_back(AZ_CRC_CE("PhysicsRigidBodyService"));
        provided.push_back(AZ_CRC_CE("PhysicsCharacterControllerService"));
    }

    void EditorJoltCharacterControllerComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("PhysicsRigidBodyService"));
        incompatible.push_back(AZ_CRC_CE("PhysicsCharacterControllerService"));
        incompatible.push_back(AZ_CRC_CE("NonUniformScaleService"));
    }

    void EditorJoltCharacterControllerComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void EditorJoltCharacterControllerComponent::Activate()
    {
        AzToolsFramework::Components::EditorComponentBase::Activate();
        AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(GetEntityId());
    }

    void EditorJoltCharacterControllerComponent::Deactivate()
    {
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();
        AzToolsFramework::Components::EditorComponentBase::Deactivate();
    }

    void EditorJoltCharacterControllerComponent::DisplayEntityViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo,
        AzFramework::DebugDisplayRequests& debugDisplay)
    {
        // An explicitly assigned shape configuration wins over Height/Radius, exactly
        // as in JoltCharacterControllerComponent::CreateCharacter, so preview that
        // instead when one is set and it is a capsule.
        float height = m_height;
        float radius = m_radius;
        if (m_shapeConfig)
        {
            const auto* capsule = azdynamic_cast<const Physics::CapsuleShapeConfiguration*>(m_shapeConfig.get());
            if (!capsule)
            {
                // Some other shape drives the character; there is nothing meaningful
                // to draw here rather than a capsule that would not match.
                return;
            }
            height = capsule->m_height;
            radius = capsule->m_radius;
        }

        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        // The character shape is centred on the entity origin and Z-aligned in local
        // space; scale is deliberately dropped so the preview matches the capsule the
        // runtime builds from Height/Radius.
        worldTransform.ExtractUniformScale();

        EditorDebugDraw::DrawWireCapsule(debugDisplay, worldTransform, radius, height);
    }

    void EditorJoltCharacterControllerComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltCharacterControllerComponent>())
        {
            component->GetCharacterConfiguration() = m_characterConfig;
            component->GetShapeConfiguration() = m_shapeConfig;
            component->GetHeight() = m_height;
            component->GetRadius() = m_radius;
            component->GetRigidBodyCharacter() = m_rigidBodyCharacter;
        }
    }

} // namespace JoltPhysics
