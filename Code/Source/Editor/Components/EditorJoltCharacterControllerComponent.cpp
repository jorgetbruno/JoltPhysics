#include <Editor/Components/EditorJoltCharacterControllerComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzToolsFramework/ComponentModes/CapsuleComponentMode.h>

#include <Clients/Components/JoltCharacterControllerComponent.h>
#include <Editor/Components/EditorJoltDebugDrawUtils.h>

namespace JoltPhysics
{
    void EditorJoltCharacterControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        AzToolsFramework::ComponentModeFramework::ComponentModeDelegate::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->RegisterGenericType<AZStd::shared_ptr<Physics::ShapeConfiguration>>();

            serializeContext->Class<EditorJoltCharacterControllerComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(2)
                ->Field("CharacterConfiguration", &EditorJoltCharacterControllerComponent::m_characterConfig)
                ->Field("ShapeConfiguration", &EditorJoltCharacterControllerComponent::m_shapeConfig)
                ->Field("Height", &EditorJoltCharacterControllerComponent::m_height)
                ->Field("Radius", &EditorJoltCharacterControllerComponent::m_radius)
                ->Field("RigidBodyCharacter", &EditorJoltCharacterControllerComponent::m_rigidBodyCharacter)
                ->Field("ComponentMode", &EditorJoltCharacterControllerComponent::m_componentModeDelegate)
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
                    // Renders the Edit button that enters component mode.
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltCharacterControllerComponent::m_componentModeDelegate,
                        "Component Mode", "Character capsule component mode")
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

        const AZ::EntityComponentIdPair entityComponentIdPair(GetEntityId(), GetId());
        AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(GetEntityId());
        AzToolsFramework::CapsuleManipulatorRequestBus::Handler::BusConnect(entityComponentIdPair);
        AzToolsFramework::RadiusManipulatorRequestBus::Handler::BusConnect(entityComponentIdPair);
        AzToolsFramework::ShapeManipulatorRequestBus::Handler::BusConnect(entityComponentIdPair);
        // Addressed by entity, not by entity+component, unlike the manipulator buses.
        AzToolsFramework::EditorComponentSelectionRequestsBus::Handler::BusConnect(GetEntityId());

        m_componentModeDelegate.ConnectWithSingleComponentMode<
            EditorJoltCharacterControllerComponent, AzToolsFramework::CapsuleComponentMode>(entityComponentIdPair, this);
    }

    void EditorJoltCharacterControllerComponent::Deactivate()
    {
        m_componentModeDelegate.Disconnect();

        AzToolsFramework::EditorComponentSelectionRequestsBus::Handler::BusDisconnect();
        AzToolsFramework::ShapeManipulatorRequestBus::Handler::BusDisconnect();
        AzToolsFramework::RadiusManipulatorRequestBus::Handler::BusDisconnect();
        AzToolsFramework::CapsuleManipulatorRequestBus::Handler::BusDisconnect();
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();

        AzToolsFramework::Components::EditorComponentBase::Deactivate();
    }

    bool EditorJoltCharacterControllerComponent::TryGetEffectiveCapsule(float& height, float& radius) const
    {
        // An explicitly assigned shape configuration wins over Height/Radius, exactly
        // as in JoltCharacterControllerComponent::CreateCharacter.
        if (m_shapeConfig)
        {
            const auto* capsule = azdynamic_cast<const Physics::CapsuleShapeConfiguration*>(m_shapeConfig.get());
            if (!capsule)
            {
                return false;
            }
            height = capsule->m_height;
            radius = capsule->m_radius;
            return true;
        }

        height = m_height;
        radius = m_radius;
        return true;
    }

    void EditorJoltCharacterControllerComponent::OnShapeChangedByManipulator()
    {
        AzToolsFramework::ToolsApplicationEvents::Bus::Broadcast(
            &AzToolsFramework::ToolsApplicationEvents::InvalidatePropertyDisplay,
            AzToolsFramework::Refresh_Values);
    }

    void EditorJoltCharacterControllerComponent::DisplayEntityViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo,
        AzFramework::DebugDisplayRequests& debugDisplay)
    {
        float height = 0.0f;
        float radius = 0.0f;
        if (!TryGetEffectiveCapsule(height, radius))
        {
            // Some other shape drives the character; better nothing than a capsule
            // that would not match.
            return;
        }

        // The entity sits at the character's base, so the capsule stands on the origin
        // rather than being centred on it - see JoltCharacter::BaseToCenter.
        const AZ::Transform capsuleTransform =
            GetManipulatorSpace() * AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 0.5f * height));

        EditorDebugDraw::DrawWireCapsule(debugDisplay, capsuleTransform, radius, height);
    }

    float EditorJoltCharacterControllerComponent::GetHeight() const
    {
        return m_height;
    }

    void EditorJoltCharacterControllerComponent::SetHeight(float height)
    {
        // Height is the total including both caps, so it can never be shorter than the
        // sphere those caps would form - same rule as the capsule collider.
        m_height = AZ::GetMax(height, 2.0f * m_radius);
        OnShapeChangedByManipulator();
    }

    float EditorJoltCharacterControllerComponent::GetRadius() const
    {
        return m_radius;
    }

    void EditorJoltCharacterControllerComponent::SetRadius(float radius)
    {
        m_radius = AZ::GetClamp(radius, 0.0001f, 0.5f * m_height);
        OnShapeChangedByManipulator();
    }

    AZ::Vector3 EditorJoltCharacterControllerComponent::GetTranslationOffset() const
    {
        return AZ::Vector3::CreateZero();
    }

    void EditorJoltCharacterControllerComponent::SetTranslationOffset([[maybe_unused]] const AZ::Vector3& translationOffset)
    {
        // The character capsule is always centred on the entity; there is nowhere to
        // store an offset and the runtime would ignore one.
    }

    AZ::Transform EditorJoltCharacterControllerComponent::GetManipulatorSpace() const
    {
        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        worldTransform.ExtractUniformScale();
        return worldTransform;
    }

    AZ::Quaternion EditorJoltCharacterControllerComponent::GetRotationOffset() const
    {
        return AZ::Quaternion::CreateIdentity();
    }

    AZ::Aabb EditorJoltCharacterControllerComponent::GetEditorSelectionBoundsViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo)
    {
        float height = 0.0f;
        float radius = 0.0f;
        if (!TryGetEffectiveCapsule(height, radius))
        {
            return AZ::Aabb::CreateNull();
        }

        // Centred half a height up, matching where the capsule is drawn.
        const AZ::Aabb localBounds = AZ::Aabb::CreateCenterHalfExtents(
            AZ::Vector3(0.0f, 0.0f, 0.5f * height), AZ::Vector3(radius, radius, 0.5f * height));
        return localBounds.GetTransformedAabb(GetManipulatorSpace());
    }

    bool EditorJoltCharacterControllerComponent::SupportsEditorRayIntersect()
    {
        return false;
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
