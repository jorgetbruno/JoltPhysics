#include <Utils/JoltComponentUtils.h>
#include <Clients/Components/JoltCharacterControllerComponent.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Physics/SystemBus.h>

#include <Character/JoltCharacter.h>

namespace JoltPhysics
{
    void JoltCharacterControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<JoltCharacterGameplayRequestBus>("JoltCharacterGameplayRequestBus", "Jolt Character Gameplay")
                ->Attribute(AZ::Script::Attributes::Storage, AZ::Script::Attributes::StorageType::RuntimeOwn)
                ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::RemoveableByUser, true)
                ->Event("IsOnGround", &JoltCharacterGameplayRequests::IsOnGround, "Is On Ground")
                ->Event("GetGroundNormal", &JoltCharacterGameplayRequests::GetGroundNormal, "Get Ground Normal")
                ;
        }

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->RegisterGenericType<AZStd::shared_ptr<Physics::ShapeConfiguration>>();

            serializeContext->Class<JoltCharacterControllerComponent, AZ::Component>()
                ->Version(1)
                ->Field("CharacterConfiguration", &JoltCharacterControllerComponent::m_characterConfig)
                ->Field("ShapeConfiguration", &JoltCharacterControllerComponent::m_shapeConfig)
                ->Field("Height", &JoltCharacterControllerComponent::m_height)
                ->Field("Radius", &JoltCharacterControllerComponent::m_radius)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltCharacterControllerComponent>(
                    "Jolt Character Controller",
                    "Character controller simulated by the Jolt physics backend")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        // No AppearsInAddComponentMenu: EditorJoltCharacterControllerComponent owns
                        // the menu entry (PhysX-style editor/runtime split). The runtime component
                        // stays registered for old prefabs and BuildGameEntity.
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::RemoveableByUser, true)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltCharacterControllerComponent::m_height,
                        "Height", "Total capsule height, including the hemispherical caps.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.01f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltCharacterControllerComponent::m_radius,
                        "Radius", "Capsule radius.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.01f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltCharacterControllerComponent::m_characterConfig,
                        "Character Configuration", "Configuration of the character controller")
                    ;
            }
        }
    }

    void JoltCharacterControllerComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("PhysicsWorldBodyService"));
        // A character controller acts as a dynamic kinematic rigid body.
        provided.push_back(AZ_CRC_CE("PhysicsRigidBodyService"));
        provided.push_back(AZ_CRC_CE("PhysicsCharacterControllerService"));
    }

    void JoltCharacterControllerComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("PhysicsRigidBodyService"));
        incompatible.push_back(AZ_CRC_CE("PhysicsCharacterControllerService"));
        incompatible.push_back(AZ_CRC_CE("NonUniformScaleService"));
    }

    void JoltCharacterControllerComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void JoltCharacterControllerComponent::Activate()
    {
        Physics::DefaultWorldBus::BroadcastResult(m_attachedSceneHandle, &Physics::DefaultWorldRequests::GetDefaultSceneHandle);

        Physics::CharacterRequestBus::Handler::BusConnect(GetEntityId());
        JoltCharacterGameplayRequestBus::Handler::BusConnect(GetEntityId());
        AzPhysics::SimulatedBodyComponentRequestsBus::Handler::BusConnect(GetEntityId());
        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();

        if (m_attachedSceneHandle != AzPhysics::InvalidSceneHandle)
        {
            // Create on OnEntityActivated so the entity's transform is final.
            AZ::EntityBus::Handler::BusConnect(GetEntityId());
        }
    }

    void JoltCharacterControllerComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        AZ::EntityBus::Handler::BusDisconnect();
        AZ::TransformNotificationBus::Handler::BusDisconnect();
        AzPhysics::SimulatedBodyComponentRequestsBus::Handler::BusDisconnect();
        JoltCharacterGameplayRequestBus::Handler::BusDisconnect();
        Physics::CharacterRequestBus::Handler::BusDisconnect();

        DestroyCharacter();

        m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
    }

    void JoltCharacterControllerComponent::OnEntityActivated([[maybe_unused]] const AZ::EntityId& entityId)
    {
        AZ::EntityBus::Handler::BusDisconnect();

        CreateCharacter();
    }

    void JoltCharacterControllerComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_bodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
        {
            TryCreateCharacter();
            return;
        }

        if (AzPhysics::SimulatedBody* body = GetSimulatedBody())
        {
            m_syncingTransformFromCharacter = true;
            AZ::TransformBus::Event(GetEntityId(), &AZ::TransformBus::Events::SetWorldTM, body->GetTransform());
            m_syncingTransformFromCharacter = false;
        }
    }

    int JoltCharacterControllerComponent::GetTickOrder()
    {
        return AZ::TICK_PHYSICS;
    }

    void JoltCharacterControllerComponent::OnTransformChanged(
        [[maybe_unused]] const AZ::Transform& local, const AZ::Transform& world)
    {
        if (m_syncingTransformFromCharacter)
        {
            // The transform change came from the character itself, don't feed it back.
            return;
        }

        if (AzPhysics::SimulatedBody* body = GetSimulatedBody())
        {
            body->SetTransform(world);
        }
    }

    void JoltCharacterControllerComponent::TryCreateCharacter()
    {
        if (m_attachedSceneHandle == AzPhysics::InvalidSceneHandle)
        {
            Physics::DefaultWorldBus::BroadcastResult(m_attachedSceneHandle, &Physics::DefaultWorldRequests::GetDefaultSceneHandle);
        }

        if (m_attachedSceneHandle == AzPhysics::InvalidSceneHandle)
        {
            return;
        }

        CreateCharacter();
    }

    void JoltCharacterControllerComponent::CreateCharacter()
    {
        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        m_characterConfig.m_position = worldTransform.GetTranslation();
        m_characterConfig.m_orientation = worldTransform.GetRotation();
        m_characterConfig.m_entityId = GetEntityId();
        m_characterConfig.m_debugName = GetEntity()->GetName();
        // The inspector-editable Height/Radius drive the capsule; an explicitly-set
        // m_shapeConfig (e.g. assigned programmatically) takes precedence.
        m_characterConfig.m_shapeConfig = m_shapeConfig
            ? m_shapeConfig
            : AZStd::make_shared<Physics::CapsuleShapeConfiguration>(m_height, m_radius);

        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            if (AzPhysics::Scene* scene = physicsSystem->GetScene(m_attachedSceneHandle))
            {
                m_bodyHandle = scene->AddSimulatedBody(&m_characterConfig);
            }
        }
    }

    void JoltCharacterControllerComponent::DestroyCharacter()
    {
        if (m_bodyHandle != AzPhysics::InvalidSimulatedBodyHandle)
        {
            if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
            {
                if (AzPhysics::Scene* scene = physicsSystem->GetScene(m_attachedSceneHandle))
                {
                    scene->RemoveSimulatedBody(m_bodyHandle);
                }
            }
            m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        }

        m_syncingTransformFromCharacter = false;
    }

    // Physics::CharacterRequestBus

    AZ::Vector3 JoltCharacterControllerComponent::GetBasePosition() const
    {
        if (auto* character = const_cast<JoltCharacterControllerComponent*>(this)->GetCharacter())
        {
            return character->GetBasePosition();
        }
        return AZ::Vector3::CreateZero();
    }

    void JoltCharacterControllerComponent::SetBasePosition(const AZ::Vector3& position)
    {
        if (auto* character = GetCharacter())
        {
            character->SetBasePosition(position);
        }
    }

    AZ::Vector3 JoltCharacterControllerComponent::GetCenterPosition() const
    {
        if (auto* character = const_cast<JoltCharacterControllerComponent*>(this)->GetCharacter())
        {
            return character->GetCenterPosition();
        }
        return AZ::Vector3::CreateZero();
    }

    float JoltCharacterControllerComponent::GetStepHeight() const
    {
        if (auto* character = const_cast<JoltCharacterControllerComponent*>(this)->GetCharacter())
        {
            return character->GetStepHeight();
        }
        return 0.0f;
    }

    void JoltCharacterControllerComponent::SetStepHeight(float stepHeight)
    {
        if (auto* character = GetCharacter())
        {
            character->SetStepHeight(stepHeight);
        }
    }

    AZ::Vector3 JoltCharacterControllerComponent::GetUpDirection() const
    {
        return m_characterConfig.m_upDirection;
    }

    void JoltCharacterControllerComponent::SetUpDirection(const AZ::Vector3& upDirection)
    {
        m_characterConfig.m_upDirection = upDirection;
        if (auto* character = GetCharacter())
        {
            character->SetUpDirection(upDirection);
        }
    }

    float JoltCharacterControllerComponent::GetSlopeLimitDegrees() const
    {
        if (auto* character = const_cast<JoltCharacterControllerComponent*>(this)->GetCharacter())
        {
            return character->GetSlopeLimitDegrees();
        }
        return 0.0f;
    }

    void JoltCharacterControllerComponent::SetSlopeLimitDegrees(float slopeLimitDegrees)
    {
        if (auto* character = GetCharacter())
        {
            character->SetSlopeLimitDegrees(slopeLimitDegrees);
        }
    }

    float JoltCharacterControllerComponent::GetMaximumSpeed() const
    {
        if (auto* character = const_cast<JoltCharacterControllerComponent*>(this)->GetCharacter())
        {
            return character->GetMaximumSpeed();
        }
        return 0.0f;
    }

    void JoltCharacterControllerComponent::SetMaximumSpeed(float maximumSpeed)
    {
        if (auto* character = GetCharacter())
        {
            character->SetMaximumSpeed(maximumSpeed);
        }
    }

    AZ::Vector3 JoltCharacterControllerComponent::GetVelocity() const
    {
        if (auto* character = const_cast<JoltCharacterControllerComponent*>(this)->GetCharacter())
        {
            return character->GetVelocity();
        }
        return AZ::Vector3::CreateZero();
    }

    void JoltCharacterControllerComponent::AddVelocityForTick(const AZ::Vector3& velocity)
    {
        if (auto* character = GetCharacter())
        {
            character->AddVelocityForTick(velocity);
        }
    }

    void JoltCharacterControllerComponent::AddVelocityForPhysicsTimestep(const AZ::Vector3& velocity)
    {
        if (auto* character = GetCharacter())
        {
            character->AddVelocityForPhysicsTimestep(velocity);
        }
    }

    bool JoltCharacterControllerComponent::IsPresent() const
    {
        return IsPhysicsEnabled();
    }

    Physics::Character* JoltCharacterControllerComponent::GetCharacter()
    {
        return azdynamic_cast<Physics::Character*>(GetSimulatedBody());
    }

    // JoltCharacterGameplayRequestBus

    bool JoltCharacterControllerComponent::IsOnGround() const
    {
        if (auto* character = azdynamic_cast<JoltCharacter*>(
                const_cast<JoltCharacterControllerComponent*>(this)->GetSimulatedBody()))
        {
            return character->IsOnGround();
        }
        return false;
    }

    AZ::Vector3 JoltCharacterControllerComponent::GetGroundNormal() const
    {
        if (auto* character = azdynamic_cast<JoltCharacter*>(
                const_cast<JoltCharacterControllerComponent*>(this)->GetSimulatedBody()))
        {
            return character->GetGroundNormal();
        }
        return AZ::Vector3::CreateAxisZ();
    }

    // AzPhysics::SimulatedBodyComponentRequestsBus

    void JoltCharacterControllerComponent::EnablePhysics()
    {
        if (IsPhysicsEnabled())
        {
            return;
        }
        CreateCharacter();
    }

    void JoltCharacterControllerComponent::DisablePhysics()
    {
        DestroyCharacter();
    }

    bool JoltCharacterControllerComponent::IsPhysicsEnabled() const
    {
        return m_bodyHandle != AzPhysics::InvalidSimulatedBodyHandle;
    }

    AZ::Aabb JoltCharacterControllerComponent::GetAabb() const
    {
        if (auto* body = const_cast<JoltCharacterControllerComponent*>(this)->GetSimulatedBody())
        {
            return body->GetAabb();
        }
        return AZ::Aabb::CreateNull();
    }

    AzPhysics::SimulatedBody* JoltCharacterControllerComponent::GetSimulatedBody()
    {
        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            if (AzPhysics::Scene* scene = physicsSystem->GetScene(m_attachedSceneHandle))
            {
                return scene->GetSimulatedBodyFromHandle(m_bodyHandle);
            }
        }
        return nullptr;
    }

    AzPhysics::SimulatedBodyHandle JoltCharacterControllerComponent::GetSimulatedBodyHandle() const
    {
        return m_bodyHandle;
    }

    AzPhysics::SceneQueryHit JoltCharacterControllerComponent::RayCast(const AzPhysics::RayCastRequest& request)
    {
        if (AzPhysics::SimulatedBody* body = GetSimulatedBody())
        {
            return body->RayCast(request);
        }
        return AzPhysics::SceneQueryHit();
    }


    void JoltCharacterControllerComponent::OnAfterEntitySet()
    {
        if (m_serializedIdentifier.empty())
        {
            m_serializedIdentifier = Internal::GenerateSerializedIdentifier(this);
        }
    }

    AZStd::string JoltCharacterControllerComponent::GetSerializedIdentifier() const
    {
        return m_serializedIdentifier;
    }
} // namespace JoltPhysics
