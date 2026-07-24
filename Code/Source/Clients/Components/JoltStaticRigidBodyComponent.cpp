#include <Utils/JoltComponentUtils.h>
#include <Clients/Components/JoltStaticRigidBodyComponent.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <AzFramework/Physics/SystemBus.h>
#include <AzFramework/Physics/ColliderComponentBus.h>

#include <Clients/Components/JoltColliderComponentBase.h>
#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void JoltStaticRigidBodyComponent::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<AzPhysics::SimulatedBodyConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltStaticRigidBodyComponent, AZ::Component>()
                ->Version(1)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltStaticRigidBodyComponent>("Jolt Static Rigid Body",
                    "Static (non-moving) rigid body simulated by the Jolt physics backend")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        // No AppearsInAddComponentMenu: EditorJoltStaticRigidBodyComponent owns the
                        // menu entry (PhysX-style editor/runtime split). The runtime component
                        // stays registered for old prefabs and BuildGameEntity.
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void JoltStaticRigidBodyComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltStaticRigidBodyService"));
    }

    void JoltStaticRigidBodyComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltStaticRigidBodyService"));
        incompatible.push_back(AZ_CRC_CE("JoltRigidBodyService"));
    }

    void JoltStaticRigidBodyComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        required.push_back(AZ_CRC_CE("JoltColliderService"));
    }

    void JoltStaticRigidBodyComponent::Activate()
    {
        Physics::DefaultWorldBus::BroadcastResult(m_attachedSceneHandle, &Physics::DefaultWorldRequests::GetDefaultSceneHandle);

        if (m_attachedSceneHandle != AzPhysics::InvalidSceneHandle)
        {
            // During activation all the collider components on the entity become ready.
            // Delaying the creation of the rigid body to OnEntityActivated so all the shapes are available.
            AZ::EntityBus::Handler::BusConnect(GetEntityId());
        }
        else
        {
            // No default scene yet (e.g. entity activated outside of game mode); retry on tick.
            AZ::TickBus::Handler::BusConnect();
        }
    }

    void JoltStaticRigidBodyComponent::OnEntityActivated([[maybe_unused]] const AZ::EntityId& entityId)
    {
        AZ::EntityBus::Handler::BusDisconnect();

        CreateRigidBody();
    }

    void JoltStaticRigidBodyComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        AZ::EntityBus::Handler::BusDisconnect();

        DestroyRigidBody();

        m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
    }

    void JoltStaticRigidBodyComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_rebuildPending)
        {
            m_rebuildPending = false;
            DestroyRigidBody();
            CreateRigidBody();
        }
        else
        {
            TryCreateRigidBody();
        }

        if (m_bodyHandle != AzPhysics::InvalidSimulatedBodyHandle)
        {
            // Body created, no need to tick anymore. Static bodies don't sync transforms from the simulation.
            AZ::TickBus::Handler::BusDisconnect();
        }
    }

    int JoltStaticRigidBodyComponent::GetTickOrder()
    {
        return AZ::TICK_PHYSICS;
    }

    void JoltStaticRigidBodyComponent::OnTransformChanged([[maybe_unused]] const AZ::Transform& local, const AZ::Transform& world)
    {
        if (AzPhysics::SimulatedBody* body = GetSimulatedBody())
        {
            body->SetTransform(world);
        }
    }

    void JoltStaticRigidBodyComponent::TryCreateRigidBody()
    {
        if (m_attachedSceneHandle == AzPhysics::InvalidSceneHandle)
        {
            Physics::DefaultWorldBus::BroadcastResult(m_attachedSceneHandle, &Physics::DefaultWorldRequests::GetDefaultSceneHandle);
        }

        if (m_attachedSceneHandle == AzPhysics::InvalidSceneHandle)
        {
            return;
        }

        CreateRigidBody();
    }

    void JoltStaticRigidBodyComponent::CreateRigidBody()
    {
        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        AzPhysics::StaticRigidBodyConfiguration configuration;
        configuration.m_position = worldTransform.GetTranslation();
        configuration.m_orientation = worldTransform.GetRotation();
        configuration.m_entityId = GetEntityId();
        configuration.m_debugName = GetEntity()->GetName();

        AzPhysics::ShapeColliderPairList shapeColliderPairs;
        for (AZ::Component* component : GetEntity()->GetComponents())
        {
            if (auto* collider = azrtti_cast<JoltColliderComponentBase*>(component))
            {
                for (const AzPhysics::ShapeColliderPair& pair : collider->GetShapeColliderPairs())
                {
                    shapeColliderPairs.push_back(pair);
                }
            }
        }
        if (shapeColliderPairs.size() == 1)
        {
            configuration.m_colliderAndShapeData = shapeColliderPairs.front();
        }
        else if (!shapeColliderPairs.empty())
        {
            configuration.m_colliderAndShapeData = shapeColliderPairs;
        }

        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            if (AzPhysics::Scene* scene = physicsSystem->GetScene(m_attachedSceneHandle))
            {
                m_bodyHandle = scene->AddSimulatedBody(&configuration);
            }
        }

        if (m_bodyHandle != AzPhysics::InvalidSimulatedBodyHandle)
        {
            AzPhysics::SimulatedBodyComponentRequestsBus::Handler::BusConnect(GetEntityId());
            AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
            Physics::ColliderComponentEventBus::Handler::BusConnect(GetEntityId());
        }
    }

    void JoltStaticRigidBodyComponent::DestroyRigidBody()
    {
        AZ::TransformNotificationBus::Handler::BusDisconnect();
        Physics::ColliderComponentEventBus::Handler::BusDisconnect();
        AzPhysics::SimulatedBodyComponentRequestsBus::Handler::BusDisconnect();

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
    }

    void JoltStaticRigidBodyComponent::OnColliderChanged()
    {
        // Never rebuild inside the bus dispatch; defer to the next tick.
        m_rebuildPending = true;
        AZ::TickBus::Handler::BusConnect();
    }

    void JoltStaticRigidBodyComponent::EnablePhysics()
    {
        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            if (AzPhysics::Scene* scene = physicsSystem->GetScene(m_attachedSceneHandle))
            {
                scene->EnableSimulationOfBody(m_bodyHandle);
            }
        }
    }

    void JoltStaticRigidBodyComponent::DisablePhysics()
    {
        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            if (AzPhysics::Scene* scene = physicsSystem->GetScene(m_attachedSceneHandle))
            {
                scene->DisableSimulationOfBody(m_bodyHandle);
            }
        }
    }

    bool JoltStaticRigidBodyComponent::IsPhysicsEnabled() const
    {
        return m_bodyHandle != AzPhysics::InvalidSimulatedBodyHandle;
    }

    AzPhysics::SimulatedBodyHandle JoltStaticRigidBodyComponent::GetSimulatedBodyHandle() const
    {
        return m_bodyHandle;
    }

    AzPhysics::SimulatedBody* JoltStaticRigidBodyComponent::GetSimulatedBody()
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

    AzPhysics::SceneQueryHit JoltStaticRigidBodyComponent::RayCast(const AzPhysics::RayCastRequest& request)
    {
        if (AzPhysics::SimulatedBody* body = GetSimulatedBody())
        {
            return body->RayCast(request);
        }
        return AzPhysics::SceneQueryHit();
    }

    AZ::Aabb JoltStaticRigidBodyComponent::GetAabb() const
    {
        if (AzPhysics::SimulatedBody* body = const_cast<JoltStaticRigidBodyComponent*>(this)->GetSimulatedBody())
        {
            return body->GetAabb();
        }
        return AZ::Aabb::CreateNull();
    }

    void JoltStaticRigidBodyComponent::OnAfterEntitySet()
    {
        if (m_serializedIdentifier.empty())
        {
            m_serializedIdentifier = Internal::GenerateSerializedIdentifier(this);
        }
    }

    AZStd::string JoltStaticRigidBodyComponent::GetSerializedIdentifier() const
    {
        return m_serializedIdentifier;
    }
} // namespace JoltPhysics
