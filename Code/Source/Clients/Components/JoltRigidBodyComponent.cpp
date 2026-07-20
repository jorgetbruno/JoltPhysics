#include <Clients/Components/JoltRigidBodyComponent.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <AzFramework/Physics/SystemBus.h>

#include <Clients/Components/JoltColliderComponentBase.h>
#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void JoltRigidBodyComponent::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<AzPhysics::SimulatedBodyConfiguration>(context);
        Internal::ReflectOnce<AzPhysics::RigidBodyConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltRigidBodyComponent, AZ::Component>()
                ->Version(1)
                ->Field("RigidBodyConfiguration", &JoltRigidBodyComponent::m_configuration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltRigidBodyComponent>("Jolt Rigid Body",
                    "Dynamic rigid body simulated by the Jolt physics backend")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltRigidBodyComponent::m_configuration,
                        "Configuration", "Rigid body configuration")
                    ;

                editContext->Class<AzPhysics::RigidBodyConfiguration>("Jolt Rigid Body Configuration", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AzPhysics::RigidBodyConfiguration::m_kinematic,
                        "Kinematic", "Determines how the movement/position of the rigid body is controlled.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AzPhysics::RigidBodyConfiguration::m_mass,
                        "Mass", "Mass of the rigid body in kilograms.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AzPhysics::RigidBodyConfiguration::m_initialLinearVelocity,
                        "Initial linear velocity", "Initial linear velocity of the rigid body when it is activated.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AzPhysics::RigidBodyConfiguration::m_initialAngularVelocity,
                        "Initial angular velocity", "Initial angular velocity of the rigid body when it is activated.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AzPhysics::RigidBodyConfiguration::m_linearDamping,
                        "Linear damping", "Damping applied to the linear velocity of the rigid body.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AzPhysics::RigidBodyConfiguration::m_angularDamping,
                        "Angular damping", "Damping applied to the angular velocity of the rigid body.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AzPhysics::RigidBodyConfiguration::m_sleepMinEnergy,
                        "Sleep threshold", "Kinetic energy below which the rigid body is allowed to go to sleep.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AzPhysics::RigidBodyConfiguration::m_startAsleep,
                        "Start asleep", "Whether the rigid body starts asleep when it is activated.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AzPhysics::RigidBodyConfiguration::m_gravityEnabled,
                        "Gravity enabled", "Whether the rigid body is affected by gravity.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AzPhysics::RigidBodyConfiguration::m_ccdEnabled,
                        "CCD enabled", "Whether continuous collision detection is enabled for the rigid body.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AzPhysics::RigidBodyConfiguration::m_maxAngularVelocity,
                        "Maximum angular velocity", "Upper limit on the angular velocity of the rigid body.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ;
            }
        }
    }

    void JoltRigidBodyComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltRigidBodyService"));
    }

    void JoltRigidBodyComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltRigidBodyService"));
        incompatible.push_back(AZ_CRC_CE("JoltStaticRigidBodyService"));
    }

    void JoltRigidBodyComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        required.push_back(AZ_CRC_CE("JoltColliderService"));
    }

    void JoltRigidBodyComponent::Activate()
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

    void JoltRigidBodyComponent::OnEntityActivated([[maybe_unused]] const AZ::EntityId& entityId)
    {
        AZ::EntityBus::Handler::BusDisconnect();

        CreateRigidBody();
    }

    void JoltRigidBodyComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        AZ::EntityBus::Handler::BusDisconnect();

        DestroyRigidBody();

        m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
    }

    void JoltRigidBodyComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_bodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
        {
            TryCreateRigidBody();
            return;
        }

        if (AzPhysics::RigidBody* body = GetRigidBody();
            body != nullptr && !body->IsKinematic())
        {
            m_syncingTransformFromBody = true;
            AZ::TransformBus::Event(GetEntityId(), &AZ::TransformBus::Events::SetWorldTM, body->GetTransform());
            m_syncingTransformFromBody = false;
        }
    }

    int JoltRigidBodyComponent::GetTickOrder()
    {
        return AZ::TICK_PHYSICS;
    }

    void JoltRigidBodyComponent::OnTransformChanged([[maybe_unused]] const AZ::Transform& local, const AZ::Transform& world)
    {
        if (m_syncingTransformFromBody)
        {
            // The transform change came from the physics simulation itself, don't feed it back.
            return;
        }

        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            if (body->IsKinematic())
            {
                body->SetKinematicTarget(world);
            }
            else
            {
                body->SetTransform(world);
            }
        }
    }

    void JoltRigidBodyComponent::TryCreateRigidBody()
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

    void JoltRigidBodyComponent::CreateRigidBody()
    {
        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        m_configuration.m_position = worldTransform.GetTranslation();
        m_configuration.m_orientation = worldTransform.GetRotation();
        m_configuration.m_entityId = GetEntityId();
        m_configuration.m_debugName = GetEntity()->GetName();

        AzPhysics::ShapeColliderPairList shapeColliderPairs;
        for (JoltColliderComponentBase* collider : GetEntity()->FindComponents<JoltColliderComponentBase>())
        {
            shapeColliderPairs.push_back(collider->GetShapeColliderPair());
        }
        if (shapeColliderPairs.size() == 1)
        {
            m_configuration.m_colliderAndShapeData = shapeColliderPairs.front();
        }
        else if (!shapeColliderPairs.empty())
        {
            m_configuration.m_colliderAndShapeData = shapeColliderPairs;
        }

        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            if (AzPhysics::Scene* scene = physicsSystem->GetScene(m_attachedSceneHandle))
            {
                m_bodyHandle = scene->AddSimulatedBody(&m_configuration);
            }
        }

        if (m_bodyHandle != AzPhysics::InvalidSimulatedBodyHandle)
        {
            Physics::RigidBodyRequestBus::Handler::BusConnect(GetEntityId());
            AzPhysics::SimulatedBodyComponentRequestsBus::Handler::BusConnect(GetEntityId());
            AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
            AZ::TickBus::Handler::BusConnect();
        }
    }

    void JoltRigidBodyComponent::DestroyRigidBody()
    {
        AZ::TransformNotificationBus::Handler::BusDisconnect();
        AzPhysics::SimulatedBodyComponentRequestsBus::Handler::BusDisconnect();
        Physics::RigidBodyRequestBus::Handler::BusDisconnect();

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

        m_syncingTransformFromBody = false;
    }

    void JoltRigidBodyComponent::EnablePhysics()
    {
        if (IsPhysicsEnabled())
        {
            return;
        }

        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            if (AzPhysics::Scene* scene = physicsSystem->GetScene(m_attachedSceneHandle))
            {
                scene->EnableSimulationOfBody(m_bodyHandle);
            }
        }

        Physics::RigidBodyNotificationBus::Event(
            GetEntityId(), &Physics::RigidBodyNotificationBus::Events::OnPhysicsEnabled, GetEntityId());
    }

    void JoltRigidBodyComponent::DisablePhysics()
    {
        if (!IsPhysicsEnabled())
        {
            return;
        }

        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            if (AzPhysics::Scene* scene = physicsSystem->GetScene(m_attachedSceneHandle))
            {
                scene->DisableSimulationOfBody(m_bodyHandle);
            }
        }

        Physics::RigidBodyNotificationBus::Event(
            GetEntityId(), &Physics::RigidBodyNotificationBus::Events::OnPhysicsDisabled, GetEntityId());
    }

    bool JoltRigidBodyComponent::IsPhysicsEnabled() const
    {
        return GetRigidBodyConst() != nullptr;
    }

    AzPhysics::SimulatedBodyHandle JoltRigidBodyComponent::GetSimulatedBodyHandle() const
    {
        return m_bodyHandle;
    }

    AzPhysics::SimulatedBody* JoltRigidBodyComponent::GetSimulatedBody()
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

    AzPhysics::RigidBody* JoltRigidBodyComponent::GetRigidBody()
    {
        return static_cast<AzPhysics::RigidBody*>(GetSimulatedBody());
    }

    const AzPhysics::RigidBody* JoltRigidBodyComponent::GetRigidBodyConst() const
    {
        return const_cast<JoltRigidBodyComponent*>(this)->GetRigidBody();
    }

    AzPhysics::SceneQueryHit JoltRigidBodyComponent::RayCast(const AzPhysics::RayCastRequest& request)
    {
        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            return body->RayCast(request);
        }
        return AzPhysics::SceneQueryHit();
    }

    AZ::Aabb JoltRigidBodyComponent::GetAabb() const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->GetAabb();
        }
        return AZ::Aabb::CreateNull();
    }

    AZ::Vector3 JoltRigidBodyComponent::GetCenterOfMassWorld() const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->GetCenterOfMassWorld();
        }
        return AZ::Vector3::CreateZero();
    }

    AZ::Vector3 JoltRigidBodyComponent::GetCenterOfMassLocal() const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->GetCenterOfMassLocal();
        }
        return AZ::Vector3::CreateZero();
    }

    AZ::Matrix3x3 JoltRigidBodyComponent::GetInertiaWorld() const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->GetInertiaWorld();
        }
        return AZ::Matrix3x3::CreateZero();
    }

    AZ::Matrix3x3 JoltRigidBodyComponent::GetInertiaLocal() const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->GetInertiaLocal();
        }
        return AZ::Matrix3x3::CreateZero();
    }

    AZ::Matrix3x3 JoltRigidBodyComponent::GetInverseInertiaWorld() const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->GetInverseInertiaWorld();
        }
        return AZ::Matrix3x3::CreateZero();
    }

    AZ::Matrix3x3 JoltRigidBodyComponent::GetInverseInertiaLocal() const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->GetInverseInertiaLocal();
        }
        return AZ::Matrix3x3::CreateZero();
    }

    float JoltRigidBodyComponent::GetMass() const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->GetMass();
        }
        return 0.0f;
    }

    float JoltRigidBodyComponent::GetInverseMass() const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->GetInverseMass();
        }
        return 0.0f;
    }

    void JoltRigidBodyComponent::SetMass(float mass)
    {
        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            body->SetMass(mass);
        }
    }

    void JoltRigidBodyComponent::SetCenterOfMassOffset(const AZ::Vector3& comOffset)
    {
        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            body->SetCenterOfMassOffset(comOffset);
        }
    }

    AZ::Vector3 JoltRigidBodyComponent::GetLinearVelocity() const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->GetLinearVelocity();
        }
        return AZ::Vector3::CreateZero();
    }

    void JoltRigidBodyComponent::SetLinearVelocity(const AZ::Vector3& velocity)
    {
        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            body->SetLinearVelocity(velocity);
        }
    }

    AZ::Vector3 JoltRigidBodyComponent::GetAngularVelocity() const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->GetAngularVelocity();
        }
        return AZ::Vector3::CreateZero();
    }

    void JoltRigidBodyComponent::SetAngularVelocity(const AZ::Vector3& angularVelocity)
    {
        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            body->SetAngularVelocity(angularVelocity);
        }
    }

    AZ::Vector3 JoltRigidBodyComponent::GetLinearVelocityAtWorldPoint(const AZ::Vector3& worldPoint) const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->GetLinearVelocityAtWorldPoint(worldPoint);
        }
        return AZ::Vector3::CreateZero();
    }

    void JoltRigidBodyComponent::ApplyLinearImpulse(const AZ::Vector3& impulse)
    {
        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            body->ApplyLinearImpulse(impulse);
        }
    }

    void JoltRigidBodyComponent::ApplyLinearImpulseAtWorldPoint(const AZ::Vector3& impulse, const AZ::Vector3& worldPoint)
    {
        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            body->ApplyLinearImpulseAtWorldPoint(impulse, worldPoint);
        }
    }

    void JoltRigidBodyComponent::ApplyAngularImpulse(const AZ::Vector3& angularImpulse)
    {
        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            body->ApplyAngularImpulse(angularImpulse);
        }
    }

    float JoltRigidBodyComponent::GetLinearDamping() const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->GetLinearDamping();
        }
        return 0.0f;
    }

    void JoltRigidBodyComponent::SetLinearDamping(float damping)
    {
        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            body->SetLinearDamping(damping);
        }
    }

    float JoltRigidBodyComponent::GetAngularDamping() const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->GetAngularDamping();
        }
        return 0.0f;
    }

    void JoltRigidBodyComponent::SetAngularDamping(float damping)
    {
        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            body->SetAngularDamping(damping);
        }
    }

    bool JoltRigidBodyComponent::IsAwake() const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->IsAwake();
        }
        return false;
    }

    void JoltRigidBodyComponent::ForceAsleep()
    {
        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            body->ForceAsleep();
        }
    }

    void JoltRigidBodyComponent::ForceAwake()
    {
        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            body->ForceAwake();
        }
    }

    bool JoltRigidBodyComponent::IsKinematic() const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->IsKinematic();
        }
        return false;
    }

    void JoltRigidBodyComponent::SetKinematic(bool kinematic)
    {
        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            body->SetKinematic(kinematic);
        }
    }

    void JoltRigidBodyComponent::SetKinematicTarget(const AZ::Transform& targetPosition)
    {
        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            body->SetKinematicTarget(targetPosition);
        }
    }

    bool JoltRigidBodyComponent::IsGravityEnabled() const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->IsGravityEnabled();
        }
        return false;
    }

    void JoltRigidBodyComponent::SetGravityEnabled(bool enabled)
    {
        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            body->SetGravityEnabled(enabled);
        }
    }

    void JoltRigidBodyComponent::SetSimulationEnabled(bool enabled)
    {
        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            if (AzPhysics::Scene* scene = physicsSystem->GetScene(m_attachedSceneHandle))
            {
                if (enabled)
                {
                    scene->EnableSimulationOfBody(m_bodyHandle);
                }
                else
                {
                    scene->DisableSimulationOfBody(m_bodyHandle);
                }
            }
        }
    }

    float JoltRigidBodyComponent::GetSleepThreshold() const
    {
        if (const AzPhysics::RigidBody* body = GetRigidBodyConst())
        {
            return body->GetSleepThreshold();
        }
        return 0.0f;
    }

    void JoltRigidBodyComponent::SetSleepThreshold(float threshold)
    {
        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            body->SetSleepThreshold(threshold);
        }
    }
} // namespace JoltPhysics
