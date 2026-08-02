#include <Utils/JoltComponentUtils.h>
#include <Clients/Components/JoltRigidBodyComponent.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <AzFramework/Physics/SystemBus.h>
#include <AzFramework/Physics/ColliderComponentBus.h>

#include <Clients/Components/JoltColliderComponentBase.h>
#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void JoltRigidBodyComponent::Reflect(AZ::ReflectContext* context)
    {
        // The single most-used physics script surface, and in a Jolt project nobody was
        // reflecting it: AzFramework declares Physics::RigidBodyRequestBus but leaves the
        // script binding to a backend, and the only implementation in 26.05 lives in the
        // PhysX gem - which a Jolt project has to disable. So Lua and Script Canvas could
        // not set a velocity, apply an impulse or read a mass on any body.
        //
        // Guarded like the gem's property handlers: if PhysX (or anything else) got there
        // first, leave its registration alone rather than fighting over the name.
        Internal::ReflectEBusOnce(context, "RigidBodyRequestBus",
            [](AZ::BehaviorContext* behaviorContext)
            {
                behaviorContext->EBus<Physics::RigidBodyRequestBus>("RigidBodyRequestBus")
                    ->Attribute(AZ::Script::Attributes::Storage, AZ::Script::Attributes::StorageType::RuntimeOwn)
                    ->Attribute(AZ::Script::Attributes::Category, "Physics")
                    ->Event("EnablePhysics", &Physics::RigidBodyRequests::EnablePhysics)
                    ->Event("DisablePhysics", &Physics::RigidBodyRequests::DisablePhysics)
                    ->Event("IsPhysicsEnabled", &Physics::RigidBodyRequests::IsPhysicsEnabled)

                    ->Event("GetCenterOfMassWorld", &Physics::RigidBodyRequests::GetCenterOfMassWorld)
                    ->Event("GetCenterOfMassLocal", &Physics::RigidBodyRequests::GetCenterOfMassLocal)
                    ->Event("SetCenterOfMassOffset", &Physics::RigidBodyRequests::SetCenterOfMassOffset)

                    ->Event("GetMass", &Physics::RigidBodyRequests::GetMass)
                    ->Event("GetInverseMass", &Physics::RigidBodyRequests::GetInverseMass)
                    ->Event("SetMass", &Physics::RigidBodyRequests::SetMass)

                    ->Event("GetLinearVelocity", &Physics::RigidBodyRequests::GetLinearVelocity)
                    ->Event("SetLinearVelocity", &Physics::RigidBodyRequests::SetLinearVelocity)
                    ->Event("GetAngularVelocity", &Physics::RigidBodyRequests::GetAngularVelocity)
                    ->Event("SetAngularVelocity", &Physics::RigidBodyRequests::SetAngularVelocity)
                    ->Event("GetLinearVelocityAtWorldPoint", &Physics::RigidBodyRequests::GetLinearVelocityAtWorldPoint)

                    ->Event("ApplyLinearImpulse", &Physics::RigidBodyRequests::ApplyLinearImpulse)
                    ->Event("ApplyLinearImpulseAtWorldPoint", &Physics::RigidBodyRequests::ApplyLinearImpulseAtWorldPoint)
                    ->Event("ApplyAngularImpulse", &Physics::RigidBodyRequests::ApplyAngularImpulse)

                    ->Event("GetLinearDamping", &Physics::RigidBodyRequests::GetLinearDamping)
                    ->Event("SetLinearDamping", &Physics::RigidBodyRequests::SetLinearDamping)
                    ->Event("GetAngularDamping", &Physics::RigidBodyRequests::GetAngularDamping)
                    ->Event("SetAngularDamping", &Physics::RigidBodyRequests::SetAngularDamping)

                    ->Event("IsAwake", &Physics::RigidBodyRequests::IsAwake)
                    ->Event("ForceAsleep", &Physics::RigidBodyRequests::ForceAsleep)
                    ->Event("ForceAwake", &Physics::RigidBodyRequests::ForceAwake)
                    ->Event("GetSleepThreshold", &Physics::RigidBodyRequests::GetSleepThreshold)
                    ->Event("SetSleepThreshold", &Physics::RigidBodyRequests::SetSleepThreshold)

                    ->Event("IsKinematic", &Physics::RigidBodyRequests::IsKinematic)
                    ->Event("SetKinematic", &Physics::RigidBodyRequests::SetKinematic)
                    ->Event("SetKinematicTarget", &Physics::RigidBodyRequests::SetKinematicTarget)

                    ->Event("IsGravityEnabled", &Physics::RigidBodyRequests::IsGravityEnabled)
                    ->Event("SetGravityEnabled", &Physics::RigidBodyRequests::SetGravityEnabled)
                    ->Event("SetSimulationEnabled", &Physics::RigidBodyRequests::SetSimulationEnabled)

                    ->Event("GetAabb", &Physics::RigidBodyRequests::GetAabb)
                    ;
            });

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
                        // No AppearsInAddComponentMenu: EditorJoltRigidBodyComponent owns the
                        // menu entry (PhysX-style editor/runtime split). The runtime component
                        // stays registered for old prefabs and BuildGameEntity.
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::RemoveableByUser, true)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltRigidBodyComponent::m_configuration,
                        "Configuration", "Rigid body configuration")
                    ;

                editContext->Class<AzPhysics::RigidBodyConfiguration>("Jolt Rigid Body Configuration", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::RemoveableByUser, true)
                        ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AzPhysics::RigidBodyConfiguration::m_kinematic,
                        "Kinematic", "Determines how the movement/position of the rigid body is controlled.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AzPhysics::RigidBodyConfiguration::m_computeMass,
                        "Compute mass", "Derive the mass from the collider volumes and their material densities. "
                        "Untick to author a mass directly.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AzPhysics::RigidBodyConfiguration::m_mass,
                        "Mass", "Mass of the rigid body in kilograms. Read-only while Compute mass is ticked, "
                        "because the geometry decides it.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        // Greyed rather than hidden: a field that vanishes looks like a
                        // missing feature, while a greyed one with the checkbox above it
                        // says who is in charge of the value.
                        ->Attribute(AZ::Edit::Attributes::ReadOnly, &AzPhysics::RigidBodyConfiguration::m_computeMass)
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

        Physics::RigidBodyRequestBus::Handler::BusConnect(GetEntityId());
        AzPhysics::SimulatedBodyComponentRequestsBus::Handler::BusConnect(GetEntityId());
        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
        Physics::ColliderComponentEventBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();

        if (m_attachedSceneHandle != AzPhysics::InvalidSceneHandle)
        {
            // During activation all the collider components on the entity become ready.
            // Delaying the creation of the rigid body to OnEntityActivated so all the shapes are available.
            AZ::EntityBus::Handler::BusConnect(GetEntityId());
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
        Physics::ColliderComponentEventBus::Handler::BusDisconnect();
        AZ::TransformNotificationBus::Handler::BusDisconnect();
        AzPhysics::SimulatedBodyComponentRequestsBus::Handler::BusDisconnect();
        Physics::RigidBodyRequestBus::Handler::BusDisconnect();

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

        if (m_rebuildPending)
        {
            m_rebuildPending = false;
            RebuildRigidBody();
            return;
        }

        AzPhysics::RigidBody* body = GetRigidBody();
        if (body == nullptr)
        {
            return;
        }

        // Writing an entity transform is not cheap - SetWorldTM dispatches on the
        // transform notification bus, propagates to children and dirties entity bounds and
        // visibility - and every rigid body in the level used to pay it every frame,
        // moving or not. That made the cost scale with how many bodies exist rather than
        // how many are moving, which defeats the point of Jolt letting them sleep.
        //
        // The gate is the pose itself rather than IsAwake, which covers three cases at
        // once: a settled body writes nothing, a body that changes pose *while* asleep
        // (a restored snapshot, a teleport) still reaches its entity, and a kinematic body
        // moved through SetKinematicTarget does too. That last one used to be excluded
        // outright, so a script- or C++-driven platform collided at its new pose while its
        // render mesh and every attached child stayed behind. Entity-driven moves record
        // the pose as they go (see OnTransformChanged), so this does not fight them.
        const AZ::Transform bodyTransform = body->GetTransform();
        if (bodyTransform.IsClose(m_lastSyncedTransform))
        {
            return;
        }
        m_lastSyncedTransform = bodyTransform;

        m_syncingTransformFromBody = true;
        AZ::TransformBus::Event(GetEntityId(), &AZ::TransformBus::Events::SetWorldTM, bodyTransform);
        m_syncingTransformFromBody = false;
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

            // The entity already holds this pose, so record it as synced: otherwise the
            // next tick would see the body agreeing with a stale cache and write the same
            // transform straight back.
            m_lastSyncedTransform = world;
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
    }

    void JoltRigidBodyComponent::DestroyRigidBody()
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

        m_syncingTransformFromBody = false;
    }

    void JoltRigidBodyComponent::OnColliderChanged()
    {
        // Never rebuild inside the bus dispatch; defer to the next tick.
        m_rebuildPending = true;
    }

    void JoltRigidBodyComponent::RebuildRigidBody()
    {
        if (m_bodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
        {
            return;
        }

        AZ::Vector3 linearVelocity = AZ::Vector3::CreateZero();
        AZ::Vector3 angularVelocity = AZ::Vector3::CreateZero();
        bool isAwake = true;
        if (AzPhysics::RigidBody* body = GetRigidBody())
        {
            linearVelocity = body->GetLinearVelocity();
            angularVelocity = body->GetAngularVelocity();
            isAwake = body->IsAwake();
        }

        DestroyRigidBody();

        // Stale collider data must not survive into the new body.
        m_configuration.m_colliderAndShapeData = AzPhysics::ShapeColliderPairList{};

        CreateRigidBody();

        if (AzPhysics::RigidBody* newBody = GetRigidBody())
        {
            newBody->SetLinearVelocity(linearVelocity);
            newBody->SetAngularVelocity(angularVelocity);
            if (!isAwake)
            {
                newBody->ForceAsleep();
            }
        }
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

    void JoltRigidBodyComponent::OnAfterEntitySet()
    {
        if (m_serializedIdentifier.empty())
        {
            m_serializedIdentifier = Internal::GenerateSerializedIdentifier(this);
        }
    }

    AZStd::string JoltRigidBodyComponent::GetSerializedIdentifier() const
    {
        return m_serializedIdentifier;
    }
} // namespace JoltPhysics
