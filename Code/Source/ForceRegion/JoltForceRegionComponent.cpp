#include <ForceRegion/JoltForceRegionComponent.h>
#include <ForceRegion/JoltWindProvider.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/algorithm.h>

#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>

namespace JoltPhysics
{
    JoltForceRegionComponent::JoltForceRegionComponent(const JoltForceRegion& forceRegion, const AZStd::string& windTag)
        : m_forceRegion(forceRegion)
        , m_windTag(windTag)
    {
    }

    void JoltForceRegionComponent::Reflect(AZ::ReflectContext* context)
    {
        JoltForceRegion::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltForceRegionComponent, AZ::Component>()
                ->Version(1)
                ->Field("ForceRegion", &JoltForceRegionComponent::m_forceRegion)
                ->Field("WindTag", &JoltForceRegionComponent::m_windTag)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltForceRegionComponent>("Jolt Force Region",
                    "Applies forces to bodies inside a trigger collider")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::RemoveableByUser, true)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltForceRegionComponent::m_forceRegion,
                        "Forces", "Forces applied to every body inside the region.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltForceRegionComponent::m_windTag,
                        "Wind tag", "Publishes this region through the engine's wind interface under this tag. "
                        "Match it against the global or local wind tag in the Jolt configuration; leave empty "
                        "for a region that is not wind.")
                    ;
            }
        }
    }

    void JoltForceRegionComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltForceRegionService"));
    }

    void JoltForceRegionComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltForceRegionService"));
    }

    void JoltForceRegionComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        // A region is a trigger volume, so it needs geometry to be a volume of.
        required.push_back(AZ_CRC_CE("JoltColliderService"));
    }

    void JoltForceRegionComponent::Activate()
    {
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        if (!physicsSystem || !sceneInterface)
        {
            return;
        }

        m_sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        if (m_sceneHandle == AzPhysics::InvalidSceneHandle)
        {
            return;
        }

        // Occupancy comes from the scene's trigger events rather than an overlap query per
        // frame: the region only pays for bodies actually crossing its boundary.
        m_triggerHandler = AzPhysics::SceneEvents::OnSceneTriggersEvent::Handler(
            [this]([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, const AzPhysics::TriggerEventList& events)
            {
                for (const AzPhysics::TriggerEvent& triggerEvent : events)
                {
                    if (triggerEvent.m_triggerBody == nullptr ||
                        triggerEvent.m_triggerBody->GetEntityId() != GetEntityId())
                    {
                        continue;
                    }

                    auto existing =
                        AZStd::find(m_bodiesInRegion.begin(), m_bodiesInRegion.end(), triggerEvent.m_otherBodyHandle);
                    if (triggerEvent.m_type == AzPhysics::TriggerEvent::Type::Enter)
                    {
                        if (existing == m_bodiesInRegion.end())
                        {
                            m_bodiesInRegion.push_back(triggerEvent.m_otherBodyHandle);
                        }
                    }
                    else if (existing != m_bodiesInRegion.end())
                    {
                        m_bodiesInRegion.erase(existing);
                    }
                }
            });
        sceneInterface->RegisterSceneTriggersEventHandler(m_sceneHandle, m_triggerHandler);

        AZ::TickBus::Handler::BusConnect();

        // Wind is read far more often than regions appear, so the provider keeps a list
        // rather than searching entities each time it is asked.
        RegisterForceRegionForWind(this);
    }

    void JoltForceRegionComponent::Deactivate()
    {
        UnregisterForceRegionForWind(this);

        AZ::TickBus::Handler::BusDisconnect();
        m_triggerHandler.Disconnect();
        m_bodiesInRegion.clear();
        m_sceneHandle = AzPhysics::InvalidSceneHandle;
    }

    int JoltForceRegionComponent::GetTickOrder()
    {
        return AZ::TICK_PHYSICS;
    }

    JoltForceRegionParams JoltForceRegionComponent::BuildRegionParams() const
    {
        JoltForceRegionParams params;
        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        params.m_position = worldTransform.GetTranslation();
        params.m_rotation = worldTransform.GetRotation();
        return params;
    }

    AZ::Vector3 JoltForceRegionComponent::CalculateNetForce(const JoltForceRegionEntityParams& entity) const
    {
        return m_forceRegion.CalculateNetForce(entity, BuildRegionParams());
    }

    void JoltForceRegionComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_bodiesInRegion.empty() || m_forceRegion.m_forces.empty() || deltaTime <= 0.0f)
        {
            return;
        }

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        if (!sceneInterface)
        {
            return;
        }

        const JoltForceRegionParams regionParams = BuildRegionParams();

        for (auto it = m_bodiesInRegion.begin(); it != m_bodiesInRegion.end();)
        {
            AzPhysics::SimulatedBody* simulatedBody = sceneInterface->GetSimulatedBodyFromHandle(m_sceneHandle, *it);
            auto* rigidBody = azdynamic_cast<AzPhysics::RigidBody*>(simulatedBody);
            if (rigidBody == nullptr)
            {
                // Removed while inside, or never a body forces can act on. Its Exit event
                // will never arrive, so drop it here rather than retrying every frame.
                it = m_bodiesInRegion.erase(it);
                continue;
            }

            JoltForceRegionEntityParams entityParams;
            entityParams.m_position = rigidBody->GetPosition();
            entityParams.m_velocity = rigidBody->GetLinearVelocity();
            entityParams.m_aabb = rigidBody->GetAabb();
            entityParams.m_mass = rigidBody->GetMass();

            const AZ::Vector3 force = m_forceRegion.CalculateNetForce(entityParams, regionParams);
            if (!force.IsZero())
            {
                // Force over the frame becomes an impulse, so the result does not depend on
                // frame rate. A sleeping body feels nothing until something wakes it, which
                // matches PhysX and keeps a region from holding the whole scene awake.
                rigidBody->ApplyLinearImpulse(force * deltaTime);
            }
            ++it;
        }
    }
} // namespace JoltPhysics
