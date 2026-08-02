#include <System/JoltSystem.h>
#include <RigidBody/JoltStaticRigidBody.h>
#include <RigidBody/JoltRigidBody.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/CollisionBus.h>
#include <Utils/JoltComponentUtils.h>
#include <Clients/Components/JoltColliderComponentBase.h>

#include <AzCore/Component/NonUniformScaleBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Color.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/algorithm.h>

#include <AzFramework/Physics/ColliderComponentBus.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    namespace
    {
        // World uniform scale times any NonUniformScale component: the scale the render
        // mesh is drawn at, so the collision has to match (mirrors PhysX's
        // Utils::GetOverallScale).
        AZ::Vector3 GetOverallEntityScale(AZ::EntityId entityId)
        {
            float uniformScale = 1.0f;
            AZ::TransformBus::EventResult(uniformScale, entityId, &AZ::TransformBus::Events::GetWorldUniformScale);
            AZ::Vector3 nonUniformScale = AZ::Vector3::CreateOne();
            AZ::NonUniformScaleRequestBus::EventResult(nonUniformScale, entityId, &AZ::NonUniformScaleRequests::GetScale);
            return nonUniformScale * uniformScale;
        }
    } // namespace

    void JoltColliderComponentBase::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<Physics::ColliderConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->RegisterGenericType<AZStd::shared_ptr<Physics::ColliderConfiguration>>();

            serializeContext->Class<JoltColliderComponentBase, AZ::Component>()
                ->Version(1)
                ->Field("ColliderConfiguration", &JoltColliderComponentBase::m_colliderConfiguration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltColliderComponentBase>("Jolt Collider Base", "Base configuration for Jolt colliders")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltColliderComponentBase::m_colliderConfiguration,
                        "Collider Configuration", "Configuration shared by all Jolt colliders")
                    ;
            }
        }
    }

    void JoltColliderComponentBase::Activate()
    {
        Physics::CollisionFilteringRequestBus::Handler::BusConnect(GetEntityId());

        Physics::ColliderComponentEventBus::Event(
            GetEntityId(), &Physics::ColliderComponentEvents::OnColliderChanged);
    }

    void JoltColliderComponentBase::Deactivate()
    {
        Physics::CollisionFilteringRequestBus::Handler::BusDisconnect();

        Physics::ColliderComponentEventBus::Event(
            GetEntityId(), &Physics::ColliderComponentEvents::OnColliderChanged);
    }

    AzPhysics::ShapeColliderPairList JoltColliderComponentBase::GetShapeColliderPairs() const
    {
        AzPhysics::ShapeColliderPairList pairs{ GetShapeColliderPair() };
        ApplyOverallScale(pairs);
        return pairs;
    }

    bool JoltColliderComponentBase::TryFindCollisionLayer(
        const AZStd::string& layerName, AzPhysics::CollisionLayer& outLayer)
    {
        auto* joltSystem = GetJoltSystem();
        return joltSystem != nullptr &&
            joltSystem->GetJoltConfiguration().m_collisionConfig.m_collisionLayers.TryGetLayer(layerName, outLayer);
    }

    bool JoltColliderComponentBase::MatchesColliderTag(AZ::Crc32 colliderTag) const
    {
        // The empty tag addresses every collider on the entity, which is what a caller
        // that does not care about tags passes.
        const Physics::ColliderConfiguration& colliderConfig =
            const_cast<JoltColliderComponentBase*>(this)->GetColliderConfiguration();
        return colliderTag == AZ::Crc32() || colliderTag == AZ::Crc32(colliderConfig.m_tag.c_str());
    }

    void JoltColliderComponentBase::ApplyFilteringToBody()
    {
        AzPhysics::SimulatedBody* simulatedBody = nullptr;
        AzPhysics::SimulatedBodyComponentRequestsBus::EventResult(
            simulatedBody, GetEntityId(), &AzPhysics::SimulatedBodyComponentRequestsBus::Events::GetSimulatedBody);

        // Jolt can move a live body between object layers, so a filtering change costs a
        // re-resolve rather than a body rebuild.
        const Physics::ColliderConfiguration& colliderConfig = GetColliderConfiguration();
        if (auto* rigidBody = azrtti_cast<JoltRigidBody*>(simulatedBody))
        {
            rigidBody->SetObjectLayerFrom(colliderConfig.m_collisionLayer, colliderConfig.m_collisionGroupId);
        }
        else if (auto* staticBody = azrtti_cast<JoltStaticRigidBody*>(simulatedBody))
        {
            staticBody->SetObjectLayerFrom(colliderConfig.m_collisionLayer, colliderConfig.m_collisionGroupId);
        }
    }

    void JoltColliderComponentBase::SetCollisionLayer(const AZStd::string& layerName, AZ::Crc32 colliderTag)
    {
        if (!MatchesColliderTag(colliderTag))
        {
            return;
        }

        AzPhysics::CollisionLayer layer;
        if (!TryFindCollisionLayer(layerName, layer))
        {
            AZ_Warning("JoltPhysics", false,
                "SetCollisionLayer: no collision layer named '%s'. Layers are defined in the Jolt Physics "
                "Configuration window.", layerName.c_str());
            return;
        }

        GetColliderConfiguration().m_collisionLayer = layer;
        ApplyFilteringToBody();
    }

    AZStd::string JoltColliderComponentBase::GetCollisionLayerName()
    {
        auto* joltSystem = GetJoltSystem();
        if (!joltSystem)
        {
            return {};
        }
        return joltSystem->GetJoltConfiguration().m_collisionConfig.m_collisionLayers.GetName(
            GetColliderConfiguration().m_collisionLayer);
    }

    void JoltColliderComponentBase::SetCollisionGroup(const AZStd::string& groupName, AZ::Crc32 colliderTag)
    {
        if (!MatchesColliderTag(colliderTag))
        {
            return;
        }

        auto* joltSystem = GetJoltSystem();
        if (!joltSystem)
        {
            return;
        }

        const AzPhysics::CollisionGroups& groups =
            joltSystem->GetJoltConfiguration().m_collisionConfig.m_collisionGroups;
        const AzPhysics::CollisionGroups::Id groupId = groups.FindGroupIdByName(groupName);
        if (groups.FindGroupNameById(groupId) != groupName)
        {
            AZ_Warning("JoltPhysics", false,
                "SetCollisionGroup: no collision group named '%s'. Groups are defined in the Jolt Physics "
                "Configuration window.", groupName.c_str());
            return;
        }

        GetColliderConfiguration().m_collisionGroupId = groupId;
        ApplyFilteringToBody();
    }

    AZStd::string JoltColliderComponentBase::GetCollisionGroupName()
    {
        auto* joltSystem = GetJoltSystem();
        if (!joltSystem)
        {
            return {};
        }
        return joltSystem->GetJoltConfiguration().m_collisionConfig.m_collisionGroups.FindGroupNameById(
            GetColliderConfiguration().m_collisionGroupId);
    }

    void JoltColliderComponentBase::ToggleCollisionLayer(
        const AZStd::string& layerName, AZ::Crc32 colliderTag, bool enabled)
    {
        if (!MatchesColliderTag(colliderTag))
        {
            return;
        }

        AzPhysics::CollisionLayer layer;
        if (!TryFindCollisionLayer(layerName, layer))
        {
            AZ_Warning("JoltPhysics", false, "ToggleCollisionLayer: no collision layer named '%s'.", layerName.c_str());
            return;
        }

        // A collider stores which *named* group it uses, not a mask of its own, so an
        // arbitrary toggle has nowhere to live: the result is expressible only if some
        // configured group already has exactly that mask. Rather than silently doing
        // nothing (the behaviour this whole fix exists to remove), find one or say why not.
        auto* joltSystem = GetJoltSystem();
        if (!joltSystem)
        {
            return;
        }

        const AzPhysics::CollisionGroups& groups =
            joltSystem->GetJoltConfiguration().m_collisionConfig.m_collisionGroups;
        AzPhysics::CollisionGroup group = groups.FindGroupById(GetColliderConfiguration().m_collisionGroupId);
        group.SetLayer(layer, enabled);

        AzPhysics::CollisionGroups::Id matchingId;
        bool matched = false;
        for (const AzPhysics::CollisionGroups::Preset& preset : groups.GetPresets())
        {
            if (preset.m_group.GetMask() == group.GetMask())
            {
                matchingId = preset.m_id;
                matched = true;
                break;
            }
        }
        if (!matched)
        {
            AZ_Warning("JoltPhysics", false,
                "ToggleCollisionLayer: turning '%s' %s would produce a collision mask that no configured group "
                "has. A collider references a named group rather than carrying its own mask, so define a group "
                "with that combination in the Jolt Physics Configuration window and use SetCollisionGroup.",
                layerName.c_str(), enabled ? "on" : "off");
            return;
        }

        GetColliderConfiguration().m_collisionGroupId = matchingId;
        ApplyFilteringToBody();
    }

    void JoltColliderComponentBase::ApplyOverallScale(AzPhysics::ShapeColliderPairList& pairs) const
    {
        const AZ::Vector3 overallScale = GetOverallEntityScale(GetEntityId());
        if (overallScale == AZ::Vector3::CreateOne())
        {
            return;
        }

        for (AzPhysics::ShapeColliderPair& pair : pairs)
        {
            if (!pair.second || pair.second->GetShapeType() == Physics::ShapeType::Heightfield)
            {
                continue;
            }

            // Idempotent assignment on the component's own configuration: in this gem
            // the shape config carries no authored scale of its own, so m_scale is
            // exactly the entity scale (mirrors PhysX's UpdateScaleForShapeConfigs).
            pair.second->m_scale = overallScale;

            // The collider offset is authored in unscaled entity space and must scale
            // with the shape; clone rather than mutate the serialized offset.
            if (pair.first && !pair.first->m_position.IsZero())
            {
                pair.first = AZStd::make_shared<Physics::ColliderConfiguration>(*pair.first);
                pair.first->m_position *= overallScale;
            }
        }
    }

    void JoltColliderComponentBase::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltColliderService"));
    }

    void JoltColliderComponentBase::GetIncompatibleServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        // Multiple colliders per entity are allowed; they are combined into a
        // single compound simulated body (mirrors PhysX BaseColliderComponent).
    }

    void JoltColliderComponentBase::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void JoltColliderComponentBase::OnAfterEntitySet()
    {
        if (m_serializedIdentifier.empty())
        {
            m_serializedIdentifier = Internal::GenerateSerializedIdentifier(this);
        }
    }

    AZStd::string JoltColliderComponentBase::GetSerializedIdentifier() const
    {
        return m_serializedIdentifier;
    }
} // namespace JoltPhysics
