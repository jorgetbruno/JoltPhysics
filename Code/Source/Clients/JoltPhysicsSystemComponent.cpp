#include <Clients/JoltPhysicsSystemComponent.h>

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include <System/JoltSystem.h>
#include <Shape/JoltShapeUtils.h>

namespace JoltPhysics
{
    void JoltPhysicsSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        JoltSystemConfiguration::Reflect(context);
        JoltSceneConfiguration::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltPhysicsSystemComponent, AZ::Component>()
                ->Version(1)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltPhysicsSystemComponent>(
                    "Jolt Physics System",
                    "Provides Jolt Physics backend for O3DE's AzPhysics interface")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void JoltPhysicsSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("PhysicsService"));
        provided.push_back(AZ_CRC_CE("JoltPhysicsService"));
    }

    void JoltPhysicsSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("PhysicsService"));
        incompatible.push_back(AZ_CRC_CE("PhysXService"));
    }

    void JoltPhysicsSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void JoltPhysicsSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    void JoltPhysicsSystemComponent::Init()
    {
    }

    void JoltPhysicsSystemComponent::Activate()
    {
        m_physicsSystem = GetJoltSystem();
        if (m_physicsSystem)
        {
            EnablePhysics();
        }

        Physics::SystemRequestBus::Handler::BusConnect();
        Physics::CollisionRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void JoltPhysicsSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        Physics::CollisionRequestBus::Handler::BusDisconnect();
        Physics::SystemRequestBus::Handler::BusDisconnect();

        DisablePhysics();
    }

    void JoltPhysicsSystemComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_enabled && m_physicsSystem)
        {
            m_physicsSystem->Simulate(deltaTime);
        }
    }

    int JoltPhysicsSystemComponent::GetTickOrder()
    {
        return AZ::TICK_PHYSICS_SYSTEM;
    }

    void JoltPhysicsSystemComponent::EnablePhysics()
    {
        if (!m_enabled && m_physicsSystem)
        {
            JoltSystemConfiguration config;
            m_physicsSystem->Initialize(&config);
            m_enabled = true;
        }
    }

    void JoltPhysicsSystemComponent::DisablePhysics()
    {
        if (m_enabled && m_physicsSystem)
        {
            m_physicsSystem->Shutdown();
            m_enabled = false;
        }
    }

    void JoltPhysicsSystemComponent::ActivatePhysicsSimulation()
    {
        EnablePhysics();
    }

    AZStd::shared_ptr<Physics::Shape> JoltPhysicsSystemComponent::CreateShape(
        const Physics::ColliderConfiguration& colliderConfiguration,
        const Physics::ShapeConfiguration& configuration)
    {
        return JoltShapeUtils::CreateShape(colliderConfiguration, configuration);
    }

    void JoltPhysicsSystemComponent::ReleaseNativeMeshObject([[maybe_unused]] void* nativeMeshObject)
    {
        // TODO: Implement mesh object release
    }

    void JoltPhysicsSystemComponent::ReleaseNativeHeightfieldObject([[maybe_unused]] void* nativeHeightfieldObject)
    {
        // TODO: Implement heightfield object release
    }

    bool JoltPhysicsSystemComponent::CookConvexMeshToFile(
        [[maybe_unused]] const AZStd::string& filePath,
        [[maybe_unused]] const AZ::Vector3* vertices,
        [[maybe_unused]] AZ::u32 vertexCount)
    {
        // TODO: Implement convex mesh cooking
        return false;
    }

    bool JoltPhysicsSystemComponent::CookConvexMeshToMemory(
        [[maybe_unused]] const AZ::Vector3* vertices,
        [[maybe_unused]] AZ::u32 vertexCount,
        [[maybe_unused]] AZStd::vector<AZ::u8>& result)
    {
        // TODO: Implement convex mesh cooking to memory
        return false;
    }

    bool JoltPhysicsSystemComponent::CookTriangleMeshToFile(
        [[maybe_unused]] const AZStd::string& filePath,
        [[maybe_unused]] const AZ::Vector3* vertices,
        [[maybe_unused]] AZ::u32 vertexCount,
        [[maybe_unused]] const AZ::u32* indices,
        [[maybe_unused]] AZ::u32 indexCount)
    {
        // TODO: Implement triangle mesh cooking
        return false;
    }

    bool JoltPhysicsSystemComponent::CookTriangleMeshToMemory(
        [[maybe_unused]] const AZ::Vector3* vertices,
        [[maybe_unused]] AZ::u32 vertexCount,
        [[maybe_unused]] const AZ::u32* indices,
        [[maybe_unused]] AZ::u32 indexCount,
        [[maybe_unused]] AZStd::vector<AZ::u8>& result)
    {
        // TODO: Implement triangle mesh cooking to memory
        return false;
    }

    AzPhysics::CollisionLayer JoltPhysicsSystemComponent::GetCollisionLayerByName(const AZStd::string& layerName)
    {
        AzPhysics::CollisionLayer layer;
        TryGetCollisionLayerByName(layerName, layer);
        return layer;
    }

    AZStd::string JoltPhysicsSystemComponent::GetCollisionLayerName(const AzPhysics::CollisionLayer& layer)
    {
        if (m_physicsSystem)
        {
            const auto& config = m_physicsSystem->GetJoltConfiguration();
            if (layer.GetIndex() < AzPhysics::CollisionLayers::MaxCollisionLayers)
            {
                return config.m_collisionConfig.m_collisionLayers.GetName(layer);
            }
        }
        return {};
    }

    bool JoltPhysicsSystemComponent::TryGetCollisionLayerByName(const AZStd::string& layerName, AzPhysics::CollisionLayer& layer)
    {
        if (m_physicsSystem)
        {
            const auto& config = m_physicsSystem->GetJoltConfiguration();
            return config.m_collisionConfig.m_collisionLayers.TryGetLayer(layerName, layer);
        }
        return false;
    }

    AzPhysics::CollisionGroup JoltPhysicsSystemComponent::GetCollisionGroupByName(const AZStd::string& groupName)
    {
        AzPhysics::CollisionGroup group;
        TryGetCollisionGroupByName(groupName, group);
        return group;
    }

    bool JoltPhysicsSystemComponent::TryGetCollisionGroupByName(const AZStd::string& groupName, AzPhysics::CollisionGroup& group)
    {
        if (m_physicsSystem)
        {
            const auto& config = m_physicsSystem->GetJoltConfiguration();
            return config.m_collisionConfig.m_collisionGroups.TryFindGroup(groupName, group);
        }
        return false;
    }

    AZStd::string JoltPhysicsSystemComponent::GetCollisionGroupName(const AzPhysics::CollisionGroup& group)
    {
        if (m_physicsSystem)
        {
            const auto& config = m_physicsSystem->GetJoltConfiguration();
            return config.m_collisionConfig.m_collisionGroups.FindGroupNameById(group.GetGroupId());
        }
        return {};
    }

    AzPhysics::CollisionGroup JoltPhysicsSystemComponent::GetCollisionGroupById(const AzPhysics::CollisionGroups::Id& groupId)
    {
        AzPhysics::CollisionGroup group;
        TryGetCollisionGroupById(groupId, group);
        return group;
    }

    bool JoltPhysicsSystemComponent::TryGetCollisionGroupById(
        const AzPhysics::CollisionGroups::Id& groupId,
        AzPhysics::CollisionGroup& group)
    {
        if (m_physicsSystem)
        {
            const auto& config = m_physicsSystem->GetJoltConfiguration();
            return config.m_collisionConfig.m_collisionGroups.TryFindGroup(groupId, group);
        }
        return false;
    }

    void JoltPhysicsSystemComponent::SetCollisionLayerName(int index, const AZStd::string& layerName)
    {
        if (m_physicsSystem)
        {
            m_physicsSystem->SetCollisionLayerName(index, layerName);
        }
    }

    void JoltPhysicsSystemComponent::CreateCollisionGroup(const AZStd::string& groupName, const AzPhysics::CollisionGroup& group)
    {
        if (m_physicsSystem)
        {
            m_physicsSystem->CreateCollisionGroup(groupName, group);
        }
    }

    AzPhysics::CollisionConfiguration JoltPhysicsSystemComponent::GetCollisionConfiguration()
    {
        if (m_physicsSystem)
        {
            return m_physicsSystem->GetJoltConfiguration().m_collisionConfig;
        }
        return {};
    }

} // namespace JoltPhysics
