#include <Clients/JoltPhysicsSystemComponent.h>

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include <AzFramework/Physics/Collision/CollisionLayers.h>
#include <AzFramework/Physics/Collision/CollisionGroups.h>
#include <AzFramework/Physics/Configuration/SceneConfiguration.h>

#include <System/JoltSystem.h>
#include <Shape/JoltShapeUtils.h>
#include <Scene/JoltScene.h>
#include <Debug/JoltDebugRenderer.h>
#include <Utils/Conversions.h>
#include <Utils/ReflectionUtils.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace JoltPhysics
{
    void JoltPhysicsSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        // Reflect the AzPhysics/AzFramework configuration classes used by the Jolt
        // components, unless another module already reflected them.
        Internal::ReflectOnce<AzPhysics::CollisionLayer>(context);
        Internal::ReflectOnce<AzPhysics::CollisionLayers>(context);
        Internal::ReflectOnce<AzPhysics::CollisionGroup>(context);
        Internal::ReflectOnce<AzPhysics::CollisionGroups>(context);
        Internal::ReflectOnce<AzPhysics::CollisionConfiguration>(context);
        Internal::ReflectOnce<AzPhysics::SystemConfiguration>(context);
        Internal::ReflectOnce<AzPhysics::SceneConfiguration>(context);

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
        m_defaultWorldComponent.Activate();

        m_physicsSystem = GetJoltSystem();
        if (m_physicsSystem)
        {
            EnablePhysics();
        }

        Physics::SystemRequestBus::Handler::BusConnect();
        Physics::CollisionRequestBus::Handler::BusConnect();
        Physics::SystemDebugRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void JoltPhysicsSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        Physics::SystemDebugRequestBus::Handler::BusDisconnect();
        Physics::CollisionRequestBus::Handler::BusDisconnect();
        Physics::SystemRequestBus::Handler::BusDisconnect();

        m_defaultWorldComponent.Deactivate();

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
            return config.m_collisionConfig.m_collisionGroups.TryFindGroupByName(groupName, group);
        }
        return false;
    }

    AZStd::string JoltPhysicsSystemComponent::GetCollisionGroupName(const AzPhysics::CollisionGroup& group)
    {
        AZStd::string groupName;
        if (m_physicsSystem)
        {
            const auto& config = m_physicsSystem->GetJoltConfiguration();
            for (const auto& preset : config.m_collisionConfig.m_collisionGroups.GetPresets())
            {
                if (preset.m_group.GetMask() == group.GetMask())
                {
                    groupName = preset.m_name;
                    break;
                }
            }
        }
        return groupName;
    }

    AzPhysics::CollisionGroup JoltPhysicsSystemComponent::GetCollisionGroupById(const AzPhysics::CollisionGroups::Id& groupId)
    {
        if (m_physicsSystem)
        {
            const auto& config = m_physicsSystem->GetJoltConfiguration();
            return config.m_collisionConfig.m_collisionGroups.FindGroupById(groupId);
        }
        return AzPhysics::CollisionGroup::All;
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

    bool JoltPhysicsSystemComponent::ShouldCollide(
        const Physics::ColliderConfiguration& colliderConfigurationA,
        const Physics::ColliderConfiguration& colliderConfigurationB)
    {
        const AzPhysics::CollisionGroup groupA = GetCollisionGroupById(colliderConfigurationA.m_collisionGroupId);
        const AzPhysics::CollisionGroup groupB = GetCollisionGroupById(colliderConfigurationB.m_collisionGroupId);
        return groupA.IsSet(colliderConfigurationB.m_collisionLayer) && groupB.IsSet(colliderConfigurationA.m_collisionLayer);
    }

    namespace
    {
        class DistanceBodyDrawFilter final : public JPH::BodyDrawFilter
        {
        public:
            DistanceBodyDrawFilter(const AZ::Vector3& cameraPosition, float drawDistance)
                : m_cameraPositionSq(cameraPosition.GetLengthSq() > 0.0f ? Conversions::ToJolt(cameraPosition) : JPH::RVec3::sZero())
                , m_drawDistanceSq(drawDistance * drawDistance)
                , m_enabled(drawDistance > 0.0f)
            {
            }

            bool ShouldDraw(const JPH::Body& inBody) const override
            {
                if (!m_enabled)
                {
                    return true;
                }
                return (inBody.GetPosition() - m_cameraPositionSq).LengthSq() <= m_drawDistanceSq;
            }

        private:
            JPH::RVec3 m_cameraPositionSq;
            float m_drawDistanceSq = 0.0f;
            bool m_enabled = false;
        };
    }

    void JoltPhysicsSystemComponent::DebugDrawPhysics(const Physics::DebugDrawSettings& settings)
    {
        if (!m_physicsSystem)
        {
            return;
        }

        JPH::BodyManager::DrawSettings drawSettings;
        drawSettings.mDrawShape = true;
        drawSettings.mDrawShapeWireframe = settings.m_isWireframe;
        drawSettings.mDrawShapeColor = JPH::BodyManager::EShapeColor::SleepColor;
        drawSettings.mDrawCenterOfMassTransform = settings.m_drawBodyTransforms;

        JoltDebugRenderer debugRenderer(&settings);
        DistanceBodyDrawFilter bodyDrawFilter(settings.m_cameraPos, settings.m_drawDistance);

        for (const auto& scene : m_physicsSystem->GetAllScenes())
        {
            if (auto* joltScene = static_cast<JoltScene*>(scene.get()))
            {
                if (auto* physicsSystem = joltScene->GetJoltPhysicsSystem())
                {
                    physicsSystem->DrawBodies(drawSettings, &debugRenderer, &bodyDrawFilter);
                }
            }
        }
    }

} // namespace JoltPhysics
