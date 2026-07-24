#include <Clients/JoltPhysicsSystemComponent.h>

#include <AzCore/Console/IConsole.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Physics/Collision/CollisionLayers.h>
#include <AzFramework/Physics/Collision/CollisionGroups.h>
#include <AzFramework/Physics/Configuration/SceneConfiguration.h>

#include <ISystem.h>

#include <System/JoltSystem.h>
#include <Shape/JoltShapeUtils.h>
#include <Shape/JoltMeshUtils.h>
#include <Scene/JoltScene.h>
#include <Debug/JoltDebugRenderer.h>
#include <Utils/Conversions.h>
#include <Utils/ReflectionUtils.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace JoltPhysics
{
    namespace
    {
        AZ_CVAR(int, jolt_Debug, 0, nullptr, AZ::ConsoleFunctorFlags::Null,
            "Draw Jolt physics collider shapes each frame (0 = off, 1 = wireframe).");
    }

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

        // Physics::System and Physics::SystemRequests are the same type (see SystemBus.h:
        // "using SystemRequests = System;"), but AZ::Interface<Physics::System> is a separate
        // registration mechanism from the SystemRequestBus above. Some callers (e.g. the
        // WhiteBox gem's EditorWhiteBoxColliderComponent) go through AZ::Interface<Physics::System>
        // specifically, so both need to resolve to this component.
        AZ::Interface<Physics::System>::Register(this);
    }

    void JoltPhysicsSystemComponent::Deactivate()
    {
        AZ::Interface<Physics::System>::Unregister(this);

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

        if (jolt_Debug != 0)
        {
            DrawColliderShapes();
        }
    }

    void JoltPhysicsSystemComponent::DrawColliderShapes()
    {
        Physics::DebugDrawSettings settings;
        settings.m_isWireframe = true;
        // Draw everything regardless of camera distance (debug toggle is meant to show all).
        settings.m_drawDistance = 100000.0f;

        settings.m_drawLineCB = [](const Physics::DebugDrawVertex& from, const Physics::DebugDrawVertex& to,
                                   [[maybe_unused]] const AZStd::shared_ptr<AzPhysics::SimulatedBody>& body,
                                   [[maybe_unused]] float thickness, [[maybe_unused]] void* udata)
        {
            const AZStd::vector<AZ::Vector3> points = { from.m_position, to.m_position };
            AzFramework::DebugDisplayRequestBus::Broadcast(
                &AzFramework::DebugDisplayRequests::DrawLines, points, from.m_color);
        };

        settings.m_drawTriBatchCB = [](const Physics::DebugDrawVertex* verts, AZ::u32 numVerts, const AZ::u32* indices,
                                       AZ::u32 numIndices,
                                       [[maybe_unused]] const AZStd::shared_ptr<AzPhysics::SimulatedBody>& body,
                                       [[maybe_unused]] void* udata)
        {
            AZStd::vector<AZ::Vector3> positions;
            AZStd::vector<AZ::u32> indexList;
            positions.reserve(numVerts);
            indexList.reserve(numIndices);
            for (AZ::u32 i = 0; i < numVerts; ++i)
            {
                positions.push_back(verts[i].m_position);
            }
            for (AZ::u32 i = 0; i < numIndices; ++i)
            {
                indexList.push_back(indices[i]);
            }
            const AZ::Color color = numVerts > 0 ? verts[0].m_color : AZ::Color(0.0f, 1.0f, 0.0f, 1.0f);
            AzFramework::DebugDisplayRequestBus::Broadcast(
                &AzFramework::DebugDisplayRequests::DrawTrianglesIndexed, positions, indexList, color);
        };

        DebugDrawPhysics(settings);
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

    void JoltPhysicsSystemComponent::ReleaseNativeMeshObject(void* nativeMeshObject)
    {
        // Balances the AddRef() taken in JoltShapeUtils::CreateJoltShapeFromConfig when a
        // CookedMesh shape is first built and cached on its configuration.
        if (nativeMeshObject)
        {
            static_cast<JPH::Shape*>(nativeMeshObject)->Release();
        }
    }

    void JoltPhysicsSystemComponent::ReleaseNativeHeightfieldObject(void* nativeHeightfieldObject)
    {
        // Balances the AddRef() taken in JoltHeightfieldColliderComponent::BuildHeightfieldShape
        // when the cached pointer is first set on the shape configuration. This is a *separate*
        // reference from the component's own m_nativeShape (JPH::RefConst), which manages its
        // own lifetime independently - do not assume this is the last reference to the shape.
        if (nativeHeightfieldObject)
        {
            static_cast<JPH::Shape*>(nativeHeightfieldObject)->Release();
        }
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
        const AZ::Vector3* vertices,
        AZ::u32 vertexCount,
        const AZ::u32* indices,
        AZ::u32 indexCount,
        AZStd::vector<AZ::u8>& result)
    {
        // Jolt needs no offline cooking pass (MeshShapeSettings::Create() builds its BVH
        // from a raw triangle list), so "cooking" here is just packing the geometry into
        // the blob format JoltMeshUtils::CreateMeshShapeFromCookedData expects.
        result = JoltMeshUtils::PackTriangleMesh(vertices, vertexCount, indices, indexCount);
        return !result.empty();
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
        debugRenderer.SetCameraPos(Conversions::ToJoltR(settings.m_cameraPos));
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
