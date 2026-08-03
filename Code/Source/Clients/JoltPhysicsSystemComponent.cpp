#include <Clients/JoltPhysicsSystemComponent.h>

#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/Console/IConsole.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Physics/Collision/CollisionLayers.h>
#include <AzFramework/Physics/Collision/CollisionGroups.h>
#include <AzFramework/Physics/Configuration/SceneConfiguration.h>

#include <ISystem.h>

#include <Joint/JoltJointConfiguration.h>
#include <Clients/Components/JoltMeshColliderComponent.h>
#include <JoltPhysics/Pipeline/JoltMeshAsset.h>
#include <Pipeline/JoltMeshAssetHandler.h>
#include <System/CollisionLayerFilters.h>
#include <System/JoltSystem.h>
#include <Shape/JoltShapeUtils.h>
#include <Shape/JoltMeshUtils.h>

#include <AzCore/IO/SystemFile.h>
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

        //! Adds an asset type and its file extension to the AssetCatalog and keeps the handler
        //! alive for the rest of the component's lifetime. Mirrors PhysX's RegisterAsset helper.
        template<typename AssetHandlerT, typename AssetT>
        void RegisterAsset(AZStd::vector<AZStd::unique_ptr<AZ::Data::AssetHandler>>& assetHandlers)
        {
            auto handler = AZStd::make_unique<AssetHandlerT>();
            handler->Register();
            AZ::Data::AssetCatalogRequestBus::Broadcast(&AZ::Data::AssetCatalogRequests::EnableCatalogForAsset, AZ::AzTypeInfo<AssetT>::Uuid());
            AZ::Data::AssetCatalogRequestBus::Broadcast(&AZ::Data::AssetCatalogRequests::AddExtension, AssetHandlerT::s_assetFileExtension);
            assetHandlers.emplace_back(AZStd::move(handler));
        }
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

        // Reflected here so that every process loading this gem (game launcher, editor and
        // Asset Processor alike) can ObjectStream-load .joltmesh product assets.
        Pipeline::JoltMeshAsset::Reflect(context);

        // Reflected here rather than from a component, as the other joint configurations
        // are: gear and rack-and-pinion have no components of their own yet.
        JoltGearJointConfiguration::Reflect(context);
        JoltRackAndPinionJointConfiguration::Reflect(context);

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
        // Activate() registers the .joltmesh asset type with the asset catalog, so the catalog
        // services must be up before this component activates (same reason PhysX declares these).
        dependent.push_back(AZ_CRC_CE("AssetDatabaseService"));
        dependent.push_back(AZ_CRC_CE("AssetCatalogService"));
    }

    void JoltPhysicsSystemComponent::Init()
    {
    }

    void JoltPhysicsSystemComponent::Activate()
    {
        m_defaultWorldComponent.Activate();

        // The .joltmesh handler is registered in the runtime component (not only in the builder)
        // because games must load .joltmesh product assets as well.
        RegisterAsset<Pipeline::JoltMeshAssetHandler, Pipeline::JoltMeshAsset>(m_assetHandlers);

        m_physicsSystem = GetJoltSystem();
        if (m_physicsSystem)
        {
            EnablePhysics();
        }

        Physics::SystemRequestBus::Handler::BusConnect();
        Physics::CollisionRequestBus::Handler::BusConnect();
        Physics::SystemDebugRequestBus::Handler::BusConnect();
        JoltPhysicsSystemRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();

        // Physics::System and Physics::SystemRequests are the same type (see SystemBus.h:
        // "using SystemRequests = System;"), but AZ::Interface<Physics::System> is a separate
        // registration mechanism from the SystemRequestBus above. Some callers (e.g. the
        // WhiteBox gem's EditorWhiteBoxColliderComponent) go through AZ::Interface<Physics::System>
        // specifically, so both need to resolve to this component.
        AZ::Interface<Physics::System>::Register(this);

        // Nothing else supplies wind in a Jolt project - the engine declares the interface
        // and PhysX, which this gem replaces, is the only backend that implements it.
        m_windProvider = AZStd::make_unique<JoltWindProvider>();
    }

    void JoltPhysicsSystemComponent::Deactivate()
    {
        AZ::Interface<Physics::System>::Unregister(this);

        m_windProvider.reset();

        AZ::TickBus::Handler::BusDisconnect();
        JoltPhysicsSystemRequestBus::Handler::BusDisconnect();
        Physics::SystemDebugRequestBus::Handler::BusDisconnect();
        Physics::CollisionRequestBus::Handler::BusDisconnect();
        Physics::SystemRequestBus::Handler::BusDisconnect();

        m_defaultWorldComponent.Deactivate();

        DisablePhysics();

        // Destroying the handlers unregisters them from the asset manager and AssetTypeInfoBus
        // (the catalog has no per-type unregister, so this is the full teardown). Done after
        // physics shutdown, mirroring PhysX, so that any JoltMeshAsset still in flight can be
        // released while its handler is still alive.
        m_assetHandlers.clear();
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

        // Jolt hands this renderer one primitive at a time, and each used to become two
        // heap allocations and a bus broadcast of its own - so a mesh or heightfield level
        // spent hundreds of thousands of allocations and broadcasts per frame, which made
        // the toggle unusable on exactly the scenes it exists to inspect.
        //
        // Primitives are accumulated by colour instead and flushed once. Jolt uses a
        // handful of colours (sleeping versus awake, per motion type), so a frame costs a
        // handful of broadcasts rather than one per triangle. The buffers are static so
        // they keep their capacity between frames rather than reallocating from nothing.
        struct BatchedGeometry
        {
            AZStd::unordered_map<AZ::u32, AZStd::vector<AZ::Vector3>> m_linesByColor;
            AZStd::unordered_map<AZ::u32, AZStd::vector<AZ::Vector3>> m_trianglesByColor;
        };
        static BatchedGeometry batched;

        for (auto& [color, points] : batched.m_linesByColor)
        {
            points.clear();
        }
        for (auto& [color, points] : batched.m_trianglesByColor)
        {
            points.clear();
        }

        settings.m_drawLineCB = [](const Physics::DebugDrawVertex& from, const Physics::DebugDrawVertex& to,
                                   [[maybe_unused]] const AZStd::shared_ptr<AzPhysics::SimulatedBody>& body,
                                   [[maybe_unused]] float thickness, void* udata)
        {
            auto* geometry = static_cast<BatchedGeometry*>(udata);
            AZStd::vector<AZ::Vector3>& points = geometry->m_linesByColor[from.m_color.ToU32()];
            points.push_back(from.m_position);
            points.push_back(to.m_position);
        };

        settings.m_drawTriBatchCB = [](const Physics::DebugDrawVertex* verts, AZ::u32 numVerts, const AZ::u32* indices,
                                       AZ::u32 numIndices,
                                       [[maybe_unused]] const AZStd::shared_ptr<AzPhysics::SimulatedBody>& body,
                                       void* udata)
        {
            if (numVerts == 0 || numIndices == 0)
            {
                return;
            }

            // Flattened rather than kept indexed: batches from different shapes cannot
            // share an index base, and re-basing every batch to merge them would cost more
            // than the vertices it saves at these sizes.
            auto* geometry = static_cast<BatchedGeometry*>(udata);
            AZStd::vector<AZ::Vector3>& points = geometry->m_trianglesByColor[verts[0].m_color.ToU32()];
            for (AZ::u32 i = 0; i < numIndices; ++i)
            {
                if (indices[i] < numVerts)
                {
                    points.push_back(verts[indices[i]].m_position);
                }
            }
        };

        settings.m_udata = &batched;

        DebugDrawPhysics(settings);

        // One broadcast per colour, for the whole scene.
        for (const auto& [packedColor, points] : batched.m_linesByColor)
        {
            if (!points.empty())
            {
                AZ::Color color;
                color.FromU32(packedColor);
                AzFramework::DebugDisplayRequestBus::Broadcast(
                    &AzFramework::DebugDisplayRequests::DrawLines, points, color);
            }
        }
        for (const auto& [packedColor, points] : batched.m_trianglesByColor)
        {
            if (!points.empty())
            {
                AZ::Color color;
                color.FromU32(packedColor);
                AzFramework::DebugDisplayRequestBus::Broadcast(
                    &AzFramework::DebugDisplayRequests::DrawTriangles, points, color);
            }
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
            // The project's saved configuration (Jolt Physics Configuration window)
            // lives in the settings registry; fall back to defaults for projects that
            // never saved one.
            const auto& registryManager = m_physicsSystem->GetSettingsRegistryManager();

            JoltSystemConfiguration config = registryManager.LoadSystemConfiguration()
                .value_or(JoltSystemConfiguration());
            m_physicsSystem->Initialize(&config);

            if (auto sceneConfig = registryManager.LoadDefaultSceneConfiguration())
            {
                m_physicsSystem->UpdateDefaultSceneConfiguration(*sceneConfig);
            }

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

    namespace
    {
        // Writes a cooked blob to disk. The blob is self-describing (magic/version header),
        // so this is a plain byte dump - there is no separate on-disk format.
        bool WriteCookedBlobToFile(const AZStd::string& filePath, const AZStd::vector<AZ::u8>& blob)
        {
            if (blob.empty())
            {
                return false;
            }

            AZ::IO::SystemFile file;
            if (!file.Open(filePath.c_str(),
                    AZ::IO::SystemFile::SF_OPEN_CREATE | AZ::IO::SystemFile::SF_OPEN_WRITE_ONLY))
            {
                AZ_Error("JoltPhysics", false, "Failed to open '%s' for writing cooked mesh data.", filePath.c_str());
                return false;
            }

            const AZ::IO::SystemFile::SizeType written = file.Write(blob.data(), blob.size());
            file.Close();

            if (written != blob.size())
            {
                AZ_Error("JoltPhysics", false, "Failed to write cooked mesh data to '%s' (%llu of %zu bytes).",
                    filePath.c_str(), static_cast<AZ::u64>(written), blob.size());
                return false;
            }
            return true;
        }
    }

    bool JoltPhysicsSystemComponent::CookConvexMeshToFile(
        const AZStd::string& filePath,
        const AZ::Vector3* vertices,
        AZ::u32 vertexCount)
    {
        return WriteCookedBlobToFile(filePath, JoltMeshUtils::PackConvexMesh(vertices, vertexCount));
    }

    bool JoltPhysicsSystemComponent::CookConvexMeshToMemory(
        const AZ::Vector3* vertices,
        AZ::u32 vertexCount,
        AZStd::vector<AZ::u8>& result)
    {
        // Jolt builds the hull from the raw point cloud on ConvexHullShapeSettings::Create(),
        // so "cooking" is just packing the points into the blob CreateConvexShapeFromCookedData
        // expects (see JoltMeshUtils).
        result = JoltMeshUtils::PackConvexMesh(vertices, vertexCount);
        return !result.empty();
    }

    bool JoltPhysicsSystemComponent::CookTriangleMeshToFile(
        const AZStd::string& filePath,
        const AZ::Vector3* vertices,
        AZ::u32 vertexCount,
        const AZ::u32* indices,
        AZ::u32 indexCount)
    {
        return WriteCookedBlobToFile(
            filePath, JoltMeshUtils::PackTriangleMesh(vertices, vertexCount, indices, indexCount));
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

    JPH::PhysicsSystem* JoltPhysicsSystemComponent::GetNativePhysicsSystem(AzPhysics::SceneHandle sceneHandle)
    {
        auto* systemInterface = AZ::Interface<AzPhysics::SystemInterface>::Get();
        if (!systemInterface || sceneHandle == AzPhysics::InvalidSceneHandle)
        {
            return nullptr;
        }

        // The cast is safe here because this gem owns the scene type; an extension gem
        // asking through this bus gets null rather than a foreign backend's pointer.
        auto* joltScene = azrtti_cast<JoltScene*>(systemInterface->GetScene(sceneHandle));
        return joltScene ? joltScene->GetJoltPhysicsSystem() : nullptr;
    }

    bool JoltPhysicsSystemComponent::SaveSimulationState(
        AzPhysics::SceneHandle sceneHandle, AZStd::vector<AZ::u8>& outState)
    {
        auto* systemInterface = AZ::Interface<AzPhysics::SystemInterface>::Get();
        if (!systemInterface || sceneHandle == AzPhysics::InvalidSceneHandle)
        {
            return false;
        }
        auto* joltScene = azrtti_cast<JoltScene*>(systemInterface->GetScene(sceneHandle));
        return joltScene && joltScene->SaveSimulationState(outState);
    }

    bool JoltPhysicsSystemComponent::RestoreSimulationState(
        AzPhysics::SceneHandle sceneHandle, AZStd::span<const AZ::u8> state)
    {
        auto* systemInterface = AZ::Interface<AzPhysics::SystemInterface>::Get();
        if (!systemInterface || sceneHandle == AzPhysics::InvalidSceneHandle)
        {
            return false;
        }
        auto* joltScene = azrtti_cast<JoltScene*>(systemInterface->GetScene(sceneHandle));
        return joltScene && joltScene->RestoreSimulationState(state);
    }

    AZ::u32 JoltPhysicsSystemComponent::AcquireObjectLayer(
        const AzPhysics::CollisionLayer& collisionLayer, const AzPhysics::CollisionGroups::Id& collisionGroupId, bool isMoving)
    {
        // Reuses the same registry every body this gem creates goes through, so an
        // extension gem's bodies are filtered identically to this gem's own.
        return JoltPhysics::AcquireObjectLayer(collisionLayer, collisionGroupId, isMoving);
    }

    bool JoltPhysicsSystemComponent::ObjectLayerMatchesQueryMask(AZ::u32 objectLayer, AZ::u64 collisionGroupMask)
    {
        return JoltPhysics::ObjectLayerMatchesQueryMask(static_cast<JPH::ObjectLayer>(objectLayer), collisionGroupMask);
    }

    AzPhysics::ShapeColliderPairList JoltPhysicsSystemComponent::GetColliderShapesFromMeshAsset(
        const Pipeline::JoltMeshAssetData& assetData,
        const Physics::ColliderConfiguration& colliderConfiguration,
        const AZ::Vector3& overallScale)
    {
        // The same call the Jolt Mesh Collider makes, so an extension gem's shapes come
        // out of an asset identically to a collider component's - including the per-shape
        // overrides and material slots the Scene Builder wrote into it.
        return ExpandJoltMeshAssetColliderShapes(assetData, colliderConfiguration, overallScale);
    }

    void JoltPhysicsSystemComponent::ApplyMaterialSlotsFromMeshAsset(
        const Pipeline::JoltMeshAssetData& assetData, bool useMaterialsFromAsset, Physics::MaterialSlots& materialSlots)
    {
        ApplyJoltMeshAssetMaterialSlots(assetData, useMaterialsFromAsset, materialSlots);
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
                    // Constraints carry their own debug view - for vehicles that is the
                    // suspension, wheel poses and an RPM meter, matching what the editor's
                    // wheel preview shows but live in game.
                    physicsSystem->DrawConstraints(&debugRenderer);
                }
            }
        }
    }

} // namespace JoltPhysics
