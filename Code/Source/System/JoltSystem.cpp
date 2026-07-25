#include <System/JoltSystem.h>

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Console/ILogger.h>

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/PhysicsSettings.h>

#include <Scene/JoltScene.h>

namespace JoltPhysics
{
    AZ_CLASS_ALLOCATOR_IMPL(JoltSystem, AZ::SystemAllocator);

    namespace Internal
    {
        static void JoltTraceCallback(const char* format, ...)
        {
            va_list args;
            va_start(args, format);
            char buffer[1024];
            vsnprintf(buffer, sizeof(buffer), format, args);
            va_end(args);
            AZLOG_INFO("JoltPhysics: %s", buffer);
        }

#ifdef JPH_ENABLE_ASSERTS
        static bool JoltAssertCallback(const char* expression, const char* message, const char* file, unsigned int line)
        {
            AZ_Error("JoltPhysics", false, "Jolt Assert: %s (%s) at %s:%u", expression, message ? message : "", file, line);
            return true;
        }
#endif
    }

    JoltSystem::JoltSystem(AZStd::unique_ptr<JoltSettingsRegistryManager> registryManager)
        : m_registryManager(AZStd::move(registryManager))
    {
        JoltAllocator::Install();

        JPH::Trace = Internal::JoltTraceCallback;
#ifdef JPH_ENABLE_ASSERTS
        JPH::AssertFailed = Internal::JoltAssertCallback;
#endif

        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    }

    JoltSystem::~JoltSystem()
    {
        Shutdown();

        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;

        JoltAllocator::Uninstall();
    }

    void JoltSystem::Initialize(const AzPhysics::SystemConfiguration* config)
    {
        if (m_state == State::Initialized)
        {
            return;
        }

        if (config)
        {
            if (const auto* joltConfig = azdynamic_cast<const JoltSystemConfiguration*>(config))
            {
                m_systemConfig = *joltConfig;
            }
            else
            {
                m_systemConfig = JoltSystemConfiguration();
            }
        }

        m_allocator = AZStd::make_unique<JPH::TempAllocatorImpl>(AllocationArenaSize);

        m_materialManager = AZStd::make_unique<JoltMaterialManager>();
        m_materialManager->Init();

        const int numThreads = m_systemConfig.m_maxJobThreads > 0
            ? static_cast<int>(m_systemConfig.m_maxJobThreads)
            : static_cast<int>(AZStd::thread::hardware_concurrency()) - 1;
        m_jobSystem = AZStd::make_unique<JPH::JobSystemThreadPool>(
            JPH::cMaxPhysicsJobs,
            JPH::cMaxPhysicsBarriers,
            numThreads
        );

        // The registry hands out the object layers the three filters interpret, so it has
        // to be ready before any body is created.
        m_objectLayerRegistry.Initialize();
        m_broadPhaseInterface.Initialize(&m_objectLayerRegistry);
        m_objectVsBroadPhaseLayerFilter.Initialize(&m_objectLayerRegistry);
        m_objectLayerPairFilter.Initialize(&m_objectLayerRegistry);

        m_sceneInterface.Initialize(this);

        m_state = State::Initialized;

        AZLOG_INFO("JoltPhysics: System initialized with %d threads", numThreads);
    }

    void JoltSystem::Reinitialize()
    {
        if (m_state != State::Initialized)
        {
            return;
        }

        AZStd::vector<AzPhysics::SceneConfiguration> sceneConfigs;

        for (const auto& scene : m_sceneList)
        {
            if (scene)
            {
                sceneConfigs.push_back(scene->GetConfiguration());
            }
        }

        RemoveAllScenes();
        Shutdown();

        Initialize(&m_systemConfig);

        for (const auto& config : sceneConfigs)
        {
            AddScene(config);
        }
    }

    void JoltSystem::Shutdown()
    {
        if (m_state == State::Shutdown || m_state == State::Uninitialized)
        {
            return;
        }

        RemoveAllScenes();

        m_sceneInterface.Shutdown();

        if (m_materialManager)
        {
            m_materialManager->DeleteAllMaterials();
            m_materialManager.reset();
        }

        m_jobSystem.reset();
        m_allocator.reset();

        m_state = State::Shutdown;

        AZLOG_INFO("JoltPhysics: System shut down");
    }

    void JoltSystem::Simulate(float deltaTime)
    {
        if (m_state != State::Initialized)
        {
            return;
        }

        const float fixedDeltaTime = m_systemConfig.m_fixedTimestep;
        m_accumulatedTime += deltaTime;

        while (m_accumulatedTime >= fixedDeltaTime)
        {
            for (const auto& scene : m_sceneList)
            {
                if (scene && scene->IsEnabled())
                {
                    scene->StartSimulation(fixedDeltaTime);
                    scene->FinishSimulation();
                }
            }
            m_accumulatedTime -= fixedDeltaTime;
        }
    }

    AzPhysics::SceneHandle JoltSystem::AddScene(const AzPhysics::SceneConfiguration& config)
    {
        if (m_state != State::Initialized)
        {
            return AzPhysics::InvalidSceneHandle;
        }

        AzPhysics::SceneIndex sceneIndex;

        if (!m_freeSceneSlots.empty())
        {
            sceneIndex = m_freeSceneSlots.front();
            m_freeSceneSlots.pop();
        }
        else
        {
            sceneIndex = static_cast<AzPhysics::SceneIndex>(m_sceneList.size());
            m_sceneList.push_back(nullptr);
        }

        AzPhysics::SceneHandle handle(AZ::Crc32(config.m_sceneName), sceneIndex);

        auto scene = AZStd::make_unique<JoltScene>(config, handle);
        scene->InitializeJoltSystem();
        m_sceneList[sceneIndex] = AZStd::move(scene);

        AZLOG_INFO("JoltPhysics: Added scene '%s'", config.m_sceneName.c_str());

        return handle;
    }

    AzPhysics::SceneHandleList JoltSystem::AddScenes(const AzPhysics::SceneConfigurationList& configs)
    {
        AzPhysics::SceneHandleList handles;
        handles.reserve(configs.size());

        for (const auto& config : configs)
        {
            handles.push_back(AddScene(config));
        }

        return handles;
    }

    AzPhysics::SceneHandle JoltSystem::GetSceneHandle(const AZStd::string& sceneName)
    {
        const AZ::Crc32 sceneId(sceneName);

        for (AzPhysics::SceneIndex i = 0; i < m_sceneList.size(); ++i)
        {
            if (m_sceneList[i] && m_sceneList[i]->GetConfiguration().m_sceneName == sceneName)
            {
                return AzPhysics::SceneHandle(sceneId, i);
            }
        }

        return AzPhysics::InvalidSceneHandle;
    }

    AzPhysics::Scene* JoltSystem::GetScene(AzPhysics::SceneHandle handle)
    {
        if (handle == AzPhysics::InvalidSceneHandle)
        {
            return nullptr;
        }

        const auto index = AZStd::get<AzPhysics::SceneIndex>(handle);
        if (index < m_sceneList.size())
        {
            return m_sceneList[index].get();
        }

        return nullptr;
    }

    AzPhysics::SceneList JoltSystem::GetScenes(const AzPhysics::SceneHandleList& handles)
    {
        AzPhysics::SceneList scenes;
        scenes.reserve(handles.size());

        for (const auto& handle : handles)
        {
            scenes.emplace_back(GetScene(handle));
        }

        return scenes;
    }

    AzPhysics::SceneList& JoltSystem::GetAllScenes()
    {
        return m_sceneList;
    }

    void JoltSystem::RemoveScene(AzPhysics::SceneHandle handle)
    {
        if (handle == AzPhysics::InvalidSceneHandle)
        {
            return;
        }

        const auto index = AZStd::get<AzPhysics::SceneIndex>(handle);
        if (index < m_sceneList.size() && m_sceneList[index])
        {
            m_sceneList[index].reset();
            m_freeSceneSlots.push(index);
        }
    }

    void JoltSystem::RemoveScenes(const AzPhysics::SceneHandleList& handles)
    {
        for (const auto& handle : handles)
        {
            RemoveScene(handle);
        }
    }

    void JoltSystem::RemoveAllScenes()
    {
        m_sceneList.clear();

        while (!m_freeSceneSlots.empty())
        {
            m_freeSceneSlots.pop();
        }
    }

    AZStd::pair<AzPhysics::SceneHandle, AzPhysics::SimulatedBodyHandle> JoltSystem::FindAttachedBodyHandleFromEntityId(
        AZ::EntityId entityId)
    {
        for (const auto& scene : m_sceneList)
        {
            if (!scene)
            {
                continue;
            }

            for (const auto& [crc, body] : static_cast<JoltScene*>(scene.get())->GetSimulatedBodyList())
            {
                if (body && body->GetEntityId() == entityId)
                {
                    return { static_cast<JoltScene*>(scene.get())->GetSceneHandle(), body->m_bodyHandle };
                }
            }
        }

        return { AzPhysics::InvalidSceneHandle, AzPhysics::InvalidSimulatedBodyHandle };
    }

    const AzPhysics::SystemConfiguration* JoltSystem::GetConfiguration() const
    {
        return m_state == State::Initialized ? &m_systemConfig : nullptr;
    }

    void JoltSystem::UpdateConfiguration(const AzPhysics::SystemConfiguration* newConfig, bool forceReinitialization)
    {
        if (!newConfig)
        {
            return;
        }

        if (const auto* joltConfig = azdynamic_cast<const JoltSystemConfiguration*>(newConfig))
        {
            m_systemConfig = *joltConfig;

            // Solver settings apply to live scenes immediately; the capacity settings
            // in the same configuration only apply when a scene's physics system is
            // (re)created.
            for (auto& scene : m_sceneList)
            {
                if (auto* joltScene = azdynamic_cast<JoltScene*>(scene.get()))
                {
                    joltScene->ApplySystemConfiguration(m_systemConfig);
                }
            }

            // Listeners (e.g. the configuration window, collision property dropdowns)
            // refresh from this; the base-class event is how AzPhysics broadcasts it.
            m_configChangeEvent.Signal(&m_systemConfig);
        }

        if (forceReinitialization)
        {
            Reinitialize();
        }
    }

    void JoltSystem::UpdateDefaultSceneConfiguration(const AzPhysics::SceneConfiguration& sceneConfiguration)
    {
        if (m_defaultSceneConfiguration != sceneConfiguration)
        {
            m_defaultSceneConfiguration = sceneConfiguration;

            // The default world component listens for this and pushes the new
            // configuration (gravity etc.) into the live default scene.
            m_onDefaultSceneConfigurationChangedEvent.Signal(&m_defaultSceneConfiguration);
        }
    }

    const AzPhysics::SceneConfiguration& JoltSystem::GetDefaultSceneConfiguration() const
    {
        return m_defaultSceneConfiguration;
    }

    const JoltSystemConfiguration& JoltSystem::GetJoltConfiguration() const
    {
        return m_systemConfig;
    }

    const JoltSettingsRegistryManager& JoltSystem::GetSettingsRegistryManager() const
    {
        return *m_registryManager;
    }

    void JoltSystem::SetCollisionLayerName(int index, const AZStd::string& layerName)
    {
        if (index >= 0 && index < AzPhysics::CollisionLayers::MaxCollisionLayers)
        {
            m_systemConfig.m_collisionConfig.m_collisionLayers.SetName(
                static_cast<AZ::u8>(index), layerName);
        }
    }

    void JoltSystem::CreateCollisionGroup(const AZStd::string& groupName, const AzPhysics::CollisionGroup& group)
    {
        m_systemConfig.m_collisionConfig.m_collisionGroups.CreateGroup(groupName, group);
        m_collisionGroupMasks.push_back(group.GetMask());
    }

    AZ::u32 JoltSystem::GetCollisionGroupIndex(const AzPhysics::CollisionGroup& group) const
    {
        for (AZ::u32 i = 0; i < m_collisionGroupMasks.size(); ++i)
        {
            if (m_collisionGroupMasks[i] == group.GetMask())
            {
                return i;
            }
        }
        return 0;
    }

    AZ::u32 JoltSystem::GetCollisionGroupIndex(const AzPhysics::CollisionGroups::Id& groupId) const
    {
        const AzPhysics::CollisionGroup group = m_systemConfig.m_collisionConfig.m_collisionGroups.FindGroupById(groupId);
        return GetCollisionGroupIndex(group);
    }

    AZ::u64 JoltSystem::GetCollisionMask(AZ::u32 index) const
    {
        if (index < m_collisionGroupMasks.size())
        {
            return m_collisionGroupMasks[index];
        }
        return 0;
    }

    AZStd::vector<AZ::u64>* JoltSystem::GetCollisionMasks()
    {
        return &m_collisionGroupMasks;
    }

    JPH::TempAllocatorImpl* JoltSystem::GetJoltAllocator()
    {
        return m_allocator.get();
    }

    JPH::JobSystemThreadPool* JoltSystem::GetJoltJobSystem()
    {
        return m_jobSystem.get();
    }

    BroadPhaseLayerInterfaceImpl& JoltSystem::GetBroadPhaseLayerInterface()
    {
        return m_broadPhaseInterface;
    }

    ObjectVsBroadPhaseLayerFilterImpl& JoltSystem::GetObjectVsBroadPhaseLayerFilter()
    {
        return m_objectVsBroadPhaseLayerFilter;
    }

    ObjectLayerPairFilterImpl& JoltSystem::GetObjectLayerPairFilter()
    {
        return m_objectLayerPairFilter;
    }

    JoltObjectLayerRegistry& JoltSystem::GetObjectLayerRegistry()
    {
        return m_objectLayerRegistry;
    }

    JoltSystem* GetJoltSystem()
    {
        return static_cast<JoltSystem*>(AZ::Interface<AzPhysics::SystemInterface>::Get());
    }

} // namespace JoltPhysics

namespace JoltPhysics
{
    const AzPhysics::CollisionGroups::Id JoltSystemConfiguration::AllGroupId = []()
    {
        AzPhysics::CollisionGroups::Id id;
        id.m_id = AZ::Uuid::CreateString("{3E1A6F84-9C2D-4B57-8A0E-1D5F7C93B4A2}");
        return id;
    }();

    const AzPhysics::CollisionGroups::Id JoltSystemConfiguration::NoneGroupId = []()
    {
        AzPhysics::CollisionGroups::Id id;
        id.m_id = AZ::Uuid::CreateString("{7B4D2E90-58C1-4A63-9F2B-6E0A3D8C15F7}");
        return id;
    }();

    JoltSystemConfiguration::JoltSystemConfiguration()
    {
        // Layer 0 is AzPhysics::CollisionLayer::Default, the layer every collider gets
        // when nothing is chosen, so it is the one that has to be named.
        m_collisionConfig.m_collisionLayers.SetName(AzPhysics::CollisionLayer::Default, "Default");

        // Read-only so the configuration window shows them locked: a project can add
        // groups but removing "All" would break every collider that never picked one
        // (an empty group id resolves to All).
        m_collisionConfig.m_collisionGroups.CreateGroup(
            "All", AzPhysics::CollisionGroup::All, AllGroupId, /*readOnly*/ true);
        m_collisionConfig.m_collisionGroups.CreateGroup(
            "None", AzPhysics::CollisionGroup::None, NoneGroupId, /*readOnly*/ true);
    }

    void JoltSystemConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltSystemConfiguration, AzPhysics::SystemConfiguration>()
                ->Version(2)
                ->Field("MaxBodies", &JoltSystemConfiguration::m_maxBodies)
                ->Field("NumBodyMutexes", &JoltSystemConfiguration::m_numBodyMutexes)
                ->Field("MaxBodyPairs", &JoltSystemConfiguration::m_maxBodyPairs)
                ->Field("MaxContactConstraints", &JoltSystemConfiguration::m_maxContactConstraints)
                ->Field("TempAllocatorSize", &JoltSystemConfiguration::m_tempAllocatorSize)
                ->Field("MaxJobThreads", &JoltSystemConfiguration::m_maxJobThreads)
                ->Field("NumVelocitySteps", &JoltSystemConfiguration::m_numVelocitySteps)
                ->Field("NumPositionSteps", &JoltSystemConfiguration::m_numPositionSteps)
                ->Field("Baumgarte", &JoltSystemConfiguration::m_baumgarte)
                ->Field("SpeculativeContactDistance", &JoltSystemConfiguration::m_speculativeContactDistance)
                ->Field("PenetrationSlop", &JoltSystemConfiguration::m_penetrationSlop)
                ->Field("TimeBeforeSleep", &JoltSystemConfiguration::m_timeBeforeSleep)
                ->Field("PointVelocitySleepThreshold", &JoltSystemConfiguration::m_pointVelocitySleepThreshold)
                ->Field("AllowSleeping", &JoltSystemConfiguration::m_allowSleeping)
                ->Field("DeterministicSimulation", &JoltSystemConfiguration::m_deterministicSimulation)
                ->Field("CollisionSteps", &JoltSystemConfiguration::m_collisionSteps)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                // Shown by the Jolt Physics Configuration window's Global Configuration
                // tab. These take effect on the next physics system initialization.
                editContext->Class<JoltSystemConfiguration>("Jolt Configuration", "Jolt solver capacity settings")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSystemConfiguration::m_maxBodies,
                        "Max Bodies", "Maximum number of bodies the physics system supports.")
                        ->Attribute(AZ::Edit::Attributes::Min, 1u)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSystemConfiguration::m_numBodyMutexes,
                        "Body Mutexes", "Number of mutexes protecting body access. 0 uses Jolt's default.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSystemConfiguration::m_maxBodyPairs,
                        "Max Body Pairs", "Maximum number of body pairs the broadphase can queue for narrowphase.")
                        ->Attribute(AZ::Edit::Attributes::Min, 1u)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSystemConfiguration::m_maxContactConstraints,
                        "Max Contact Constraints", "Maximum number of contact constraints the solver can process in a step.")
                        ->Attribute(AZ::Edit::Attributes::Min, 1u)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSystemConfiguration::m_tempAllocatorSize,
                        "Temp Allocator Size", "Bytes reserved for the per-step temporary allocator.")
                        ->Attribute(AZ::Edit::Attributes::Min, 1024u)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSystemConfiguration::m_maxJobThreads,
                        "Max Job Threads", "Number of worker threads for the physics job system. 0 uses hardware concurrency minus one.")
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Solver Settings")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSystemConfiguration::m_numVelocitySteps,
                        "Velocity Steps", "Solver velocity iterations per step. More is more accurate and slower.")
                        ->Attribute(AZ::Edit::Attributes::Min, 1u)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSystemConfiguration::m_numPositionSteps,
                        "Position Steps", "Solver position iterations per step. More reduces penetration.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSystemConfiguration::m_baumgarte,
                        "Baumgarte", "Fraction of remaining penetration resolved each step. Higher is stiffer but can add energy.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 1.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSystemConfiguration::m_speculativeContactDistance,
                        "Speculative Contact Distance", "Radius around objects where speculative contacts are created. "
                        "Helps prevent fast objects tunneling; too high makes contacts feel soft.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSystemConfiguration::m_penetrationSlop,
                        "Penetration Slop", "Overlap bodies are allowed to keep, which stabilizes contacts. "
                        "Lower looks tighter but can jitter.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSystemConfiguration::m_timeBeforeSleep,
                        "Time Before Sleep", "How long a body must be still before it is put to sleep.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " s")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSystemConfiguration::m_pointVelocitySleepThreshold,
                        "Sleep Velocity Threshold", "Velocity below which a body counts as still for sleeping.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m/s")
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &JoltSystemConfiguration::m_allowSleeping,
                        "Allow Sleeping", "Whether resting bodies may sleep. Turn off to debug sleep-related issues.")
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &JoltSystemConfiguration::m_deterministicSimulation,
                        "Deterministic Simulation", "Run the simulation deterministically. Turning this off buys some performance.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSystemConfiguration::m_collisionSteps,
                        "Collision Steps", "Jolt collision sub-steps per physics update. Raising it improves fast-object "
                        "behavior at proportional CPU cost.")
                        ->Attribute(AZ::Edit::Attributes::Min, 1)
                        ->Attribute(AZ::Edit::Attributes::Max, 8)
                    ;
            }
        }
    }

} // namespace JoltPhysics
