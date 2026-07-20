#pragma once

#include <AzCore/Component/TickBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/Configuration/SystemConfiguration.h>

#include <Configuration/JoltSettingsRegistryManager.h>
#include <JoltPhysics/Configuration/JoltConfiguration.h>
#include <Material/JoltMaterialManager.h>
#include <Scene/JoltSceneInterface.h>
#include <System/CollisionLayerFilters.h>
#include <System/JoltAllocator.h>

namespace JPH
{
    class TempAllocatorImpl;
    class JobSystemThreadPool;
}

namespace JoltPhysics
{
    class JoltPhysicsMaterial;

    class JoltSystem
        : public AZ::Interface<AzPhysics::SystemInterface>::Registrar
    {
    public:
        AZ_CLASS_ALLOCATOR_DECL;
        AZ_RTTI(JoltSystem, "{39383D29-C69C-4C36-84EE-874A3B5C9106}", AzPhysics::SystemInterface);

        JoltSystem(AZStd::unique_ptr<JoltSettingsRegistryManager> registryManager);
        virtual ~JoltSystem();

        void Initialize(const AzPhysics::SystemConfiguration* config) override;
        void Reinitialize() override;
        void Shutdown() override;
        void Simulate(float deltaTime) override;

        AzPhysics::SceneHandle AddScene(const AzPhysics::SceneConfiguration& config) override;
        AzPhysics::SceneHandleList AddScenes(const AzPhysics::SceneConfigurationList& configs) override;
        AzPhysics::SceneHandle GetSceneHandle(const AZStd::string& sceneName) override;
        AzPhysics::Scene* GetScene(AzPhysics::SceneHandle handle) override;
        AzPhysics::SceneList GetScenes(const AzPhysics::SceneHandleList& handles) override;
        AzPhysics::SceneList& GetAllScenes() override;
        void RemoveScene(AzPhysics::SceneHandle handle) override;
        void RemoveScenes(const AzPhysics::SceneHandleList& handles) override;
        void RemoveAllScenes() override;

        AZStd::pair<AzPhysics::SceneHandle, AzPhysics::SimulatedBodyHandle> FindAttachedBodyHandleFromEntityId(AZ::EntityId entityId) override;

        const AzPhysics::SystemConfiguration* GetConfiguration() const override;
        void UpdateConfiguration(const AzPhysics::SystemConfiguration* newConfig, bool forceReinitialization = false) override;
        void UpdateDefaultSceneConfiguration(const AzPhysics::SceneConfiguration& sceneConfiguration) override;
        const AzPhysics::SceneConfiguration& GetDefaultSceneConfiguration() const override;

        const JoltSystemConfiguration& GetJoltConfiguration() const;
        const JoltSettingsRegistryManager& GetSettingsRegistryManager() const;

        void SetCollisionLayerName(int index, const AZStd::string& layerName);
        void CreateCollisionGroup(const AZStd::string& groupName, const AzPhysics::CollisionGroup& group);

        //! Creates a collision group preset and returns its id (for use in collider configurations).
        AzPhysics::CollisionGroups::Id CreateCollisionGroupPreset(
            const AZStd::string& groupName, const AzPhysics::CollisionGroup& group)
        {
            return m_systemConfig.m_collisionConfig.m_collisionGroups.CreateGroup(groupName, group);
        }

        AZ::u32 GetCollisionGroupIndex(const AzPhysics::CollisionGroup& group) const;
        AZ::u32 GetCollisionGroupIndex(const AzPhysics::CollisionGroups::Id& groupId) const;

        AZ::u64 GetCollisionMask(AZ::u32 index) const;
        AZStd::vector<AZ::u64>* GetCollisionMasks();

        JPH::TempAllocatorImpl* GetJoltAllocator();
        JPH::JobSystemThreadPool* GetJoltJobSystem();
        BroadPhaseLayerInterfaceImpl& GetBroadPhaseLayerInterface();
        ObjectVsBroadPhaseLayerFilterImpl& GetObjectVsBroadPhaseLayerFilter();
        ObjectLayerPairFilterImpl& GetObjectLayerPairFilter();

    private:
        AZStd::vector<AZ::u64> m_collisionGroupMasks;
        JoltSystemConfiguration m_systemConfig;
        AzPhysics::SceneConfiguration m_defaultSceneConfiguration;
        AzPhysics::SceneList m_sceneList;
        AZStd::queue<AzPhysics::SceneIndex> m_freeSceneSlots;

        float m_accumulatedTime = 0.0f;

        enum class State : AZ::u8
        {
            Uninitialized = 0,
            Initialized,
            Shutdown
        };
        State m_state = State::Uninitialized;

        static constexpr unsigned int AllocationArenaSize = 256 * 1024 * 1024;

        JoltPhysicsMaterial* m_defaultMaterial = nullptr;

        AZStd::unique_ptr<JPH::TempAllocatorImpl> m_allocator;
        AZStd::unique_ptr<JPH::JobSystemThreadPool> m_jobSystem;

        AZStd::unique_ptr<JoltMaterialManager> m_materialManager;

        BroadPhaseLayerInterfaceImpl m_broadPhaseInterface;
        ObjectVsBroadPhaseLayerFilterImpl m_objectVsBroadPhaseLayerFilter;
        ObjectLayerPairFilterImpl m_objectLayerPairFilter;

        AZStd::unique_ptr<JoltSettingsRegistryManager> m_registryManager;
        JoltSceneInterface m_sceneInterface;
    };

    JoltSystem* GetJoltSystem();

} // namespace JoltPhysics
