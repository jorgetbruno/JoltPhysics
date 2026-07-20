#include <Clients/DefaultWorldComponent.h>

#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/Configuration/SceneConfiguration.h>

namespace JoltPhysics
{
    JoltDefaultWorldComponent::JoltDefaultWorldComponent()
        : m_onDefaultSceneConfigChangedHandler(
            [this](const AzPhysics::SceneConfiguration* config)
            {
                if (config != nullptr)
                {
                    UpdateDefaultConfiguration(*config);
                }
            })
    {
    }

    void JoltDefaultWorldComponent::Activate()
    {
        AzFramework::GameEntityContextEventBus::Handler::BusConnect();
        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            physicsSystem->RegisterOnDefaultSceneConfigurationChangedEventHandler(m_onDefaultSceneConfigChangedHandler);
        }
    }

    void JoltDefaultWorldComponent::Deactivate()
    {
        AzFramework::GameEntityContextEventBus::Handler::BusDisconnect();
        Physics::DefaultWorldBus::Handler::BusDisconnect();
        m_onDefaultSceneConfigChangedHandler.Disconnect();
    }

    AzPhysics::SceneHandle JoltDefaultWorldComponent::GetDefaultSceneHandle() const
    {
        return m_sceneHandle;
    }

    void JoltDefaultWorldComponent::OnPreGameEntitiesStarted()
    {
        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            AzPhysics::SceneConfiguration sceneConfig = physicsSystem->GetDefaultSceneConfiguration();
            sceneConfig.m_sceneName = AzPhysics::DefaultPhysicsSceneName;
            m_sceneHandle = physicsSystem->AddScene(sceneConfig);
            if (m_sceneHandle != AzPhysics::InvalidSceneHandle)
            {
                Physics::DefaultWorldBus::Handler::BusConnect();
            }
        }
    }

    void JoltDefaultWorldComponent::OnGameEntitiesReset()
    {
        Physics::DefaultWorldBus::Handler::BusDisconnect();
        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            physicsSystem->RemoveScene(m_sceneHandle);
        }
        m_sceneHandle = AzPhysics::InvalidSceneHandle;
    }

    void JoltDefaultWorldComponent::UpdateDefaultConfiguration(const AzPhysics::SceneConfiguration& config)
    {
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        if (physicsSystem == nullptr)
        {
            return;
        }

        auto* scene = physicsSystem->GetScene(m_sceneHandle);
        if (scene == nullptr)
        {
            return;
        }

        const AzPhysics::SceneConfiguration& currentConfig = scene->GetConfiguration();
        if (currentConfig != config)
        {
            scene->UpdateConfiguration(config);
        }
    }
} // namespace JoltPhysics
