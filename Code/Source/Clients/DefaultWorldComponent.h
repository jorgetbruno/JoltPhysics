#pragma once

#include <AzFramework/Entity/GameEntityContextBus.h>
#include <AzFramework/Physics/SystemBus.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>

namespace JoltPhysics
{
    // Sub component used by the system component for spawning the default physics scene.
    // Creates the default scene when the game context activates (before game entities start)
    // and removes it when the game entities reset.
    class JoltDefaultWorldComponent
        : private Physics::DefaultWorldBus::Handler
        , private AzFramework::GameEntityContextEventBus::Handler
    {
    public:
        JoltDefaultWorldComponent();
        ~JoltDefaultWorldComponent() override = default;

        void Activate();
        void Deactivate();

    private:
        // Physics::DefaultWorldBus
        AzPhysics::SceneHandle GetDefaultSceneHandle() const override;

        // AzFramework::GameEntityContextEventBus
        void OnPreGameEntitiesStarted() override;
        void OnGameEntitiesReset() override;

        void UpdateDefaultConfiguration(const AzPhysics::SceneConfiguration& config);

        AzPhysics::SceneHandle m_sceneHandle = AzPhysics::InvalidSceneHandle;
        AzPhysics::SystemEvents::OnDefaultSceneConfigurationChangedEvent::Handler m_onDefaultSceneConfigChangedHandler;
    };
} // namespace JoltPhysics
