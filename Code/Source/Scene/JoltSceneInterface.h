#pragma once

#include <AzCore/Interface/Interface.h>
#include <AzFramework/Physics/PhysicsScene.h>

namespace JoltPhysics
{
    class JoltSystem;

    class JoltSceneInterface
    {
    public:
        void Initialize(JoltSystem* system);
        void Shutdown();

    private:
        JoltSystem* m_system = nullptr;
    };

} // namespace JoltPhysics
