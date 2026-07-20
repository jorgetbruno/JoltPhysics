#include <Scene/JoltSceneInterface.h>
#include <System/JoltSystem.h>

namespace JoltPhysics
{
    void JoltSceneInterface::Initialize(JoltSystem* system)
    {
        m_system = system;
    }

    void JoltSceneInterface::Shutdown()
    {
        m_system = nullptr;
    }

} // namespace JoltPhysics
