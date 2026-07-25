#pragma once

#include <AzCore/Interface/Interface.h>
#include <AzFramework/Physics/Configuration/SystemConfiguration.h>
#include <AzFramework/Physics/PhysicsSystem.h>

namespace JoltPhysics::Editor
{
    //! Shared by both collision property handlers. Inline in a named namespace rather
    //! than duplicated in each .cpp because the unity build folds them into one
    //! translation unit, where two anonymous-namespace copies collide.
    //!
    //! Returns null before the physics system is up, which the editor property grid
    //! can hit while a level is loading.
    inline const AzPhysics::CollisionConfiguration* GetCollisionConfiguration()
    {
        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            if (const AzPhysics::SystemConfiguration* configuration = physicsSystem->GetConfiguration())
            {
                return &configuration->m_collisionConfig;
            }
        }
        return nullptr;
    }
} // namespace JoltPhysics::Editor
