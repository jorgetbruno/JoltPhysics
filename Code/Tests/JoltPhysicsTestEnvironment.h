#pragma once

#include <AzTest/GemTestEnvironment.h>
#include <AzFramework/Application/Application.h>

namespace JoltPhysics
{
    class JoltPhysicsTestEnvironment : public AZ::Test::GemTestEnvironment
    {
    protected:
        // Use AzFramework::Application so all AzFramework physics types
        // (ColliderConfiguration, MaterialSlots, ...) are reflected like in the editor.
        AZ::ComponentApplication* CreateApplicationInstance() override
        {
            return aznew AzFramework::Application();
        }

        // Registers the gem's component descriptors like the runtime module does.
        void AddGemsAndComponents() override;
    };

} // namespace JoltPhysics
