#pragma once

#include <AzTest/GemTestEnvironment.h>
#include <AzFramework/Application/Application.h>

namespace JoltPhysics
{
    class JoltPhysicsEditorTestEnvironment : public AZ::Test::GemTestEnvironment
    {
    protected:
        AZ::ComponentApplication* CreateApplicationInstance() override
        {
            return aznew AzFramework::Application();
        }
    };

} // namespace JoltPhysics
