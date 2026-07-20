#pragma once

#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

namespace JoltPhysics
{
    class JoltPhysicsEditorTestEnvironment : public AZ::Test::ITestEnvironment
    {
    public:
        void SetupEnvironment() override;
        void TeardownEnvironment() override;
    };

} // namespace JoltPhysics
