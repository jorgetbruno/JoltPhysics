#pragma once

#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

namespace JoltPhysics
{
    class JoltPhysicsTestEnvironment : public AZ::Test::ITestEnvironment
    {
    public:
        void SetupEnvironment() override;
        void TeardownEnvironment() override;
    };

} // namespace JoltPhysics
