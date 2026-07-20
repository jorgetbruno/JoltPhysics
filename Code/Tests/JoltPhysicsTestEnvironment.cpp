#include <JoltPhysicsTestEnvironment.h>

namespace JoltPhysics
{
    void JoltPhysicsTestEnvironment::SetupEnvironment()
    {
    }

    void JoltPhysicsTestEnvironment::TeardownEnvironment()
    {
    }

} // namespace JoltPhysics

AZ_UNIT_TEST_HOOK(new JoltPhysics::JoltPhysicsTestEnvironment);
