#include <JoltPhysicsEditorTestEnvironment.h>

namespace JoltPhysics
{
    void JoltPhysicsEditorTestEnvironment::SetupEnvironment()
    {
    }

    void JoltPhysicsEditorTestEnvironment::TeardownEnvironment()
    {
    }

} // namespace JoltPhysics

AZ_UNIT_TEST_HOOK(new JoltPhysics::JoltPhysicsEditorTestEnvironment);
