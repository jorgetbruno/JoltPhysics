#include <Tests/JoltPhysicsEditorTestEnvironment.h>

#include <AzCore/Memory/SystemAllocator.h>

namespace JoltPhysics
{
    void JoltPhysicsEditorTestEnvironment::SetupEnvironment()
    {
        AZ::AllocatorInstance<AZ::SystemAllocator>::Create();
    }

    void JoltPhysicsEditorTestEnvironment::TeardownEnvironment()
    {
        AZ::AllocatorInstance<AZ::SystemAllocator>::Destroy();
    }

} // namespace JoltPhysics

AZ_UNIT_TEST_HOOK(new JoltPhysics::JoltPhysicsEditorTestEnvironment);
