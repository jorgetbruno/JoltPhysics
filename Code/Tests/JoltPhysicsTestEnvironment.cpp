#include <Tests/JoltPhysicsTestEnvironment.h>

#include <AzCore/Memory/SystemAllocator.h>

namespace JoltPhysics
{
    void JoltPhysicsTestEnvironment::SetupEnvironment()
    {
        AZ::AllocatorInstance<AZ::SystemAllocator>::Create();
    }

    void JoltPhysicsTestEnvironment::TeardownEnvironment()
    {
        AZ::AllocatorInstance<AZ::SystemAllocator>::Destroy();
    }

} // namespace JoltPhysics

AZ_UNIT_TEST_HOOK(new JoltPhysics::JoltPhysicsTestEnvironment);
