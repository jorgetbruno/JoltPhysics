#include <JoltPhysicsTestEnvironment.h>

#include <Clients/ComponentDescriptors.h>

namespace JoltPhysics
{
    void JoltPhysicsTestEnvironment::AddGemsAndComponents()
    {
        AZStd::list<AZ::ComponentDescriptor*> descriptors = GetDescriptors();
        AZStd::vector<AZ::ComponentDescriptor*> descriptorVector(descriptors.begin(), descriptors.end());
        AddComponentDescriptors(descriptorVector);
    }

} // namespace JoltPhysics

AZ_UNIT_TEST_HOOK(new JoltPhysics::JoltPhysicsTestEnvironment);
