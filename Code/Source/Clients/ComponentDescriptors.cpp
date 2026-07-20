#include <Clients/ComponentDescriptors.h>
#include <Clients/JoltPhysicsSystemComponent.h>

namespace JoltPhysics
{
    AZStd::list<AZ::ComponentDescriptor*> GetDescriptors()
    {
        AZStd::list<AZ::ComponentDescriptor*> descriptors;

        descriptors.push_back(JoltPhysicsSystemComponent::CreateDescriptor());

        return descriptors;
    }

} // namespace JoltPhysics
