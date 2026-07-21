#include <Clients/ComponentDescriptors.h>
#include <Clients/JoltPhysicsSystemComponent.h>
#include <Clients/Components/JoltBoxColliderComponent.h>
#include <Clients/Components/JoltSphereColliderComponent.h>
#include <Clients/Components/JoltCapsuleColliderComponent.h>
#include <Clients/Components/JoltRigidBodyComponent.h>
#include <Clients/Components/JoltStaticRigidBodyComponent.h>
#include <Clients/Components/JoltStaticCompoundColliderComponent.h>
#include <Clients/Components/JoltHeightfieldColliderComponent.h>

namespace JoltPhysics
{
    AZStd::list<AZ::ComponentDescriptor*> GetDescriptors()
    {
        AZStd::list<AZ::ComponentDescriptor*> descriptors;

        descriptors.push_back(JoltPhysicsSystemComponent::CreateDescriptor());
        descriptors.push_back(JoltBoxColliderComponent::CreateDescriptor());
        descriptors.push_back(JoltSphereColliderComponent::CreateDescriptor());
        descriptors.push_back(JoltCapsuleColliderComponent::CreateDescriptor());
        descriptors.push_back(JoltRigidBodyComponent::CreateDescriptor());
        descriptors.push_back(JoltStaticRigidBodyComponent::CreateDescriptor());
        descriptors.push_back(JoltStaticCompoundColliderComponent::CreateDescriptor());
        descriptors.push_back(JoltMutableCompoundColliderComponent::CreateDescriptor());
        descriptors.push_back(JoltHeightfieldColliderComponent::CreateDescriptor());

        return descriptors;
    }

} // namespace JoltPhysics
