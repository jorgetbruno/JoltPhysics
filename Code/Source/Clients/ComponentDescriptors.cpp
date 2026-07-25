#include <Clients/ComponentDescriptors.h>
#include <Clients/JoltPhysicsSystemComponent.h>
#include <Clients/Components/JoltBoxColliderComponent.h>
#include <Clients/Components/JoltSphereColliderComponent.h>
#include <Clients/Components/JoltCapsuleColliderComponent.h>
#include <Clients/Components/JoltCylinderColliderComponent.h>
#include <Clients/Components/JoltMeshColliderComponent.h>
#include <Clients/Components/JoltRigidBodyComponent.h>
#include <Clients/Components/JoltStaticRigidBodyComponent.h>
#include <Clients/Components/JoltStaticCompoundColliderComponent.h>
#include <Clients/Components/JoltHeightfieldColliderComponent.h>
#include <Clients/Components/JoltCharacterControllerComponent.h>
#include <Clients/Components/JoltJointComponents.h>
#include <Clients/Components/JoltVehicleComponent.h>

namespace JoltPhysics
{
    AZStd::list<AZ::ComponentDescriptor*> GetDescriptors()
    {
        AZStd::list<AZ::ComponentDescriptor*> descriptors;

        descriptors.push_back(JoltPhysicsSystemComponent::CreateDescriptor());
        descriptors.push_back(JoltBoxColliderComponent::CreateDescriptor());
        descriptors.push_back(JoltSphereColliderComponent::CreateDescriptor());
        descriptors.push_back(JoltCapsuleColliderComponent::CreateDescriptor());
        descriptors.push_back(JoltCylinderColliderComponent::CreateDescriptor());
        descriptors.push_back(JoltMeshColliderComponent::CreateDescriptor());
        descriptors.push_back(JoltRigidBodyComponent::CreateDescriptor());
        descriptors.push_back(JoltStaticRigidBodyComponent::CreateDescriptor());
        descriptors.push_back(JoltStaticCompoundColliderComponent::CreateDescriptor());
        descriptors.push_back(JoltMutableCompoundColliderComponent::CreateDescriptor());
        descriptors.push_back(JoltHeightfieldColliderComponent::CreateDescriptor());
        descriptors.push_back(JoltCharacterControllerComponent::CreateDescriptor());
        descriptors.push_back(JoltFixedJointComponent::CreateDescriptor());
        descriptors.push_back(JoltBallJointComponent::CreateDescriptor());
        descriptors.push_back(JoltHingeJointComponent::CreateDescriptor());
        descriptors.push_back(JoltPrismaticJointComponent::CreateDescriptor());
        descriptors.push_back(JoltD6JointComponent::CreateDescriptor());
        descriptors.push_back(JoltDistanceJointComponent::CreateDescriptor());
        descriptors.push_back(JoltConeJointComponent::CreateDescriptor());
        descriptors.push_back(JoltSwingTwistJointComponent::CreateDescriptor());
        descriptors.push_back(JoltVehicleComponent::CreateDescriptor());

        return descriptors;
    }

} // namespace JoltPhysics
