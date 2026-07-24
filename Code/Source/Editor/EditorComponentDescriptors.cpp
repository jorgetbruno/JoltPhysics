#include <Editor/EditorComponentDescriptors.h>
#include <Editor/JoltPhysicsEditorSystemComponent.h>
#include <Editor/Components/EditorJoltBoxColliderComponent.h>
#include <Editor/Components/EditorJoltSphereColliderComponent.h>
#include <Editor/Components/EditorJoltCapsuleColliderComponent.h>
#include <Editor/Components/EditorJoltRigidBodyComponent.h>
#include <Editor/Components/EditorJoltStaticRigidBodyComponent.h>
#include <Editor/Components/EditorJoltHeightfieldColliderComponent.h>
#include <Editor/Components/EditorJoltStaticCompoundColliderComponent.h>
#include <Editor/Components/EditorJoltMutableCompoundColliderComponent.h>
#include <Editor/Components/EditorJoltCharacterControllerComponent.h>
#include <Editor/Components/EditorJoltVehicleComponent.h>
#include <Editor/Components/EditorJoltFixedJointComponent.h>
#include <Editor/Components/EditorJoltBallJointComponent.h>
#include <Editor/Components/EditorJoltHingeJointComponent.h>
#include <Editor/Components/EditorJoltPrismaticJointComponent.h>
#include <Editor/Components/EditorJoltD6JointComponent.h>

namespace JoltPhysics
{
    AZStd::list<AZ::ComponentDescriptor*> GetEditorDescriptors()
    {
        AZStd::list<AZ::ComponentDescriptor*> descriptors;

        descriptors.push_back(JoltPhysicsEditorSystemComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltBoxColliderComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltSphereColliderComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltCapsuleColliderComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltRigidBodyComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltStaticRigidBodyComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltHeightfieldColliderComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltStaticCompoundColliderComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltMutableCompoundColliderComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltCharacterControllerComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltVehicleComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltFixedJointComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltBallJointComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltHingeJointComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltPrismaticJointComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltD6JointComponent::CreateDescriptor());

        return descriptors;
    }

} // namespace JoltPhysics
