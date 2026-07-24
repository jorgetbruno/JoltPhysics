#include <Editor/EditorComponentDescriptors.h>
#include <Editor/JoltPhysicsEditorSystemComponent.h>
#include <Editor/Components/EditorJoltBoxColliderComponent.h>
#include <Editor/Components/EditorJoltSphereColliderComponent.h>
#include <Editor/Components/EditorJoltCapsuleColliderComponent.h>

namespace JoltPhysics
{
    AZStd::list<AZ::ComponentDescriptor*> GetEditorDescriptors()
    {
        AZStd::list<AZ::ComponentDescriptor*> descriptors;

        descriptors.push_back(JoltPhysicsEditorSystemComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltBoxColliderComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltSphereColliderComponent::CreateDescriptor());
        descriptors.push_back(EditorJoltCapsuleColliderComponent::CreateDescriptor());

        return descriptors;
    }

} // namespace JoltPhysics
