#include <Editor/EditorComponentDescriptors.h>
#include <Editor/JoltPhysicsEditorSystemComponent.h>

namespace JoltPhysics
{
    AZStd::list<AZ::ComponentDescriptor*> GetEditorDescriptors()
    {
        AZStd::list<AZ::ComponentDescriptor*> descriptors;

        descriptors.push_back(JoltPhysicsEditorSystemComponent::CreateDescriptor());

        return descriptors;
    }

} // namespace JoltPhysics
