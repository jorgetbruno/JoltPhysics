#include <Editor/Components/EditorJoltJointComponentBase.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace JoltPhysics
{
    void EditorJoltJointComponentBase::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltJointComponentBase, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(1)
                ->Field("Configuration", &EditorJoltJointComponentBase::m_configuration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                // The JoltJointComponentConfiguration field-level edit context is registered
                // by the runtime JoltJointComponentBase::Reflect, which also runs in this dll.
                editContext->Class<EditorJoltJointComponentBase>(
                    "Jolt Joint Base", "Base configuration shared by Jolt joints")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltJointComponentBase::m_configuration,
                        "Configuration", "Lead/follower entities and local joint frame")
                    ;
            }
        }
    }

    void EditorJoltJointComponentBase::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltJointService"));
    }

    void EditorJoltJointComponentBase::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltJointService"));
    }

    void EditorJoltJointComponentBase::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

} // namespace JoltPhysics
