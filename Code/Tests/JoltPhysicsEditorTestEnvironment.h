#pragma once

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzTest/GemTestEnvironment.h>
#include <AzFramework/Application/Application.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

#include <Editor/EditorComponentDescriptors.h>
#include <Editor/Components/EditorJoltBoxColliderComponent.h>
#include <Editor/Components/EditorJoltCharacterControllerComponent.h>

namespace JoltPhysics
{
    class JoltPhysicsEditorTestEnvironment : public AZ::Test::GemTestEnvironment
    {
    protected:
        AZ::ComponentApplication* CreateApplicationInstance() override
        {
            // AzFramework::Application rather than ToolsApplication: the latter would
            // reflect every editor type but drags Qt-adjacent systems along and
            // crashes on teardown without a QApplication. The one editor type the
            // gem's components need beyond that - their EditorComponentBase base
            // class, without which JSON serialization halts on the base-class chain -
            // is reflected explicitly below.
            return aznew AzFramework::Application();
        }

        void PostCreateApplication() override
        {
            AZ::SerializeContext* serializeContext = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(
                serializeContext, &AZ::ComponentApplicationBus::Events::GetSerializeContext);
            if (serializeContext)
            {
                AzToolsFramework::Components::EditorComponentBase::Reflect(serializeContext);
            }
        }

        void AddGemsAndComponents() override
        {
            // The components under test. Registered individually rather than through
            // GetEditorDescriptors(): several of the other editor descriptors crash
            // this environment's teardown (tracked separately), and each component's
            // Reflect pulls in the configuration types it needs via ReflectOnce, so
            // the subset stays serialization-complete for these tests.
            //
            // Passed as a named array: a braced list makes overload resolution try
            // span's is_array_convertible, which forms ComponentDescriptor(*)[] on the
            // abstract base and hard-errors.
            AZ::ComponentDescriptor* descriptors[] = {
                EditorJoltCharacterControllerComponent::CreateDescriptor(),
                EditorJoltBoxColliderComponent::CreateDescriptor(),
            };
            AddComponentDescriptors(descriptors);
        }
    };

} // namespace JoltPhysics
