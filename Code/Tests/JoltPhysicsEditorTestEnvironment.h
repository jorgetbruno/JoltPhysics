#pragma once

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/vector.h>
#include <AzTest/GemTestEnvironment.h>
#include <AzFramework/Application/Application.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

#include <Editor/EditorComponentDescriptors.h>

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
            // The gem's full editor descriptor list, so the tests see the same
            // reflection the editor does. This used to be a hand-picked subset because
            // the full list segfaulted teardown; the cause was a test that deleted a
            // descriptor singleton it did not own, leaving the application to release
            // freed memory - see JoltPhysicsEditorSystemComponentTests.
            const AZStd::list<AZ::ComponentDescriptor*> descriptorList = GetEditorDescriptors();

            // Copied into a named vector: a braced list makes overload resolution try
            // span's is_array_convertible, which forms ComponentDescriptor(*)[] on the
            // abstract base and hard-errors.
            const AZStd::vector<AZ::ComponentDescriptor*> descriptors(descriptorList.begin(), descriptorList.end());
            AddComponentDescriptors(descriptors);
        }
    };

} // namespace JoltPhysics
