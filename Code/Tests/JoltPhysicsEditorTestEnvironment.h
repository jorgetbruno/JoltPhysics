#pragma once

#include <AzTest/GemTestEnvironment.h>
#include <AzFramework/Application/Application.h>
#include <AzFramework/Components/TransformComponent.h>

#include <Editor/Components/EditorJoltCharacterControllerComponent.h>

namespace JoltPhysics
{
    class JoltPhysicsEditorTestEnvironment : public AZ::Test::GemTestEnvironment
    {
    protected:
        AZ::ComponentApplication* CreateApplicationInstance() override
        {
            return aznew AzFramework::Application();
        }

        void AddGemsAndComponents() override
        {
            // Enough to activate an editor component on an entity: the component under
            // test plus the TransformComponent its GetRequiredServices asks for.
            //
            // Passed as a named array rather than a braced list: the braced form makes
            // overload resolution try the span overload, whose is_array_convertible
            // check forms ComponentDescriptor(*)[] on the abstract base and hard-errors.
            AZ::ComponentDescriptor* descriptors[] = {
                EditorJoltCharacterControllerComponent::CreateDescriptor(),
                AzFramework::TransformComponent::CreateDescriptor(),
            };
            AddComponentDescriptors(descriptors);
        }
    };

} // namespace JoltPhysics
