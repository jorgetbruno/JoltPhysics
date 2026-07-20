#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <Editor/JoltPhysicsEditorSystemComponent.h>

namespace JoltPhysics
{
    class JoltPhysicsEditorSystemComponentTests : public ::testing::Test
    {
    };

    TEST_F(JoltPhysicsEditorSystemComponentTests, DescriptorCanBeCreated)
    {
        AZStd::unique_ptr<AZ::ComponentDescriptor> descriptor(JoltPhysicsEditorSystemComponent::CreateDescriptor());
        EXPECT_NE(descriptor, nullptr);
    }

} // namespace JoltPhysics
