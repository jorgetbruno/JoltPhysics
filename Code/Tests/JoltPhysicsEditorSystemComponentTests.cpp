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
        // Deliberately not owned. CreateDescriptor hands back a singleton that the
        // component system owns (AZ_COMPONENT backs it with an AZ::Environment
        // variable, so every call returns the same pointer), and disposing of it here
        // would leave the application holding a dangling descriptor to release at
        // teardown - which segfaulted the whole run for as long as this test wrapped
        // the pointer in a unique_ptr. A descriptor that does need disposing is
        // released with ReleaseDescriptor(), never delete.
        AZ::ComponentDescriptor* descriptor = JoltPhysicsEditorSystemComponent::CreateDescriptor();
        ASSERT_NE(descriptor, nullptr);
        EXPECT_STREQ(descriptor->GetName(), "JoltPhysicsEditorSystemComponent");

        // The same singleton, not a second descriptor.
        EXPECT_EQ(descriptor, JoltPhysicsEditorSystemComponent::CreateDescriptor());
    }

    TEST_F(JoltPhysicsEditorSystemComponentTests, TheEditorSystemComponentRequiresTheRuntimeOne)
    {
        // The editor half registers the property handlers and the configuration window,
        // both of which read through the physics system the runtime component installs.
        AZ::ComponentDescriptor::DependencyArrayType required;
        JoltPhysicsEditorSystemComponent::GetRequiredServices(required);
        EXPECT_NE(AZStd::find(required.begin(), required.end(), AZ_CRC_CE("JoltPhysicsService")), required.end());

        // And it must not coexist with PhysX's editor half: both register default
        // property handlers under the same AzFramework handler names.
        AZ::ComponentDescriptor::DependencyArrayType incompatible;
        JoltPhysicsEditorSystemComponent::GetIncompatibleServices(incompatible);
        EXPECT_NE(
            AZStd::find(incompatible.begin(), incompatible.end(), AZ_CRC_CE("PhysXEditorService")), incompatible.end());
    }

} // namespace JoltPhysics
