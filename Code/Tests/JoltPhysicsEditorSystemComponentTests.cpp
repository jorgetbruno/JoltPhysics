#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <AzCore/Component/Component.h>
#include <AzCore/Component/Entity.h>

#include <Editor/JoltPhysicsEditorSystemComponent.h>
#include <Configuration/JoltSettingsRegistryManager.h>
#include <System/JoltSystem.h>

#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

//! The editor system component requires the runtime component's service; the test
//! fixture has a real JoltSystem but not the component that normally provides it,
//! so this stub stands in (mirrors JoltPhysicsSystemComponent's declaration). It
//! lives in the GLOBAL namespace on purpose: AZ_COMPONENT's RTTI injections belong
//! to ::AZ, and declaring it inside namespace JoltPhysics poisoned AZ::Adl lookups
//! for the whole translation unit.
class JoltPhysicsServiceStubComponent : public AZ::Component
{
public:
    AZ_COMPONENT(JoltPhysicsServiceStubComponent, "{A1B2C3D4-E5F6-47A8-9B0C-1D2E3F4A5B6C}");
    static void Reflect(AZ::ReflectContext*) {}
    static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltPhysicsService"));
    }
    void Activate() override {}
    void Deactivate() override {}
};

namespace JoltPhysics
{
    class JoltPhysicsEditorSystemComponentTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));
            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            // Entity activation looks the stub up by its descriptor, so register it first.
            AZ::ComponentApplicationBus::Broadcast(
                &AZ::ComponentApplicationRequests::RegisterComponentDescriptor,
                JoltPhysicsServiceStubComponent::CreateDescriptor());

            // The entity drives the component's (protected) lifecycle methods.
            m_entity = AZStd::make_unique<AZ::Entity>();
            m_entity->CreateComponent<JoltPhysicsServiceStubComponent>();
            m_editorSystemComponent = m_entity->CreateComponent<JoltPhysicsEditorSystemComponent>();
            m_entity->Init();
            m_entity->Activate();
        }

        void TearDown() override
        {
            m_entity->Deactivate();
            m_entity.reset();
            AZ::ComponentApplicationBus::Broadcast(
                &AZ::ComponentApplicationRequests::UnregisterComponentDescriptor,
                JoltPhysicsServiceStubComponent::CreateDescriptor());
            m_system->Shutdown();
            m_system.reset();
        }

        AzPhysics::Scene* GetEditorScene()
        {
            return m_system->GetScene(m_editorSystemComponent->GetEditorSceneHandle());
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AZStd::unique_ptr<AZ::Entity> m_entity;
        JoltPhysicsEditorSystemComponent* m_editorSystemComponent = nullptr;
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

    TEST_F(JoltPhysicsEditorSystemComponentTests, EditorSceneDisablesDuringPlayAndReenables)
    {
        AzPhysics::Scene* editorScene = GetEditorScene();
        ASSERT_NE(editorScene, nullptr);
        ASSERT_TRUE(editorScene->IsEnabled());

        // PhysX parity: the editor scene sleeps while the game world runs in
        // play-in-editor, and wakes when play stops. The notifications go out over
        // the bus, the way the editor sends them (the component methods are protected).
        AzToolsFramework::EditorEntityContextNotificationBus::Broadcast(
            &AzToolsFramework::EditorEntityContextNotificationBus::Events::OnStartPlayInEditorBegin);
        EXPECT_FALSE(editorScene->IsEnabled());

        AzToolsFramework::EditorEntityContextNotificationBus::Broadcast(
            &AzToolsFramework::EditorEntityContextNotificationBus::Events::OnStopPlayInEditor);
        EXPECT_TRUE(editorScene->IsEnabled());
    }

    TEST_F(JoltPhysicsEditorSystemComponentTests, EditorSceneIsRemovedOnDeactivate)
    {
        const AzPhysics::SceneHandle editorHandle = m_editorSystemComponent->GetEditorSceneHandle();
        ASSERT_NE(editorHandle, AzPhysics::InvalidSceneHandle);

        m_entity->Deactivate();

        EXPECT_EQ(m_editorSystemComponent->GetEditorSceneHandle(), AzPhysics::InvalidSceneHandle);
        EXPECT_EQ(m_system->GetScene(editorHandle), nullptr);

        // Re-activate so TearDown's deactivate is a no-op path the component survives.
        m_entity->Activate();
    }

    TEST_F(JoltPhysicsEditorSystemComponentTests, EditorSceneIsCreatedAndNamed)
    {
        const AzPhysics::SceneHandle editorHandle = m_editorSystemComponent->GetEditorSceneHandle();
        ASSERT_NE(editorHandle, AzPhysics::InvalidSceneHandle);

        // Consumers discover the editor scene by the engine-wide EditorScene name/id.
        EXPECT_EQ(AZStd::get<AZ::Crc32>(editorHandle), AzPhysics::EditorPhysicsSceneId);

        AzPhysics::Scene* editorScene = GetEditorScene();
        ASSERT_NE(editorScene, nullptr);
        EXPECT_STREQ(editorScene->GetConfiguration().m_sceneName.c_str(), AzPhysics::EditorPhysicsSceneName);
        EXPECT_TRUE(editorScene->IsEnabled());
    }

    TEST_F(JoltPhysicsEditorSystemComponentTests, EditorSceneSupportsEditModeQueries)
    {
        AzPhysics::Scene* editorScene = GetEditorScene();
        ASSERT_NE(editorScene, nullptr);

        auto slabCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto slabShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        slabShape->m_dimensions = AZ::Vector3(20.0f, 20.0f, 1.0f);
        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(slabCollider, slabShape);
        const AzPhysics::SimulatedBodyHandle slabHandle = editorScene->AddSimulatedBody(&slabConfig);
        ASSERT_NE(slabHandle, AzPhysics::InvalidSimulatedBodyHandle);

        AzPhysics::RayCastRequest request;
        request.m_start = AZ::Vector3(0.0f, 0.0f, 5.0f);
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 100.0f;
        const AzPhysics::SceneQueryHits hits = editorScene->QueryScene(&request);

        ASSERT_EQ(hits.m_hits.size(), 1u);
        EXPECT_EQ(hits.m_hits[0].m_bodyHandle, slabHandle);
    }

} // namespace JoltPhysics
