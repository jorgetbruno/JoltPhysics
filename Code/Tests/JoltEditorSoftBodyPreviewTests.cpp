#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <AzCore/Component/Entity.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Physics/PhysicsScene.h>

#include <Clients/Components/JoltSoftBodyComponent.h>
#include <Configuration/JoltSettingsRegistryManager.h>
#include <Editor/Components/EditorJoltSoftBodyComponent.h>
#include <Editor/JoltPhysicsEditorSystemComponent.h>
#include <System/JoltSystem.h>

//! Provides the runtime component's service so the editor system component can activate
//! in the fixture (global namespace because AZ_COMPONENT's RTTI injections belong to ::AZ).
class JoltEditorSoftBodyServiceStub : public AZ::Component
{
public:
    AZ_COMPONENT(JoltEditorSoftBodyServiceStub, "{5E2C1B9D-4F7A-4E3B-8D6C-2A9F1E5B7C4D}");
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
    //! The editor soft body's live preview: a real soft body simulating in the edit-mode
    //! scene (EditorWorldBus), so cloth drapes in the viewport before play is pressed.
    class JoltEditorSoftBodyPreviewTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));
            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AZ::ComponentApplicationBus::Broadcast(
                &AZ::ComponentApplicationRequests::RegisterComponentDescriptor,
                JoltEditorSoftBodyServiceStub::CreateDescriptor());

            // The editor system component hosts the editor scene the preview attaches to.
            m_systemEntity = AZStd::make_unique<AZ::Entity>("EditorSystem");
            m_systemEntity->CreateComponent<JoltEditorSoftBodyServiceStub>();
            m_editorSystemComponent = m_systemEntity->CreateComponent<JoltPhysicsEditorSystemComponent>();
            m_systemEntity->Init();
            m_systemEntity->Activate();
        }

        void TearDown() override
        {
            if (m_entity)
            {
                if (m_entity->GetState() == AZ::Entity::State::Active)
                {
                    m_entity->Deactivate();
                }
                m_entity.reset();
            }
            m_systemEntity->Deactivate();
            m_systemEntity.reset();
            AZ::ComponentApplicationBus::Broadcast(
                &AZ::ComponentApplicationRequests::UnregisterComponentDescriptor,
                JoltEditorSoftBodyServiceStub::CreateDescriptor());
            m_system->Shutdown();
            m_system.reset();
        }

        //! Steps the editor scene the way the system tick would in the editor.
        void SimulateEditorSeconds(float seconds)
        {
            AzPhysics::Scene* editorScene = m_system->GetScene(m_editorSystemComponent->GetEditorSceneHandle());
            ASSERT_NE(editorScene, nullptr);
            const float fixedDeltaTime = 1.0f / 60.0f;
            const int steps = static_cast<int>(seconds / fixedDeltaTime);
            for (int i = 0; i < steps; ++i)
            {
                editorScene->StartSimulation(fixedDeltaTime);
                editorScene->FinishSimulation();
            }
        }

        EditorJoltSoftBodyComponent* MakeSoftBodyEntity()
        {
            m_entity = AZStd::make_unique<AZ::Entity>("EditorSoftBodyEntity");
            m_entity->CreateComponent<AzFramework::TransformComponent>();
            auto* softBody = m_entity->CreateComponent<EditorJoltSoftBodyComponent>();
            m_entity->Init();
            m_entity->Activate();
            return softBody;
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AZStd::unique_ptr<AZ::Entity> m_systemEntity;
        AZStd::unique_ptr<AZ::Entity> m_entity;
        JoltPhysicsEditorSystemComponent* m_editorSystemComponent = nullptr;
    };

    TEST_F(JoltEditorSoftBodyPreviewTests, LivePreviewSimulatesInTheEditorScene)
    {
        EditorJoltSoftBodyComponent* softBody = MakeSoftBodyEntity();

        // Off by default: the editor draws the rest shape and hosts no body.
        EXPECT_FALSE(softBody->HasLivePreviewBody());

        // Unpinned, so the fall below is unambiguous - the default corner pinning would
        // hold the sheet's top edge at its creation height.
        softBody->GetSettings().m_pinning = JoltSoftBodyPinning::None;
        softBody->GetSettings().m_allowSleeping = false;
        softBody->SetLivePreviewEnabled(true);
        ASSERT_TRUE(softBody->HasLivePreviewBody());

        // An unpinned cloth in the edit-mode scene falls when the scene steps, which is
        // what makes this a preview of the simulation rather than a drawing.
        const AZ::Aabb before = softBody->GetPreviewBody()->GetWorldBounds();
        ASSERT_TRUE(before.IsValid());

        SimulateEditorSeconds(1.0f);

        const AZ::Aabb after = softBody->GetPreviewBody()->GetWorldBounds();
        ASSERT_TRUE(after.IsValid());
        EXPECT_LT(after.GetMax().GetZ(), before.GetMax().GetZ() - 1.0f);

        // Turning the preview off removes the body from the editor scene.
        softBody->SetLivePreviewEnabled(false);
        EXPECT_FALSE(softBody->HasLivePreviewBody());
    }

    TEST_F(JoltEditorSoftBodyPreviewTests, DeactivationRemovesThePreviewBody)
    {
        EditorJoltSoftBodyComponent* softBody = MakeSoftBodyEntity();
        softBody->SetLivePreviewEnabled(true);
        ASSERT_TRUE(softBody->HasLivePreviewBody());

        // The preview body is editor furniture owned by the component; deactivating the
        // entity must not leave it behind in the editor scene.
        m_entity->Deactivate();
        EXPECT_FALSE(softBody->HasLivePreviewBody());
        m_entity.reset();
    }

    TEST_F(JoltEditorSoftBodyPreviewTests, BuildGameEntityCarriesTheSettingsButNotThePreview)
    {
        EditorJoltSoftBodyComponent* softBody = MakeSoftBodyEntity();
        softBody->SetLivePreviewEnabled(true);

        AZ::Entity gameEntity("GameEntity");
        softBody->BuildGameEntity(&gameEntity);

        auto* runtimeComponent = gameEntity.FindComponent<JoltSoftBodyComponent>();
        ASSERT_NE(runtimeComponent, nullptr);
        // Defaults ride across; the live preview toggle is editor-only and has no
        // runtime counterpart to carry to.
        EXPECT_EQ(runtimeComponent->GetSettings().m_shape, JoltSoftBodyShape::Cloth);
        EXPECT_EQ(runtimeComponent->GetSettings().m_resolution, 8u);
    }
} // namespace JoltPhysics
