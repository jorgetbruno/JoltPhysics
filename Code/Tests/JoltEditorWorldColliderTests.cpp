#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <AzCore/Component/Entity.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Components/NonUniformScaleComponent.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <AzFramework/Physics/PhysicsScene.h>

#include <Configuration/JoltSettingsRegistryManager.h>
#include <Editor/Components/EditorJoltBoxColliderComponent.h>
#include <Editor/JoltPhysicsEditorSystemComponent.h>
#include <System/JoltSystem.h>

//! Provides the runtime component's service so the editor system component can
//! activate in the fixture (same stand-in as JoltPhysicsEditorSystemComponentTests;
//! global namespace because AZ_COMPONENT's RTTI injections belong to ::AZ).
class JoltEditorWorldColliderServiceStub : public AZ::Component
{
public:
    AZ_COMPONENT(JoltEditorWorldColliderServiceStub, "{0D9E8F7A-6B5C-4D3E-A2F1-9C8B7A6D5E4F}");
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
    //! Editor collider components create static bodies in the editor scene
    //! (EditorWorldBus) so editor-time physics queries hit what the viewport shows -
    //! PhysX's CreateStaticEditorCollider equivalent.
    class JoltEditorWorldColliderTests : public ::testing::Test
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
                JoltEditorWorldColliderServiceStub::CreateDescriptor());

            // The editor system component hosts the editor scene the colliders add to.
            m_systemEntity = AZStd::make_unique<AZ::Entity>("EditorSystem");
            m_systemEntity->CreateComponent<JoltEditorWorldColliderServiceStub>();
            m_editorSystemComponent = m_systemEntity->CreateComponent<JoltPhysicsEditorSystemComponent>();
            m_systemEntity->Init();
            m_systemEntity->Activate();
        }

        void TearDown() override
        {
            if (m_colliderEntity)
            {
                if (m_colliderEntity->GetState() == AZ::Entity::State::Active)
                {
                    m_colliderEntity->Deactivate();
                }
                m_colliderEntity.reset();
            }
            m_systemEntity->Deactivate();
            m_systemEntity.reset();
            AZ::ComponentApplicationBus::Broadcast(
                &AZ::ComponentApplicationRequests::UnregisterComponentDescriptor,
                JoltEditorWorldColliderServiceStub::CreateDescriptor());
            m_system->Shutdown();
            m_system.reset();
        }

        //! An active editor entity holding an editor box collider (2 m cube by default).
        EditorJoltBoxColliderComponent* MakeBoxColliderEntity(
            const AZ::Vector3& position, const AZ::Vector3* nonUniformScale = nullptr)
        {
            m_colliderEntity = AZStd::make_unique<AZ::Entity>("EditorWorldColliderEntity");
            auto* transform = m_colliderEntity->CreateComponent<AzFramework::TransformComponent>();
            if (nonUniformScale)
            {
                auto* scaleComponent = m_colliderEntity->CreateComponent<AzFramework::NonUniformScaleComponent>();
                scaleComponent->SetScale(*nonUniformScale);
            }
            auto* collider = m_colliderEntity->CreateComponent<EditorJoltBoxColliderComponent>();
            m_colliderEntity->Init();
            m_colliderEntity->Activate();
            transform->SetWorldTM(AZ::Transform::CreateTranslation(position));
            return collider;
        }

        AzPhysics::Scene* GetEditorScene()
        {
            return m_system->GetScene(m_editorSystemComponent->GetEditorSceneHandle());
        }

        //! Ray straight down onto the given x/y, from above.
        AzPhysics::SceneQueryHits RayDownAt(const AZ::Vector3& position)
        {
            AzPhysics::RayCastRequest request;
            request.m_start = position + AZ::Vector3(0.0f, 0.0f, 50.0f);
            request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
            request.m_distance = 100.0f;
            return GetEditorScene()->QueryScene(&request);
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AZStd::unique_ptr<AZ::Entity> m_systemEntity;
        AZStd::unique_ptr<AZ::Entity> m_colliderEntity;
        JoltPhysicsEditorSystemComponent* m_editorSystemComponent = nullptr;
    };

    TEST_F(JoltEditorWorldColliderTests, AnEditorColliderIsHitByEditorSceneQueries)
    {
        MakeBoxColliderEntity(AZ::Vector3(10.0f, 0.0f, 0.0f));

        AzPhysics::SceneQueryHits hits = RayDownAt(AZ::Vector3(10.0f, 0.0f, 0.0f));
        ASSERT_EQ(hits.m_hits.size(), 1u);
        EXPECT_EQ(hits.m_hits[0].m_entityId, m_colliderEntity->GetId());
    }

    TEST_F(JoltEditorWorldColliderTests, MovingTheEntityMovesItsEditorBody)
    {
        MakeBoxColliderEntity(AZ::Vector3(10.0f, 0.0f, 0.0f));

        AZ::TransformBus::Event(
            m_colliderEntity->GetId(), &AZ::TransformBus::Events::SetWorldTM,
            AZ::Transform::CreateTranslation(AZ::Vector3(-10.0f, 0.0f, 0.0f)));

        EXPECT_EQ(RayDownAt(AZ::Vector3(10.0f, 0.0f, 0.0f)).m_hits.size(), 0u);
        AzPhysics::SceneQueryHits hits = RayDownAt(AZ::Vector3(-10.0f, 0.0f, 0.0f));
        ASSERT_EQ(hits.m_hits.size(), 1u);
        EXPECT_EQ(hits.m_hits[0].m_entityId, m_colliderEntity->GetId());
    }

    TEST_F(JoltEditorWorldColliderTests, TheEditorBodyCarriesTheEntityScale)
    {
        // A 1 m default box scaled (4, 1, 1): a ray 1.5 m off-centre on x hits only
        // because the scale reached the editor body's shape.
        const AZ::Vector3 scale(4.0f, 1.0f, 1.0f);
        MakeBoxColliderEntity(AZ::Vector3::CreateZero(), &scale);

        EXPECT_EQ(RayDownAt(AZ::Vector3(1.5f, 0.0f, 0.0f)).m_hits.size(), 1u);
        // Off the scaled box on y, where the same offset must miss.
        EXPECT_EQ(RayDownAt(AZ::Vector3(0.0f, 1.5f, 0.0f)).m_hits.size(), 0u);
    }

    TEST_F(JoltEditorWorldColliderTests, DeactivatingTheEntityRemovesItsEditorBody)
    {
        MakeBoxColliderEntity(AZ::Vector3::CreateZero());
        ASSERT_EQ(RayDownAt(AZ::Vector3::CreateZero()).m_hits.size(), 1u);

        m_colliderEntity->Deactivate();
        EXPECT_EQ(RayDownAt(AZ::Vector3::CreateZero()).m_hits.size(), 0u);
    }

    TEST_F(JoltEditorWorldColliderTests, WithoutAnEditorWorldTheColliderStillActivates)
    {
        // A launcher or a stripped-down tool has no editor system component; the
        // collider then simply has no editor body, and nothing else changes.
        m_systemEntity->Deactivate();
        MakeBoxColliderEntity(AZ::Vector3::CreateZero());
        EXPECT_EQ(m_colliderEntity->GetState(), AZ::Entity::State::Active);
        m_systemEntity->Activate();
    }
} // namespace JoltPhysics
