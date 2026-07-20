#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <Clients/Components/JoltCapsuleColliderComponent.h>
#include <Clients/Components/JoltRigidBodyComponent.h>
#include <System/JoltSystem.h>
#include <Scene/JoltScene.h>
#include <Configuration/JoltSettingsRegistryManager.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Physics/SystemBus.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    // Exercises the full component path the editor uses: entity with collider +
    // rigid body components creating a simulated body in the default world.
    class JoltComponentBodyCreationTests : public ::testing::Test
    {
    protected:
        class TestDefaultWorldHandler : public Physics::DefaultWorldBus::Handler
        {
        public:
            explicit TestDefaultWorldHandler(AzPhysics::SceneHandle sceneHandle)
                : m_sceneHandle(sceneHandle)
            {
                Physics::DefaultWorldBus::Handler::BusConnect();
            }
            ~TestDefaultWorldHandler() override
            {
                Physics::DefaultWorldBus::Handler::BusDisconnect();
            }

            AzPhysics::SceneHandle GetDefaultSceneHandle() const override
            {
                return m_sceneHandle;
            }

        private:
            AzPhysics::SceneHandle m_sceneHandle;
        };

        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "ComponentTestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);

            m_defaultWorldHandler = AZStd::make_unique<TestDefaultWorldHandler>(m_sceneHandle);
        }

        void TearDown() override
        {
            m_defaultWorldHandler.reset();
            m_system->RemoveScene(m_sceneHandle);
            m_system->Shutdown();
            m_system.reset();
        }

        void SimulateSeconds(float seconds)
        {
            const float fixedDeltaTime = 1.0f / 60.0f;
            const int steps = static_cast<int>(seconds / fixedDeltaTime);
            for (int i = 0; i < steps; ++i)
            {
                m_scene->StartSimulation(fixedDeltaTime);
                m_scene->FinishSimulation();
            }
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AzPhysics::SceneHandle m_sceneHandle;
        AzPhysics::Scene* m_scene = nullptr;
        AZStd::unique_ptr<TestDefaultWorldHandler> m_defaultWorldHandler;
    };

    TEST_F(JoltComponentBodyCreationTests, CapsuleComponentProducesZUpCapsule)
    {
        auto entity = AZStd::make_unique<AZ::Entity>("CapsuleEntity");
        entity->CreateComponent<AzFramework::TransformComponent>();
        entity->CreateComponent<JoltCapsuleColliderComponent>();
        entity->CreateComponent<JoltRigidBodyComponent>();
        entity->Init();
        entity->Activate();
        AZ::TransformBus::Event(entity->GetId(), &AZ::TransformBus::Events::SetWorldTranslation, AZ::Vector3(0.0f, 0.0f, 4.0f));

        // Static slab with top at z=0 for the capsule to land on.
        auto slabCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
        auto slabShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
        slabShape->m_dimensions = AZ::Vector3(20.0f, 20.0f, 1.0f);
        AzPhysics::StaticRigidBodyConfiguration slabConfig;
        slabConfig.m_position = AZ::Vector3(0.0f, 0.0f, -0.5f);
        slabConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(slabCollider, slabShape);
        m_scene->AddSimulatedBody(&slabConfig);

        SimulateSeconds(4.0f);

        // Read the simulated body (the component's transform sync back to the
        // entity runs on the tick bus, which this test does not pump).
        const auto [foundSceneHandle, bodyHandle] =
            m_system->FindAttachedBodyHandleFromEntityId(entity->GetId());
        ASSERT_NE(bodyHandle, AzPhysics::InvalidSimulatedBodyHandle);
        EXPECT_EQ(foundSceneHandle, m_sceneHandle);

        const float capsuleZ = m_scene->GetSimulatedBodyFromHandle(bodyHandle)->GetPosition().GetZ();
        EXPECT_NEAR(capsuleZ, 0.5f, 0.05f);
    }

} // namespace JoltPhysics
