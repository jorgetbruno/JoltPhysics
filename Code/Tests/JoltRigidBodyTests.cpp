#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <System/JoltSystem.h>
#include <Scene/JoltScene.h>
#include <Configuration/JoltSettingsRegistryManager.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>

namespace JoltPhysics
{
    class JoltRigidBodyTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "RigidBodyTestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);
        }

        void TearDown() override
        {
            m_system->RemoveScene(m_sceneHandle);
            m_system->Shutdown();
            m_system.reset();
        }

        AzPhysics::RigidBody* CreateDynamicBox(const AZ::Vector3& position)
        {
            auto colliderConfig = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto boxShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();

            AzPhysics::RigidBodyConfiguration config;
            config.m_position = position;
            config.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfig, boxShape);
            auto handle = m_scene->AddSimulatedBody(&config);
            return static_cast<AzPhysics::RigidBody*>(m_scene->GetSimulatedBodyFromHandle(handle));
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
    };

    TEST_F(JoltRigidBodyTests, SetMassUpdatesMassProperties)
    {
        auto* body = CreateDynamicBox(AZ::Vector3::CreateZero());
        ASSERT_NE(body, nullptr);

        body->SetMass(10.0f);
        EXPECT_NEAR(body->GetMass(), 10.0f, 0.01f);
        EXPECT_NEAR(body->GetInverseMass(), 0.1f, 0.001f);
    }

    TEST_F(JoltRigidBodyTests, DefaultInertiaMatchesUnitBox)
    {
        auto* body = CreateDynamicBox(AZ::Vector3::CreateZero());
        ASSERT_NE(body, nullptr);

        // Unit box, mass 1: I = m/12 * (h^2 + d^2) = 1/6 per axis.
        const AZ::Matrix3x3 inertia = body->GetInertiaLocal();
        EXPECT_NEAR(inertia(0, 0), 1.0f / 6.0f, 0.01f);
        EXPECT_NEAR(inertia(1, 1), 1.0f / 6.0f, 0.01f);
        EXPECT_NEAR(inertia(2, 2), 1.0f / 6.0f, 0.01f);

        const AZ::Matrix3x3 inverseInertia = body->GetInverseInertiaLocal();
        EXPECT_NEAR(inverseInertia(0, 0), 6.0f, 0.1f);
    }

    TEST_F(JoltRigidBodyTests, UpdateMassPropertiesAppliesOverrides)
    {
        auto* body = CreateDynamicBox(AZ::Vector3::CreateZero());
        ASSERT_NE(body, nullptr);

        body->UpdateMassProperties(
            AzPhysics::MassComputeFlags::COMPUTE_MASS | AzPhysics::MassComputeFlags::COMPUTE_INERTIA,
            AZ::Vector3::CreateZero(),
            AZ::Matrix3x3::CreateDiagonal(AZ::Vector3(1.0f, 2.0f, 3.0f)),
            5.0f);

        EXPECT_NEAR(body->GetMass(), 5.0f, 0.01f);

        const AZ::Matrix3x3 inertia = body->GetInertiaLocal();
        EXPECT_NEAR(inertia(0, 0), 1.0f, 0.01f);
        EXPECT_NEAR(inertia(1, 1), 2.0f, 0.01f);
        EXPECT_NEAR(inertia(2, 2), 3.0f, 0.01f);
    }

    TEST_F(JoltRigidBodyTests, CenterOfMassOffsetShiftsMassFrame)
    {
        auto* body = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 10.0f));
        ASSERT_NE(body, nullptr);

        body->SetCenterOfMassOffset(AZ::Vector3(0.0f, 0.0f, 1.0f));

        // Jolt-native semantics: the actor frame stays put while the collision
        // geometry (and with it the mass frame) shifts by -offset. Jolt cannot
        // express PhysX's "geometry fixed, mass frame moved" model.
        EXPECT_NEAR(body->GetCenterOfMassWorld().GetZ(), 9.0f, 0.05f);
        EXPECT_NEAR(body->GetPosition().GetZ(), 10.0f, 0.05f);

        // The 1m box now spans z in [8.5, 9.5], so a ray down from above hits z=9.5.
        AzPhysics::RayCastRequest request;
        request.m_start = AZ::Vector3(0.0f, 0.0f, 20.0f);
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 50.0f;
        AzPhysics::SceneQueryHits hits = m_scene->QueryScene(&request);
        ASSERT_EQ(hits.m_hits.size(), 1u);
        EXPECT_NEAR(hits.m_hits[0].m_position.GetZ(), 9.5f, 0.05f);
    }

    TEST_F(JoltRigidBodyTests, KinematicTargetMovesKinematicBody)
    {
        auto* body = CreateDynamicBox(AZ::Vector3::CreateZero());
        ASSERT_NE(body, nullptr);
        body->SetKinematic(true);

        body->SetKinematicTarget(AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 1.0f)));
        SimulateSeconds(1.0f / 60.0f);

        EXPECT_NEAR(body->GetPosition().GetZ(), 1.0f, 0.1f);
    }

    TEST_F(JoltRigidBodyTests, DisabledSimulationFreezesBodyUntilReenabled)
    {
        auto* body = CreateDynamicBox(AZ::Vector3(0.0f, 0.0f, 10.0f));
        ASSERT_NE(body, nullptr);

        body->SetSimulationEnabled(false);
        SimulateSeconds(0.5f);
        EXPECT_NEAR(body->GetPosition().GetZ(), 10.0f, 0.01f);

        body->SetSimulationEnabled(true);
        SimulateSeconds(0.5f);
        EXPECT_LT(body->GetPosition().GetZ(), 9.0f);
    }

    TEST_F(JoltRigidBodyTests, PerBodyRayCastHitsAndMisses)
    {
        auto* body = CreateDynamicBox(AZ::Vector3::CreateZero());
        ASSERT_NE(body, nullptr);

        AzPhysics::RayCastRequest request;
        request.m_start = AZ::Vector3(0.0f, 0.0f, 5.0f);
        request.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
        request.m_distance = 20.0f;

        AzPhysics::SceneQueryHit hit = body->RayCast(request);
        EXPECT_NEAR(hit.m_position.GetZ(), 0.5f, 0.01f);
        EXPECT_EQ(hit.m_bodyHandle, body->m_bodyHandle);

        request.m_start = AZ::Vector3(5.0f, 0.0f, 5.0f);
        AzPhysics::SceneQueryHit miss = body->RayCast(request);
        EXPECT_EQ(miss.m_bodyHandle, AzPhysics::InvalidSimulatedBodyHandle);
    }

} // namespace JoltPhysics
