#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <AzCore/Component/Entity.h>
#include <AzFramework/Components/TransformComponent.h>

#include <Configuration/JoltSettingsRegistryManager.h>
#include <Editor/Components/EditorJoltBoxColliderComponent.h>
#include <Editor/Components/EditorJoltRigidBodyComponent.h>
#include <Editor/Components/EditorJoltVehicleComponent.h>
#include <System/JoltSystem.h>

namespace JoltPhysics
{
    //! The editor vehicle's suspension settle preview: a one-shot simulation in a
    //! private scene whose results become a viewport ghost.
    class JoltEditorVehicleSettleTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));
            JoltSystemConfiguration config;
            m_system->Initialize(&config);
        }

        void TearDown() override
        {
            if (m_entity)
            {
                m_entity->Deactivate();
                m_entity.reset();
            }
            m_system->Shutdown();
            m_system.reset();
        }

        AZStd::unique_ptr<JoltSystem> m_system;
        AZStd::unique_ptr<AZ::Entity> m_entity;
    };

    TEST_F(JoltEditorVehicleSettleTests, TheSettlePreviewFindsTheRestPoseWithoutGameMode)
    {
        m_entity = AZStd::make_unique<AZ::Entity>("SettlePreviewEntity");
        m_entity->CreateComponent<AzFramework::TransformComponent>();
        m_entity->CreateComponent<EditorJoltBoxColliderComponent>();
        m_entity->CreateComponent<EditorJoltRigidBodyComponent>();
        auto* vehicle = m_entity->CreateComponent<EditorJoltVehicleComponent>();

        // Standard car: front axle steers, rear drives (the default wheel layout).
        JoltVehicleConfiguration& config = vehicle->GetVehicleConfiguration();
        struct WheelDesc { float x, y; };
        for (const auto& desc : { WheelDesc{ 0.8f, 0.45f }, WheelDesc{ 0.8f, -0.45f },
                                  WheelDesc{ -0.8f, 0.45f }, WheelDesc{ -0.8f, -0.45f } })
        {
            JoltWheelConfiguration wheel;
            wheel.m_position = AZ::Vector3(desc.x, desc.y, -0.2f);
            config.m_wheels.push_back(wheel);
        }

        m_entity->Init();
        m_entity->Activate();

        ASSERT_FALSE(vehicle->HasSettlePreview());

        // No editor world in this fixture, so the preview settles on its fallback plane
        // below full droop - which is exactly the headless path.
        vehicle->OnSettlePreviewPressed();

        ASSERT_TRUE(vehicle->HasSettlePreview());
        const auto& wheels = vehicle->GetSettlePreviewWheels();
        ASSERT_EQ(wheels.size(), 4u);
        for (const auto& wheel : wheels)
        {
            EXPECT_TRUE(wheel.m_onGround);
            // Settled somewhere inside the suspension travel, not pinned at either end.
            EXPECT_GT(wheel.m_suspensionLength, 0.0f);
            EXPECT_LE(wheel.m_suspensionLength, 0.45f + 1e-3f);
        }

        // The ghost sits below the entity (the vehicle dropped to the fallback plane).
        EXPECT_LT(wheels[0].m_localTransform.GetTranslation().GetZ(), 0.0f);

        // Re-running replaces the ghost rather than stacking previews.
        vehicle->GetVehicleConfiguration().m_wheels[0].m_radius = 0.4f;
        vehicle->OnSettlePreviewPressed();
        EXPECT_TRUE(vehicle->HasSettlePreview());
        EXPECT_EQ(vehicle->GetSettlePreviewWheels().size(), 4u);

        // The private preview scene must not leak.
        for (const auto& scene : m_system->GetAllScenes())
        {
            if (scene)
            {
                EXPECT_STRNE(scene->GetConfiguration().m_sceneName.c_str(), "VehicleSettlePreview");
            }
        }
    }
} // namespace JoltPhysics
