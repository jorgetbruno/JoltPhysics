#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <System/JoltSystem.h>
#include <Scene/JoltScene.h>
#include <Configuration/JoltSettingsRegistryManager.h>

#include <JoltPhysics/JoltModuleGlobals.h>

#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>

namespace JoltPhysics
{
    class JoltSystemTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
        }

        void TearDown() override
        {
        }
    };

    TEST_F(JoltSystemTests, DefaultConfigurationSeedsTheDefaultCollisionLayer)
    {
        // AzFramework leaves CollisionLayers as 64 blank names, so without seeding here
        // the editor's layer dropdown has nothing to list and every collider sits on an
        // unnamed layer.
        const JoltSystemConfiguration config;
        const auto& layers = config.m_collisionConfig.m_collisionLayers;

        EXPECT_STREQ(layers.GetName(AzPhysics::CollisionLayer::Default).c_str(), "Default");

        AzPhysics::CollisionLayer resolved;
        EXPECT_TRUE(layers.TryGetLayer("Default", resolved));
        EXPECT_EQ(resolved.GetIndex(), AzPhysics::CollisionLayer::Default.GetIndex());
    }

    TEST_F(JoltSystemTests, DefaultConfigurationSeedsTheAllAndNoneGroups)
    {
        const JoltSystemConfiguration config;
        const auto& groups = config.m_collisionConfig.m_collisionGroups;

        ASSERT_EQ(groups.GetPresets().size(), 2u);

        const AzPhysics::CollisionGroup all = groups.FindGroupById(JoltSystemConfiguration::AllGroupId);
        const AzPhysics::CollisionGroup none = groups.FindGroupById(JoltSystemConfiguration::NoneGroupId);

        // The names are what the dropdown shows; the masks are what they have to mean.
        EXPECT_STREQ(groups.FindGroupNameById(JoltSystemConfiguration::AllGroupId).c_str(), "All");
        EXPECT_STREQ(groups.FindGroupNameById(JoltSystemConfiguration::NoneGroupId).c_str(), "None");
        EXPECT_TRUE(all.IsSet(AzPhysics::CollisionLayer::Default));
        EXPECT_FALSE(none.IsSet(AzPhysics::CollisionLayer::Default));

        for (const auto& preset : groups.GetPresets())
        {
            EXPECT_TRUE(preset.m_readOnly) << "Default groups must not be deletable: " << preset.m_name.c_str();
        }
    }

    TEST_F(JoltSystemTests, DefaultGroupIdsAreStableAcrossConfigurations)
    {
        // A collider serializes the group *id* it was authored with. Generating a fresh
        // id per construction would orphan every collider referencing it the next run.
        const JoltSystemConfiguration first;
        const JoltSystemConfiguration second;

        const auto& firstPresets = first.m_collisionConfig.m_collisionGroups.GetPresets();
        const auto& secondPresets = second.m_collisionConfig.m_collisionGroups.GetPresets();
        ASSERT_EQ(firstPresets.size(), secondPresets.size());

        for (size_t i = 0; i < firstPresets.size(); ++i)
        {
            EXPECT_EQ(firstPresets[i].m_id, secondPresets[i].m_id);
        }
    }

    TEST_F(JoltSystemTests, SimulateBoundsItsCatchUpToMaxTimestep)
    {
        // Without a bound, a hitch feeds itself: a long stall accumulates one queued step
        // per fixed timestep it lasted, stepping that backlog takes longer than the stall,
        // and the next frame's backlog is larger still. m_maxTimestep caps how much time a
        // single Simulate call may work through.
        auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
        auto system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

        JoltSystemConfiguration config;
        config.m_fixedTimestep = 0.1f;
        config.m_maxTimestep = 0.5f; // at most five steps of simulated time per call
        system->Initialize(&config);

        AzPhysics::SceneConfiguration sceneConfig;
        sceneConfig.m_sceneName = "TimestepScene";
        const AzPhysics::SceneHandle sceneHandle = system->AddScene(sceneConfig);
        AzPhysics::Scene* scene = system->GetScene(sceneHandle);

        // A body in free fall counts the steps for us: under gravity its speed after N
        // steps is g * N * dt exactly, so the velocity says how much time was simulated.
        AzPhysics::RigidBodyConfiguration bodyConfig;
        bodyConfig.m_position = AZ::Vector3(0.0f, 0.0f, 1000.0f);
        bodyConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(
            AZStd::make_shared<Physics::ColliderConfiguration>(),
            AZStd::make_shared<Physics::BoxShapeConfiguration>());
        auto* body = azdynamic_cast<AzPhysics::RigidBody*>(
            scene->GetSimulatedBodyFromHandle(scene->AddSimulatedBody(&bodyConfig)));
        ASSERT_NE(body, nullptr);

        // Ten seconds of backlog - a debugger pause, or a long asset build - is a hundred
        // queued steps unbounded, and the body would be doing 98 m/s.
        system->Simulate(10.0f);

        // Half a second of falling, a little under g*0.5 because Jolt damps by default.
        // Bounds rather than an exact figure: the claim is that roughly five steps ran,
        // not a hundred, and damping makes the exact value a function of step count.
        const float speedAfterHitch = -body->GetLinearVelocity().GetZ();
        EXPECT_GT(speedAfterHitch, 4.0f) << "the hitch simulated less than the bound allows";
        EXPECT_LT(speedAfterHitch, 5.5f) << "the catch-up was not bounded";

        // And the backlog does not survive to be paid off later: the next ordinary frame
        // advances one step, not the ninety-five that were dropped.
        system->Simulate(0.1f);
        const float gainedInOneFrame = -body->GetLinearVelocity().GetZ() - speedAfterHitch;
        EXPECT_GT(gainedInOneFrame, 0.5f);
        EXPECT_LT(gainedInOneFrame, 1.5f) << "the dropped backlog was paid off on a later frame";

        system->RemoveScene(sceneHandle);
        system->Shutdown();
    }

    TEST_F(JoltSystemTests, TheExportedModuleGlobalsInstallerLeavesJoltUsable)
    {
        // Extension gems (JoltBuoyancy, JoltHair) each own a private copy of Jolt's
        // globals and each used to hand-copy this setup, learning the hard way that the
        // allocation hooks and the factory both start null. The header they now share is
        // exported through JoltPhysics.API, so it is worth pinning that calling it is
        // enough on its own and that a second call changes nothing.
        JPH::Factory* const factoryAtStart = JPH::Factory::sInstance;

        InstallJoltModuleGlobals();
        EXPECT_NE(JPH::Allocate, nullptr);
        EXPECT_NE(JPH::Reallocate, nullptr);
        EXPECT_NE(JPH::Free, nullptr);
        EXPECT_NE(JPH::AlignedAllocate, nullptr);
        EXPECT_NE(JPH::AlignedFree, nullptr);
        ASSERT_NE(JPH::Factory::sInstance, nullptr);

        // Idempotent: it keeps the factory that already exists rather than stranding the
        // types registered in it, and it does not re-register those types either - Jolt's
        // RegisterTypes has no re-entry guard and would just grow the factory's tables.
        JPH::Factory* const factoryAfterInstall = JPH::Factory::sInstance;
        EXPECT_FALSE(InstallJoltFactory()) << "a second call claimed it created the factory";
        InstallJoltModuleGlobals();
        EXPECT_EQ(JPH::Factory::sInstance, factoryAfterInstall);

        // A round trip through the hooks, which is what a null one would fault on.
        void* block = JPH::Allocate(64);
        ASSERT_NE(block, nullptr);
        JPH::Free(block);

        // Leave the process as it was found. When this test created the factory, that
        // also exercises the teardown half - which every module owes from its destructor,
        // while the AZ allocators are still alive.
        if (factoryAtStart == nullptr)
        {
            UninstallJoltModuleGlobals();
            EXPECT_EQ(JPH::Factory::sInstance, nullptr);
        }
    }

    TEST_F(JoltSystemTests, SystemCanBeCreated)
    {
        auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
        JoltSystem system(AZStd::move(registryManager));

        EXPECT_EQ(system.GetConfiguration(), nullptr);
    }

    TEST_F(JoltSystemTests, SystemCanBeInitialized)
    {
        auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
        JoltSystem system(AZStd::move(registryManager));

        JoltSystemConfiguration config;
        system.Initialize(&config);

        EXPECT_NE(system.GetConfiguration(), nullptr);

        system.Shutdown();
    }

    TEST_F(JoltSystemTests, SceneCanBeAdded)
    {
        auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
        JoltSystem system(AZStd::move(registryManager));

        JoltSystemConfiguration config;
        system.Initialize(&config);

        AzPhysics::SceneConfiguration sceneConfig;
        sceneConfig.m_sceneName = "TestScene";

        AzPhysics::SceneHandle handle = system.AddScene(sceneConfig);

        EXPECT_NE(handle, AzPhysics::InvalidSceneHandle);
        EXPECT_NE(system.GetScene(handle), nullptr);

        system.RemoveScene(handle);
        system.Shutdown();
    }

} // namespace JoltPhysics
