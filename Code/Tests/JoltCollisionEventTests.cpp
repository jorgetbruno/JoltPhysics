#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <System/JoltSystem.h>
#include <Scene/JoltScene.h>
#include <RigidBody/JoltRigidBody.h>
#include <Configuration/JoltSettingsRegistryManager.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    // Covers the collision-event pipeline: OnContactAdded/Persisted/Removed on the Jolt
    // contact listener -> QueueCollisionEvent -> ProcessCollisionEvents ->
    // SimulatedBody::ProcessCollisionEvent -> OnCollisionBegin/Persist/End handlers.
    class JoltCollisionEventTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
            m_system = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

            JoltSystemConfiguration config;
            m_system->Initialize(&config);

            AzPhysics::SceneConfiguration sceneConfig;
            sceneConfig.m_sceneName = "CollisionEventTestScene";
            m_sceneHandle = m_system->AddScene(sceneConfig);
            m_scene = m_system->GetScene(m_sceneHandle);
        }

        void TearDown() override
        {
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

        AzPhysics::RigidBody* GetBody(AzPhysics::SimulatedBodyHandle handle)
        {
            return azdynamic_cast<AzPhysics::RigidBody*>(m_scene->GetSimulatedBodyFromHandle(handle));
        }

        // Static slab spanning z in [-1, 1] (top surface at z = 1).
        AzPhysics::SimulatedBodyHandle AddStaticSlab(bool isTrigger = false)
        {
            auto collider = AZStd::make_shared<Physics::ColliderConfiguration>();
            collider->m_isTrigger = isTrigger;
            auto shape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            shape->m_dimensions = AZ::Vector3(10.0f, 10.0f, 2.0f);

            AzPhysics::StaticRigidBodyConfiguration config;
            config.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(collider, shape);
            return m_scene->AddSimulatedBody(&config);
        }

        // Dynamic unit box at the given height (and optional x offset, to drop several
        // boxes onto the slab without them landing on each other).
        AzPhysics::SimulatedBodyHandle AddDynamicBox(float z, float x = 0.0f)
        {
            auto collider = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto shape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            AzPhysics::RigidBodyConfiguration config;
            config.m_position = AZ::Vector3(x, 0.0f, z);
            config.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(collider, shape);
            return m_scene->AddSimulatedBody(&config);
        }

        AzPhysics::SimulatedBodyHandle m_sceneHandle;
        AzPhysics::Scene* m_scene = nullptr;

    private:
        AZStd::unique_ptr<JoltSystem> m_system;
    };

    TEST_F(JoltCollisionEventTests, FallingBodyFiresCollisionBeginAndPersistOnStaticSlab)
    {
        auto slabHandle = AddStaticSlab();
        ASSERT_NE(slabHandle, AzPhysics::InvalidSimulatedBodyHandle);
        auto boxHandle = AddDynamicBox(3.0f);
        ASSERT_NE(boxHandle, AzPhysics::InvalidSimulatedBodyHandle);

        auto* slabBody = m_scene->GetSimulatedBodyFromHandle(slabHandle);
        ASSERT_NE(slabBody, nullptr);

        int beginCount = 0;
        int persistCount = 0;
        AzPhysics::SimulatedBodyHandle beginOther = AzPhysics::InvalidSimulatedBodyHandle;
        bool normalIsVertical = false;
        bool hasContacts = false;

        AzPhysics::SimulatedBodyEvents::OnCollisionBegin::Handler beginHandler(
            [&](AzPhysics::SimulatedBodyHandle self, const AzPhysics::CollisionEvent& event)
            {
                ++beginCount;
                beginOther = (event.m_bodyHandle1 == self) ? event.m_bodyHandle2 : event.m_bodyHandle1;
                hasContacts = !event.m_contacts.empty();
                if (hasContacts)
                {
                    // Box resting on a horizontal slab: the contact normal is vertical.
                    // Sign depends on Jolt's internal body ordering, so check magnitude.
                    normalIsVertical = AZ::GetAbs(event.m_contacts[0].m_normal.GetZ()) > 0.9f;
                }
            });
        AzPhysics::SimulatedBodyEvents::OnCollisionPersist::Handler persistHandler(
            [&](AzPhysics::SimulatedBodyHandle, const AzPhysics::CollisionEvent&) { ++persistCount; });

        slabBody->RegisterOnCollisionBeginHandler(beginHandler);
        slabBody->RegisterOnCollisionPersistHandler(persistHandler);

        SimulateSeconds(1.5f);

        EXPECT_GE(beginCount, 1);
        EXPECT_EQ(beginOther, boxHandle);
        EXPECT_TRUE(hasContacts);
        EXPECT_TRUE(normalIsVertical);
        EXPECT_GT(persistCount, 0); // box keeps resting on the slab -> contact persists
    }

    TEST_F(JoltCollisionEventTests, BothBodiesReceiveCollisionBegin)
    {
        // Verifies the perspective-swap dispatch: each body is notified and sees the
        // other body as the "other" party, with itself as body1 in its own event.
        auto slabHandle = AddStaticSlab();
        auto boxHandle = AddDynamicBox(3.0f);

        auto* slabBody = m_scene->GetSimulatedBodyFromHandle(slabHandle);
        auto* boxBody = m_scene->GetSimulatedBodyFromHandle(boxHandle);
        ASSERT_NE(slabBody, nullptr);
        ASSERT_NE(boxBody, nullptr);

        int slabBegin = 0;
        int boxBegin = 0;
        AzPhysics::SimulatedBodyHandle slabSelf = AzPhysics::InvalidSimulatedBodyHandle;
        AzPhysics::SimulatedBodyHandle slabOther = AzPhysics::InvalidSimulatedBodyHandle;
        AzPhysics::SimulatedBodyHandle boxSelf = AzPhysics::InvalidSimulatedBodyHandle;
        AzPhysics::SimulatedBodyHandle boxOther = AzPhysics::InvalidSimulatedBodyHandle;

        AzPhysics::SimulatedBodyEvents::OnCollisionBegin::Handler slabHandler(
            [&](AzPhysics::SimulatedBodyHandle self, const AzPhysics::CollisionEvent& event)
            {
                ++slabBegin;
                slabSelf = event.m_bodyHandle1; // receiver is always body1 in its own event
                slabOther = (event.m_bodyHandle1 == self) ? event.m_bodyHandle2 : event.m_bodyHandle1;
            });
        AzPhysics::SimulatedBodyEvents::OnCollisionBegin::Handler boxHandler(
            [&](AzPhysics::SimulatedBodyHandle self, const AzPhysics::CollisionEvent& event)
            {
                ++boxBegin;
                boxSelf = event.m_bodyHandle1;
                boxOther = (event.m_bodyHandle1 == self) ? event.m_bodyHandle2 : event.m_bodyHandle1;
            });

        slabBody->RegisterOnCollisionBeginHandler(slabHandler);
        boxBody->RegisterOnCollisionBeginHandler(boxHandler);

        SimulateSeconds(1.5f);

        EXPECT_GE(slabBegin, 1);
        EXPECT_GE(boxBegin, 1);
        EXPECT_EQ(slabSelf, slabHandle);
        EXPECT_EQ(slabOther, boxHandle);
        EXPECT_EQ(boxSelf, boxHandle);
        EXPECT_EQ(boxOther, slabHandle);
    }

    TEST_F(JoltCollisionEventTests, SeparatingBodyFiresCollisionEnd)
    {
        auto slabHandle = AddStaticSlab();
        auto boxHandle = AddDynamicBox(1.5f); // starts resting on the slab (in contact)

        auto* slabBody = m_scene->GetSimulatedBodyFromHandle(slabHandle);
        ASSERT_NE(slabBody, nullptr);

        int beginCount = 0;
        int endCount = 0;
        AzPhysics::SimulatedBodyEvents::OnCollisionBegin::Handler beginHandler(
            [&](AzPhysics::SimulatedBodyHandle, const AzPhysics::CollisionEvent&) { ++beginCount; });
        AzPhysics::SimulatedBodyEvents::OnCollisionEnd::Handler endHandler(
            [&](AzPhysics::SimulatedBodyHandle, const AzPhysics::CollisionEvent&) { ++endCount; });
        slabBody->RegisterOnCollisionBeginHandler(beginHandler);
        slabBody->RegisterOnCollisionEndHandler(endHandler);

        // Contact forms as the box settles.
        SimulateSeconds(0.3f);
        ASSERT_GE(beginCount, 1);
        EXPECT_EQ(endCount, 0);

        // Fling the box up and away; it leaves the slab -> contact removed -> End.
        auto* box = GetBody(boxHandle);
        ASSERT_NE(box, nullptr);
        box->SetLinearVelocity(AZ::Vector3(0.0f, 0.0f, 10.0f));

        SimulateSeconds(0.5f);
        EXPECT_GE(endCount, 1);
        EXPECT_GT(box->GetPosition().GetZ(), 2.0f); // it actually left the slab
    }

    TEST_F(JoltCollisionEventTests, RemovingBodyMidContactFiresCollisionEndOnPartner)
    {
        // When a body is removed while touching another, the surviving partner must still
        // receive OnCollisionEnd -- Jolt's own OnContactRemoved for the pair only arrives a
        // step later, after the removed body's id->handle mapping is gone.
        auto slabHandle = AddStaticSlab();
        auto boxHandle = AddDynamicBox(1.5f); // starts resting on the slab (in contact)
        const AzPhysics::SimulatedBodyHandle boxHandleValue = boxHandle;

        auto* slabBody = m_scene->GetSimulatedBodyFromHandle(slabHandle);
        ASSERT_NE(slabBody, nullptr);

        int beginCount = 0;
        int endCount = 0;
        AzPhysics::SimulatedBodyHandle endOther = AzPhysics::InvalidSimulatedBodyHandle;
        AzPhysics::SimulatedBodyEvents::OnCollisionBegin::Handler beginHandler(
            [&](AzPhysics::SimulatedBodyHandle, const AzPhysics::CollisionEvent&) { ++beginCount; });
        AzPhysics::SimulatedBodyEvents::OnCollisionEnd::Handler endHandler(
            [&](AzPhysics::SimulatedBodyHandle self, const AzPhysics::CollisionEvent& event)
            {
                ++endCount;
                endOther = (event.m_bodyHandle1 == self) ? event.m_bodyHandle2 : event.m_bodyHandle1;
            });
        slabBody->RegisterOnCollisionBeginHandler(beginHandler);
        slabBody->RegisterOnCollisionEndHandler(endHandler);

        SimulateSeconds(0.3f);
        ASSERT_GE(beginCount, 1);
        EXPECT_EQ(endCount, 0);

        // Remove the box while it is resting on the slab.
        m_scene->RemoveSimulatedBody(boxHandle);
        SimulateSeconds(0.1f); // one flush is enough

        EXPECT_GE(endCount, 1);
        EXPECT_EQ(endOther, boxHandleValue);
    }

    TEST_F(JoltCollisionEventTests, TriggerVolumeDoesNotFireCollisionEvents)
    {
        // Overlaps that involve a trigger/sensor are reported through trigger events,
        // never collision events. A box falling through a sensor slab must fire zero
        // collision-begin events on either body.
        auto sensorHandle = AddStaticSlab(/*isTrigger*/ true);
        auto boxHandle = AddDynamicBox(3.0f);

        auto* sensorBody = m_scene->GetSimulatedBodyFromHandle(sensorHandle);
        auto* boxBody = m_scene->GetSimulatedBodyFromHandle(boxHandle);
        ASSERT_NE(sensorBody, nullptr);
        ASSERT_NE(boxBody, nullptr);

        int sensorCollisionBegin = 0;
        int boxCollisionBegin = 0;
        AzPhysics::SimulatedBodyEvents::OnCollisionBegin::Handler sensorHandler(
            [&](AzPhysics::SimulatedBodyHandle, const AzPhysics::CollisionEvent&) { ++sensorCollisionBegin; });
        AzPhysics::SimulatedBodyEvents::OnCollisionBegin::Handler boxHandler(
            [&](AzPhysics::SimulatedBodyHandle, const AzPhysics::CollisionEvent&) { ++boxCollisionBegin; });
        sensorBody->RegisterOnCollisionBeginHandler(sensorHandler);
        boxBody->RegisterOnCollisionBeginHandler(boxHandler);

        SimulateSeconds(2.0f); // box falls all the way through the sensor

        EXPECT_EQ(sensorCollisionBegin, 0);
        EXPECT_EQ(boxCollisionBegin, 0);
        EXPECT_LT(boxBody->GetPosition().GetZ(), 1.0f); // passed through, did not rest on it
    }

    TEST_F(JoltCollisionEventTests, SuppressedPairFiresNoCollisionEventsButStillCollides)
    {
        auto slabHandle = AddStaticSlab();
        auto boxHandle = AddDynamicBox(3.0f);

        auto* slabBody = m_scene->GetSimulatedBodyFromHandle(slabHandle);
        auto* boxBody = m_scene->GetSimulatedBodyFromHandle(boxHandle);
        ASSERT_NE(slabBody, nullptr);
        ASSERT_NE(boxBody, nullptr);

        int slabEvents = 0;
        int boxEvents = 0;
        AzPhysics::SimulatedBodyEvents::OnCollisionBegin::Handler slabBegin(
            [&](AzPhysics::SimulatedBodyHandle, const AzPhysics::CollisionEvent&) { ++slabEvents; });
        AzPhysics::SimulatedBodyEvents::OnCollisionPersist::Handler slabPersist(
            [&](AzPhysics::SimulatedBodyHandle, const AzPhysics::CollisionEvent&) { ++slabEvents; });
        AzPhysics::SimulatedBodyEvents::OnCollisionBegin::Handler boxBegin(
            [&](AzPhysics::SimulatedBodyHandle, const AzPhysics::CollisionEvent&) { ++boxEvents; });
        slabBody->RegisterOnCollisionBeginHandler(slabBegin);
        slabBody->RegisterOnCollisionPersistHandler(slabPersist);
        boxBody->RegisterOnCollisionBeginHandler(boxBegin);

        // Suppression is symmetric: registering (slab, box) also covers (box, slab).
        m_scene->SuppressCollisionEvents(slabHandle, boxHandle);

        SimulateSeconds(1.5f);

        EXPECT_EQ(slabEvents, 0);
        EXPECT_EQ(boxEvents, 0);
        // Only the events are suppressed - the box still lands on the slab (top at z=1,
        // unit box half-extent 0.5).
        EXPECT_NEAR(boxBody->GetPosition().GetZ(), 1.5f, 0.1f);
    }

    TEST_F(JoltCollisionEventTests, UnsuppressRestoresCollisionEvents)
    {
        auto slabHandle = AddStaticSlab();
        auto boxHandle = AddDynamicBox(1.5f); // starts resting on the slab

        auto* slabBody = m_scene->GetSimulatedBodyFromHandle(slabHandle);
        ASSERT_NE(slabBody, nullptr);

        int slabEvents = 0;
        AzPhysics::SimulatedBodyEvents::OnCollisionBegin::Handler slabBegin(
            [&](AzPhysics::SimulatedBodyHandle, const AzPhysics::CollisionEvent&) { ++slabEvents; });
        AzPhysics::SimulatedBodyEvents::OnCollisionPersist::Handler slabPersist(
            [&](AzPhysics::SimulatedBodyHandle, const AzPhysics::CollisionEvent&) { ++slabEvents; });
        slabBody->RegisterOnCollisionBeginHandler(slabBegin);
        slabBody->RegisterOnCollisionPersistHandler(slabPersist);

        // Reversed argument order also matches the pair.
        m_scene->SuppressCollisionEvents(boxHandle, slabHandle);
        SimulateSeconds(0.5f);
        ASSERT_EQ(slabEvents, 0);

        m_scene->UnsuppressCollisionEvents(slabHandle, boxHandle);
        SimulateSeconds(0.5f);
        EXPECT_GT(slabEvents, 0); // the resting contact now reports Persist again
    }

    TEST_F(JoltCollisionEventTests, SuppressionDoesNotAffectOtherPairs)
    {
        auto slabHandle = AddStaticSlab();
        auto suppressedBox = AddDynamicBox(3.0f, -2.0f);
        auto otherBox = AddDynamicBox(3.0f, 2.0f);

        auto* slabBody = m_scene->GetSimulatedBodyFromHandle(slabHandle);
        ASSERT_NE(slabBody, nullptr);

        int suppressedBoxEvents = 0;
        int otherBoxEvents = 0;
        AzPhysics::SimulatedBodyEvents::OnCollisionBegin::Handler slabBegin(
            [&](AzPhysics::SimulatedBodyHandle self, const AzPhysics::CollisionEvent& event)
            {
                const AzPhysics::SimulatedBodyHandle other =
                    (event.m_bodyHandle1 == self) ? event.m_bodyHandle2 : event.m_bodyHandle1;
                if (other == suppressedBox)
                {
                    ++suppressedBoxEvents;
                }
                else if (other == otherBox)
                {
                    ++otherBoxEvents;
                }
            });
        slabBody->RegisterOnCollisionBeginHandler(slabBegin);

        m_scene->SuppressCollisionEvents(slabHandle, suppressedBox);

        SimulateSeconds(2.0f);

        EXPECT_EQ(suppressedBoxEvents, 0);
        EXPECT_GE(otherBoxEvents, 1); // the un-suppressed pair still reports normally
    }

} // namespace JoltPhysics
