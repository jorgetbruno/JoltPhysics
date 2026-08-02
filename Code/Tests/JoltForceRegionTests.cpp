#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <ForceRegion/JoltForceRegionForces.h>

namespace JoltPhysics
{
    // Force regions are the PhysX component a migrating project cannot otherwise bring
    // across: fans, conveyors, currents and wind zones all rest on it. The maths of each
    // force is what decides whether authored content behaves the same, so it is pinned
    // here rather than only exercised through a level.
    class JoltForceRegionTests : public ::testing::Test
    {
    protected:
        static JoltForceRegionEntityParams MakeEntity(
            const AZ::Vector3& position = AZ::Vector3::CreateZero(),
            const AZ::Vector3& velocity = AZ::Vector3::CreateZero())
        {
            JoltForceRegionEntityParams entity;
            entity.m_position = position;
            entity.m_velocity = velocity;
            entity.m_mass = 2.0f;
            entity.m_aabb = AZ::Aabb::CreateCenterRadius(position, 0.5f);
            return entity;
        }

        static JoltForceRegionParams MakeRegion(
            const AZ::Vector3& position = AZ::Vector3::CreateZero(),
            const AZ::Quaternion& rotation = AZ::Quaternion::CreateIdentity())
        {
            JoltForceRegionParams region;
            region.m_position = position;
            region.m_rotation = rotation;
            return region;
        }
    };

    TEST_F(JoltForceRegionTests, WorldSpaceForcePushesAlongItsOwnAxisWhateverTheRegionDoes)
    {
        JoltForceWorldSpace force;
        force.m_direction = AZ::Vector3(0.0f, 0.0f, 1.0f);
        force.m_magnitude = 25.0f;

        // A rotated region must not turn a world-space force - that is the whole
        // distinction from the local-space one.
        const AZ::Quaternion rotated = AZ::Quaternion::CreateRotationX(AZ::Constants::HalfPi);
        const AZ::Vector3 result = force.CalculateForce(MakeEntity(), MakeRegion(AZ::Vector3::CreateZero(), rotated));

        EXPECT_TRUE(result.IsClose(AZ::Vector3(0.0f, 0.0f, 25.0f), 0.01f));
    }

    TEST_F(JoltForceRegionTests, LocalSpaceForceTurnsWithTheRegion)
    {
        JoltForceLocalSpace force;
        force.m_direction = AZ::Vector3(0.0f, 0.0f, 1.0f);
        force.m_magnitude = 10.0f;

        // Rotating the region a quarter turn about X takes its local +Z to world -Y.
        const AZ::Quaternion rotated = AZ::Quaternion::CreateRotationX(AZ::Constants::HalfPi);
        const AZ::Vector3 result = force.CalculateForce(MakeEntity(), MakeRegion(AZ::Vector3::CreateZero(), rotated));

        EXPECT_TRUE(result.IsClose(AZ::Vector3(0.0f, -10.0f, 0.0f), 0.01f)) << result.GetY();
    }

    TEST_F(JoltForceRegionTests, PointForcePushesOutwardAndPullsWhenNegative)
    {
        JoltForcePoint force;
        force.m_magnitude = 5.0f;

        const AZ::Vector3 outward =
            force.CalculateForce(MakeEntity(AZ::Vector3(3.0f, 0.0f, 0.0f)), MakeRegion());
        EXPECT_TRUE(outward.IsClose(AZ::Vector3(5.0f, 0.0f, 0.0f), 0.01f));

        force.m_magnitude = -5.0f;
        const AZ::Vector3 inward =
            force.CalculateForce(MakeEntity(AZ::Vector3(3.0f, 0.0f, 0.0f)), MakeRegion());
        EXPECT_TRUE(inward.IsClose(AZ::Vector3(-5.0f, 0.0f, 0.0f), 0.01f));
    }

    TEST_F(JoltForceRegionTests, PointForceOnTheCentreProducesNothingRatherThanNaN)
    {
        // Normalizing a zero offset is the obvious way to write this force and the reason
        // a body that lands exactly on the centre would poison the simulation.
        JoltForcePoint force;
        force.m_magnitude = 5.0f;

        const AZ::Vector3 result = force.CalculateForce(MakeEntity(AZ::Vector3::CreateZero()), MakeRegion());
        EXPECT_TRUE(result.IsFinite());
        EXPECT_TRUE(result.IsZero());
    }

    TEST_F(JoltForceRegionTests, DampingOpposesMotionAndScalesWithMass)
    {
        JoltForceLinearDamping force;
        force.m_damping = 3.0f;

        const AZ::Vector3 velocity(4.0f, 0.0f, 0.0f);
        const AZ::Vector3 result = force.CalculateForce(MakeEntity(AZ::Vector3::CreateZero(), velocity), MakeRegion());

        // Mass 2, damping 3, speed 4: opposing, and scaled by mass so the resulting
        // acceleration is the same for a crate and a barrel.
        EXPECT_TRUE(result.IsClose(AZ::Vector3(-24.0f, 0.0f, 0.0f), 0.01f));
    }

    TEST_F(JoltForceRegionTests, DragOpposesMotionAndVanishesAtRest)
    {
        JoltForceSimpleDrag force;
        force.m_dragCoefficient = 1.0f;
        force.m_volumeDensity = 1.0f;

        const AZ::Vector3 moving =
            force.CalculateForce(MakeEntity(AZ::Vector3::CreateZero(), AZ::Vector3(5.0f, 0.0f, 0.0f)), MakeRegion());
        EXPECT_LT(moving.GetX(), 0.0f);
        EXPECT_NEAR(moving.GetY(), 0.0f, 0.01f);

        // A body at rest gets no drag, and specifically not a NaN from normalizing a zero
        // velocity.
        const AZ::Vector3 atRest = force.CalculateForce(MakeEntity(), MakeRegion());
        EXPECT_TRUE(atRest.IsFinite());
        EXPECT_TRUE(atRest.IsZero());
    }

    TEST_F(JoltForceRegionTests, ARegionSumsItsForces)
    {
        // The composition is the point of a region: a fan is a push plus a drag, and what
        // acts on a body is their sum.
        auto push = AZStd::make_shared<JoltForceWorldSpace>();
        push->m_direction = AZ::Vector3(1.0f, 0.0f, 0.0f);
        push->m_magnitude = 20.0f;

        auto damping = AZStd::make_shared<JoltForceLinearDamping>();
        damping->m_damping = 1.0f;

        JoltForceRegion region;
        region.m_forces.push_back(push);
        region.m_forces.push_back(damping);

        const AZ::Vector3 result = region.CalculateNetForce(
            MakeEntity(AZ::Vector3::CreateZero(), AZ::Vector3(4.0f, 0.0f, 0.0f)), MakeRegion());

        // 20 N of push against 4 m/s * 1 damping * 2 kg of resistance.
        EXPECT_TRUE(result.IsClose(AZ::Vector3(12.0f, 0.0f, 0.0f), 0.01f));
    }

    TEST_F(JoltForceRegionTests, AnEmptyRegionAppliesNothing)
    {
        const JoltForceRegion region;
        EXPECT_TRUE(region.CalculateNetForce(MakeEntity(), MakeRegion()).IsZero());
        EXPECT_TRUE(region.GetWindVelocity().IsZero());
    }

    TEST_F(JoltForceRegionTests, WindVelocityComesFromTheRegionsWorldSpaceForce)
    {
        // A wind-tagged region is authored as a world-space push, and that vector is what
        // the engine's wind interface hands to cloth, vegetation and hair.
        auto push = AZStd::make_shared<JoltForceWorldSpace>();
        push->m_direction = AZ::Vector3(0.0f, 1.0f, 0.0f);
        push->m_magnitude = 7.0f;

        JoltForceRegion region;
        region.m_forces.push_back(AZStd::make_shared<JoltForceLinearDamping>());
        region.m_forces.push_back(push);

        EXPECT_TRUE(region.GetWindVelocity().IsClose(AZ::Vector3(0.0f, 7.0f, 0.0f), 0.01f));
    }
} // namespace JoltPhysics
