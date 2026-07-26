#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Components/TransformComponent.h>

#include <Editor/Components/EditorJoltBoxColliderComponent.h>
#include <Editor/Components/EditorJoltRigidBodyComponent.h>
#include <Editor/Components/EditorJoltVehicleComponent.h>
#include <Editor/Components/JoltVehicleWheelRequestBus.h>

namespace JoltPhysics
{
    //! Drives wheel placement through JoltVehicleWheelRequestBus, the path
    //! JoltVehicleComponentMode's handles take. The manipulators themselves need a
    //! viewport; everything they do to the component goes through this bus.
    class JoltVehicleComponentModeTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            m_entity = AZStd::make_unique<AZ::Entity>("VehicleModeTest");
            m_entity->CreateComponent<AzFramework::TransformComponent>();
            // The vehicle drives an existing chassis body, which in turn needs a shape.
            m_entity->CreateComponent<EditorJoltBoxColliderComponent>();
            m_entity->CreateComponent<EditorJoltRigidBodyComponent>();
            m_component = m_entity->CreateComponent<EditorJoltVehicleComponent>();
            ASSERT_NE(m_component, nullptr);

            // Two wheels is enough to tell "the right wheel moved" from "a wheel moved".
            JoltVehicleConfiguration& config = m_component->GetVehicleConfiguration();
            JoltWheelConfiguration front;
            front.m_position = AZ::Vector3(0.8f, 0.45f, -0.2f);
            JoltWheelConfiguration rear;
            rear.m_position = AZ::Vector3(-0.8f, -0.45f, -0.2f);
            config.m_wheels = { front, rear };

            m_entity->Init();
            m_entity->Activate();
            ASSERT_EQ(m_entity->GetState(), AZ::Entity::State::Active);

            m_idPair = AZ::EntityComponentIdPair(m_entity->GetId(), m_component->GetId());
        }

        void TearDown() override
        {
            if (m_entity)
            {
                m_entity->Deactivate();
                m_entity.reset();
            }
        }

        AZ::u32 GetWheelCount() const
        {
            AZ::u32 count = 0;
            JoltVehicleWheelRequestBus::EventResult(count, m_idPair, &JoltVehicleWheelRequests::GetWheelCount);
            return count;
        }

        AZ::Vector3 GetWheelPosition(AZ::u32 index) const
        {
            AZ::Vector3 position = AZ::Vector3::CreateZero();
            JoltVehicleWheelRequestBus::EventResult(
                position, m_idPair, &JoltVehicleWheelRequests::GetWheelPosition, index);
            return position;
        }

        void SetWheelPosition(AZ::u32 index, const AZ::Vector3& position)
        {
            JoltVehicleWheelRequestBus::Event(
                m_idPair, &JoltVehicleWheelRequests::SetWheelPosition, index, position);
        }

        AZ::Transform GetChassisSpace() const
        {
            AZ::Transform space = AZ::Transform::CreateIdentity();
            JoltVehicleWheelRequestBus::EventResult(space, m_idPair, &JoltVehicleWheelRequests::GetChassisSpace);
            return space;
        }

        static constexpr float Tolerance = 1e-3f;

        AZStd::unique_ptr<AZ::Entity> m_entity;
        EditorJoltVehicleComponent* m_component = nullptr;
        AZ::EntityComponentIdPair m_idPair;
    };

    TEST_F(JoltVehicleComponentModeTests, TheWheelBusReportsTheAuthoredWheels)
    {
        EXPECT_TRUE(JoltVehicleWheelRequestBus::HasHandlers(m_idPair));
        EXPECT_EQ(GetWheelCount(), 2u);
        EXPECT_TRUE(GetWheelPosition(0).IsClose(AZ::Vector3(0.8f, 0.45f, -0.2f), Tolerance));
        EXPECT_TRUE(GetWheelPosition(1).IsClose(AZ::Vector3(-0.8f, -0.45f, -0.2f), Tolerance));
    }

    TEST_F(JoltVehicleComponentModeTests, DraggingOneWheelLeavesTheOthersAlone)
    {
        const AZ::Vector3 untouched = GetWheelPosition(1);

        SetWheelPosition(0, AZ::Vector3(1.2f, 0.6f, -0.3f));

        EXPECT_TRUE(GetWheelPosition(0).IsClose(AZ::Vector3(1.2f, 0.6f, -0.3f), Tolerance));
        EXPECT_TRUE(GetWheelPosition(1).IsClose(untouched, Tolerance));
    }

    TEST_F(JoltVehicleComponentModeTests, AWheelDraggedInTheViewportIsWhatTheRuntimeWillBuild)
    {
        SetWheelPosition(1, AZ::Vector3(-1.1f, -0.5f, -0.25f));

        // The handle writes into the configuration BuildGameEntity copies, so a drag
        // survives into the game entity rather than only moving the gizmo.
        EXPECT_TRUE(m_component->GetVehicleConfiguration().m_wheels[1].m_position.IsClose(
            AZ::Vector3(-1.1f, -0.5f, -0.25f), Tolerance));
    }

    TEST_F(JoltVehicleComponentModeTests, AnOutOfRangeWheelIsIgnoredRatherThanWritingPastTheList)
    {
        SetWheelPosition(99, AZ::Vector3(5.0f, 5.0f, 5.0f));

        EXPECT_EQ(GetWheelCount(), 2u);
        EXPECT_TRUE(GetWheelPosition(0).IsClose(AZ::Vector3(0.8f, 0.45f, -0.2f), Tolerance));
        EXPECT_TRUE(GetWheelPosition(99).IsClose(AZ::Vector3::CreateZero(), Tolerance));
    }

    TEST_F(JoltVehicleComponentModeTests, TheChassisSpaceFollowsTheEntityWithoutItsScale)
    {
        AZ::Transform scaled = AZ::Transform::CreateTranslation(AZ::Vector3(3.0f, 4.0f, 5.0f));
        scaled.SetUniformScale(2.5f);
        AZ::TransformBus::Event(m_entity->GetId(), &AZ::TransformBus::Events::SetWorldTM, scaled);

        // Wheel positions are plain chassis-space offsets, so carrying scale would put
        // the handles where the wheels are not.
        const AZ::Transform space = GetChassisSpace();
        EXPECT_TRUE(space.GetTranslation().IsClose(AZ::Vector3(3.0f, 4.0f, 5.0f), Tolerance));
        EXPECT_NEAR(space.GetUniformScale(), 1.0f, Tolerance);
    }

    TEST_F(JoltVehicleComponentModeTests, AVehicleWithNoAuthoredWheelsOffersNoHandles)
    {
        // An empty wheel list means "use the type's default layout", which is built at
        // simulation time. There is nothing in the configuration to drag.
        m_component->GetVehicleConfiguration().m_wheels.clear();
        EXPECT_EQ(GetWheelCount(), 0u);
    }

    // Selection bounds are deliberately not tested here. They are reported through
    // EditorComponentSelectionRequestsBus, which is addressed per entity rather than per
    // component, and a vehicle entity necessarily carries a collider too (the vehicle
    // requires a rigid body, which requires a shape). A bus query returns one handler's
    // answer, so the collider's bounds come back instead of the vehicle's and there is
    // no way to tell the two apart from out here.

} // namespace JoltPhysics
