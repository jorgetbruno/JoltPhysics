#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Components/TransformComponent.h>

#include <AzToolsFramework/API/ComponentEntitySelectionBus.h>

#include <Editor/Components/EditorJoltHingeJointComponent.h>
#include <Editor/Components/JoltJointFrameRequestBus.h>

namespace JoltPhysics
{
    //! Drives the joint frame through JoltJointFrameRequestBus, which is the path
    //! JoltJointComponentMode's drag handles take. The manipulators themselves need a
    //! viewport; everything they do to the component goes through this bus.
    class JoltJointComponentModeTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            m_entity = AZStd::make_unique<AZ::Entity>("JointModeTest");
            m_entity->CreateComponent<AzFramework::TransformComponent>();
            m_component = m_entity->CreateComponent<EditorJoltHingeJointComponent>();
            ASSERT_NE(m_component, nullptr);

            m_entity->Init();
            m_entity->Activate();
            ASSERT_EQ(m_entity->GetState(), AZ::Entity::State::Active);

            m_idPair = AZ::EntityComponentIdPair(m_entity->GetId(), m_component->GetId());
        }

        void TearDown() override
        {
            if (m_follower)
            {
                m_follower->Deactivate();
                m_follower.reset();
            }
            if (m_entity)
            {
                m_entity->Deactivate();
                m_entity.reset();
            }
        }

        //! A second entity to act as the joint's follower, at a known transform.
        AZ::EntityId CreateFollower(const AZ::Transform& worldTransform)
        {
            m_follower = AZStd::make_unique<AZ::Entity>("JointFollower");
            m_follower->CreateComponent<AzFramework::TransformComponent>();
            m_follower->Init();
            m_follower->Activate();
            AZ::TransformBus::Event(
                m_follower->GetId(), &AZ::TransformBus::Events::SetWorldTM, worldTransform);
            return m_follower->GetId();
        }

        AZ::Transform GetLocalFrame() const
        {
            AZ::Transform frame = AZ::Transform::CreateIdentity();
            JoltJointFrameRequestBus::EventResult(frame, m_idPair, &JoltJointFrameRequests::GetJointLocalFrame);
            return frame;
        }

        void SetLocalFrame(const AZ::Transform& frame)
        {
            JoltJointFrameRequestBus::Event(m_idPair, &JoltJointFrameRequests::SetJointLocalFrame, frame);
        }

        AZ::Transform GetFrameSpace() const
        {
            AZ::Transform space = AZ::Transform::CreateIdentity();
            JoltJointFrameRequestBus::EventResult(space, m_idPair, &JoltJointFrameRequests::GetJointFrameSpace);
            return space;
        }

        AZ::Aabb GetSelectionBounds() const
        {
            AZ::Aabb bounds = AZ::Aabb::CreateNull();
            AzToolsFramework::EditorComponentSelectionRequestsBus::EventResult(
                bounds, m_entity->GetId(),
                &AzToolsFramework::EditorComponentSelectionRequests::GetEditorSelectionBoundsViewport,
                AzFramework::ViewportInfo{ 0 });
            return bounds;
        }

        static constexpr float Tolerance = 1e-3f;

        AZStd::unique_ptr<AZ::Entity> m_entity;
        AZStd::unique_ptr<AZ::Entity> m_follower;
        EditorJoltHingeJointComponent* m_component = nullptr;
        AZ::EntityComponentIdPair m_idPair;
    };

    TEST_F(JoltJointComponentModeTests, TheFrameBusIsServedSoAJointCanEnterComponentMode)
    {
        // Without a handler the mode would silently edit nothing, which is how the
        // joints behaved before they had one.
        EXPECT_TRUE(JoltJointFrameRequestBus::HasHandlers(m_idPair));
    }

    TEST_F(JoltJointComponentModeTests, ATranslationDragMovesTheFrameAndLeavesItsRotation)
    {
        const AZ::Quaternion rotation = AZ::Quaternion::CreateRotationZ(AZ::DegToRad(30.0f));
        SetLocalFrame(AZ::Transform::CreateFromQuaternionAndTranslation(rotation, AZ::Vector3::CreateZero()));

        // What a linear/planar/surface handle does: take the frame, replace its
        // translation, write it back.
        AZ::Transform dragged = GetLocalFrame();
        dragged.SetTranslation(AZ::Vector3(1.0f, -2.0f, 3.0f));
        SetLocalFrame(dragged);

        const AZ::Transform result = GetLocalFrame();
        EXPECT_TRUE(result.GetTranslation().IsClose(AZ::Vector3(1.0f, -2.0f, 3.0f), Tolerance));
        EXPECT_TRUE(result.GetRotation().IsClose(rotation, Tolerance));
    }

    TEST_F(JoltJointComponentModeTests, ARotationDragTurnsTheFrameAndLeavesItsPosition)
    {
        const AZ::Vector3 position(4.0f, 5.0f, 6.0f);
        SetLocalFrame(AZ::Transform::CreateTranslation(position));

        const AZ::Quaternion rotation = AZ::Quaternion::CreateRotationY(AZ::DegToRad(45.0f));
        AZ::Transform dragged = GetLocalFrame();
        dragged.SetRotation(rotation);
        SetLocalFrame(dragged);

        const AZ::Transform result = GetLocalFrame();
        EXPECT_TRUE(result.GetRotation().IsClose(rotation, Tolerance));
        EXPECT_TRUE(result.GetTranslation().IsClose(position, Tolerance));
    }

    TEST_F(JoltJointComponentModeTests, TheFrameSpaceIsTheJointsOwnEntityWhenNoFollowerIsNamed)
    {
        const AZ::Vector3 entityPosition(7.0f, 8.0f, 9.0f);
        AZ::TransformBus::Event(
            m_entity->GetId(), &AZ::TransformBus::Events::SetWorldTM, AZ::Transform::CreateTranslation(entityPosition));

        // Matching the runtime rule: an unset follower means this entity.
        EXPECT_TRUE(GetFrameSpace().GetTranslation().IsClose(entityPosition, Tolerance));
    }

    TEST_F(JoltJointComponentModeTests, TheFrameSpaceFollowsTheFollowerEntityWhenOneIsNamed)
    {
        AZ::TransformBus::Event(
            m_entity->GetId(), &AZ::TransformBus::Events::SetWorldTM,
            AZ::Transform::CreateTranslation(AZ::Vector3(7.0f, 8.0f, 9.0f)));

        const AZ::Vector3 followerPosition(-1.0f, -2.0f, -3.0f);
        const AZ::EntityId followerId = CreateFollower(AZ::Transform::CreateTranslation(followerPosition));
        m_component->GetJointConfiguration().m_followerEntity = followerId;

        // The frame is expressed in the follower's space, so the handles have to sit on
        // the follower rather than on the entity holding the joint component.
        EXPECT_TRUE(GetFrameSpace().GetTranslation().IsClose(followerPosition, Tolerance));
    }

    TEST_F(JoltJointComponentModeTests, TheFrameSpaceDropsEntityScale)
    {
        AZ::Transform scaled = AZ::Transform::CreateTranslation(AZ::Vector3(1.0f, 0.0f, 0.0f));
        scaled.SetUniformScale(4.0f);
        AZ::TransformBus::Event(m_entity->GetId(), &AZ::TransformBus::Events::SetWorldTM, scaled);

        // Scale changes nothing the joint does, so carrying it would stretch the
        // manipulators and the drawn frame for no reason.
        EXPECT_NEAR(GetFrameSpace().GetUniformScale(), 1.0f, Tolerance);
        EXPECT_TRUE(GetFrameSpace().GetTranslation().IsClose(AZ::Vector3(1.0f, 0.0f, 0.0f), Tolerance));
    }

    TEST_F(JoltJointComponentModeTests, TheJointIsSelectableAtItsFrame)
    {
        AZ::TransformBus::Event(
            m_entity->GetId(), &AZ::TransformBus::Events::SetWorldTM,
            AZ::Transform::CreateTranslation(AZ::Vector3(10.0f, 0.0f, 0.0f)));
        SetLocalFrame(AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 5.0f, 0.0f)));

        // A joint has no geometry; before it reported bounds it could not be clicked in
        // the viewport at all, which also meant no way into component mode.
        const AZ::Aabb bounds = GetSelectionBounds();
        ASSERT_TRUE(bounds.IsValid());
        EXPECT_TRUE(bounds.Contains(AZ::Vector3(10.0f, 5.0f, 0.0f)));
        EXPECT_FALSE(bounds.Contains(AZ::Vector3(10.0f, 0.0f, 0.0f)));
    }

    TEST_F(JoltJointComponentModeTests, SelectionBoundsTrackTheFollowerRatherThanTheJointEntity)
    {
        AZ::TransformBus::Event(
            m_entity->GetId(), &AZ::TransformBus::Events::SetWorldTM,
            AZ::Transform::CreateTranslation(AZ::Vector3(10.0f, 0.0f, 0.0f)));

        const AZ::Vector3 followerPosition(0.0f, 0.0f, 20.0f);
        m_component->GetJointConfiguration().m_followerEntity =
            CreateFollower(AZ::Transform::CreateTranslation(followerPosition));

        const AZ::Aabb bounds = GetSelectionBounds();
        ASSERT_TRUE(bounds.IsValid());
        EXPECT_TRUE(bounds.Contains(followerPosition));
    }

} // namespace JoltPhysics
