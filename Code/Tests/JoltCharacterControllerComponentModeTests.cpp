#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <AzCore/Component/Entity.h>
#include <AzFramework/Components/TransformComponent.h>

#include <AzToolsFramework/Manipulators/CapsuleManipulatorRequestBus.h>
#include <AzToolsFramework/Manipulators/RadiusManipulatorRequestBus.h>
#include <AzToolsFramework/Manipulators/ShapeManipulatorRequestBus.h>

#include <Editor/Components/EditorJoltCharacterControllerComponent.h>

namespace JoltPhysics
{
    //! Drives the character controller's manipulator buses the way
    //! CapsuleComponentMode does, so the clamping rules are exercised through the same
    //! path a viewport drag takes rather than by calling the setters directly.
    class JoltCharacterControllerComponentModeTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            m_entity = AZStd::make_unique<AZ::Entity>("CharacterControllerModeTest");
            m_entity->CreateComponent<AzFramework::TransformComponent>();
            m_component = m_entity->CreateComponent<EditorJoltCharacterControllerComponent>();
            ASSERT_NE(m_component, nullptr);

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

        float GetHeight() const
        {
            float height = 0.0f;
            AzToolsFramework::CapsuleManipulatorRequestBus::EventResult(
                height, m_idPair, &AzToolsFramework::CapsuleManipulatorRequests::GetHeight);
            return height;
        }

        void SetHeight(float height)
        {
            AzToolsFramework::CapsuleManipulatorRequestBus::Event(
                m_idPair, &AzToolsFramework::CapsuleManipulatorRequests::SetHeight, height);
        }

        float GetRadius() const
        {
            float radius = 0.0f;
            AzToolsFramework::RadiusManipulatorRequestBus::EventResult(
                radius, m_idPair, &AzToolsFramework::RadiusManipulatorRequests::GetRadius);
            return radius;
        }

        void SetRadius(float radius)
        {
            AzToolsFramework::RadiusManipulatorRequestBus::Event(
                m_idPair, &AzToolsFramework::RadiusManipulatorRequests::SetRadius, radius);
        }

        static constexpr float Tolerance = 1e-4f;

        AZStd::unique_ptr<AZ::Entity> m_entity;
        EditorJoltCharacterControllerComponent* m_component = nullptr;
        AZ::EntityComponentIdPair m_idPair;
    };

    TEST_F(JoltCharacterControllerComponentModeTests, ManipulatorBusesAnswerTheDefaultCapsule)
    {
        // The runtime fallback capsule, which the editor component mirrors.
        EXPECT_NEAR(GetHeight(), 1.8f, Tolerance);
        EXPECT_NEAR(GetRadius(), 0.3f, Tolerance);
    }

    TEST_F(JoltCharacterControllerComponentModeTests, HeightAndRadiusRoundTripWithinValidRange)
    {
        SetRadius(0.5f);
        SetHeight(3.0f);

        EXPECT_NEAR(GetRadius(), 0.5f, Tolerance);
        EXPECT_NEAR(GetHeight(), 3.0f, Tolerance);
    }

    TEST_F(JoltCharacterControllerComponentModeTests, HeightIsHeldAtOrAboveTwiceTheRadius)
    {
        SetRadius(0.5f);
        // Dragging the height below the sphere the two caps would form.
        SetHeight(0.2f);

        EXPECT_NEAR(GetHeight(), 1.0f, Tolerance);
        EXPECT_NEAR(GetRadius(), 0.5f, Tolerance);
    }

    TEST_F(JoltCharacterControllerComponentModeTests, RadiusIsHeldAtOrBelowHalfTheHeight)
    {
        SetHeight(2.0f);
        // Dragging the radius past the point where the caps would meet.
        SetRadius(5.0f);

        EXPECT_NEAR(GetRadius(), 1.0f, Tolerance);
        EXPECT_NEAR(GetHeight(), 2.0f, Tolerance);
    }

    TEST_F(JoltCharacterControllerComponentModeTests, RadiusStaysPositive)
    {
        SetRadius(-1.0f);

        EXPECT_GT(GetRadius(), 0.0f);
    }

    TEST_F(JoltCharacterControllerComponentModeTests, ClampingNeverProducesADegenerateCapsule)
    {
        // Whatever order the two are dragged in, height must remain at least the
        // diameter or the backend would have to reject the shape.
        SetRadius(2.0f);
        SetHeight(0.1f);
        EXPECT_GE(GetHeight(), 2.0f * GetRadius() - Tolerance);

        SetHeight(10.0f);
        SetRadius(100.0f);
        EXPECT_GE(GetHeight(), 2.0f * GetRadius() - Tolerance);
    }

    TEST_F(JoltCharacterControllerComponentModeTests, TheCapsuleHasNoTranslationOffset)
    {
        // The character capsule is always centred on the entity, so the offset
        // manipulator has nothing to move and must not drift.
        AZ::Vector3 offset = AZ::Vector3(1.0f, 2.0f, 3.0f);
        AzToolsFramework::ShapeManipulatorRequestBus::EventResult(
            offset, m_idPair, &AzToolsFramework::ShapeManipulatorRequests::GetTranslationOffset);
        EXPECT_TRUE(offset.IsClose(AZ::Vector3::CreateZero(), Tolerance));

        AzToolsFramework::ShapeManipulatorRequestBus::Event(
            m_idPair, &AzToolsFramework::ShapeManipulatorRequests::SetTranslationOffset,
            AZ::Vector3(4.0f, 5.0f, 6.0f));

        AzToolsFramework::ShapeManipulatorRequestBus::EventResult(
            offset, m_idPair, &AzToolsFramework::ShapeManipulatorRequests::GetTranslationOffset);
        EXPECT_TRUE(offset.IsClose(AZ::Vector3::CreateZero(), Tolerance));
    }

} // namespace JoltPhysics
