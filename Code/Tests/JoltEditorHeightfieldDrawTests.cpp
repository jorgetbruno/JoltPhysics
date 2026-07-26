#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <AzCore/Component/Entity.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Physics/HeightfieldProviderBus.h>

#include <Editor/Components/EditorJoltHeightfieldColliderComponent.h>

namespace JoltPhysics
{
    namespace
    {
        //! A heightfield provider whose data can be changed either with or without
        //! announcing it, which is the whole point of these tests.
        class TestHeightfieldProvider : public Physics::HeightfieldProviderRequestsBus::Handler
        {
        public:
            TestHeightfieldProvider(AZ::EntityId entityId, AZ::u32 columns, AZ::u32 rows)
                : m_entityId(entityId)
                , m_columns(columns)
                , m_rows(rows)
            {
                m_heights.assign(static_cast<size_t>(columns) * rows, 0.0f);
                Physics::HeightfieldProviderRequestsBus::Handler::BusConnect(entityId);
            }

            ~TestHeightfieldProvider() override
            {
                Physics::HeightfieldProviderRequestsBus::Handler::BusDisconnect();
            }

            //! What a provider that forgets to announce its edits does.
            void RaiseTerrainSilently(float height)
            {
                m_heights.assign(m_heights.size(), height);
            }

            //! What a well-behaved one does.
            void RaiseTerrainAndAnnounce(float height)
            {
                RaiseTerrainSilently(height);
                Physics::HeightfieldProviderNotificationBus::Event(
                    m_entityId, &Physics::HeightfieldProviderNotifications::OnHeightfieldDataChanged,
                    AZ::Aabb::CreateNull(), Physics::HeightfieldProviderNotifications::HeightfieldChangeMask::HeightData);
            }

            AZ::Vector2 GetHeightfieldGridSpacing() const override { return AZ::Vector2(1.0f, 1.0f); }
            void GetHeightfieldGridSize(size_t& numColumns, size_t& numRows) const override
            {
                numColumns = m_columns;
                numRows = m_rows;
            }
            AZ::u64 GetHeightfieldGridColumns() const override { return m_columns; }
            AZ::u64 GetHeightfieldGridRows() const override { return m_rows; }
            void GetHeightfieldHeightBounds(float& minHeightBounds, float& maxHeightBounds) const override
            {
                minHeightBounds = -10.0f;
                maxHeightBounds = 10.0f;
            }
            float GetHeightfieldMinHeight() const override { return -10.0f; }
            float GetHeightfieldMaxHeight() const override { return 10.0f; }
            AZ::Aabb GetHeightfieldAabb() const override { return AZ::Aabb::CreateNull(); }
            AZ::Transform GetHeightfieldTransform() const override { return AZ::Transform::CreateIdentity(); }
            AZStd::vector<AZ::Data::Asset<Physics::MaterialAsset>> GetMaterialList() const override { return {}; }
            AZStd::vector<float> GetHeights() const override { return m_heights; }
            AZStd::vector<Physics::HeightMaterialPoint> GetHeightsAndMaterials() const override
            {
                AZStd::vector<Physics::HeightMaterialPoint> points;
                points.reserve(m_heights.size());
                for (const float height : m_heights)
                {
                    points.emplace_back(height, Physics::QuadMeshType::SubdivideUpperLeftToBottomRight, 0);
                }
                return points;
            }
            void GetHeightfieldIndicesFromRegion(
                const AZ::Aabb&, size_t& startColumn, size_t& startRow, size_t& numColumns, size_t& numRows) const override
            {
                startColumn = 0;
                startRow = 0;
                numColumns = m_columns;
                numRows = m_rows;
            }
            void UpdateHeightsAndMaterials(
                const Physics::UpdateHeightfieldSampleFunction&, size_t, size_t, size_t, size_t) const override
            {
            }
            void UpdateHeightsAndMaterialsAsync(
                const Physics::UpdateHeightfieldSampleFunction&, const Physics::UpdateHeightfieldCompleteFunction&,
                size_t, size_t, size_t, size_t) const override
            {
            }

        private:
            AZ::EntityId m_entityId;
            AZ::u32 m_columns;
            AZ::u32 m_rows;
            AZStd::vector<float> m_heights;
        };

        //! Captures the batched line list the heightfield collider draws.
        class LineListRecorder : public AzFramework::DebugDisplayRequests
        {
        public:
            void DrawLines(const AZStd::vector<AZ::Vector3>& lines, [[maybe_unused]] const AZ::Color& color) override
            {
                m_lines = lines;
            }

            //! Highest Z of anything drawn, which tracks the terrain height.
            float HighestPoint() const
            {
                float highest = -AZStd::numeric_limits<float>::max();
                for (const AZ::Vector3& point : m_lines)
                {
                    highest = AZStd::max(highest, static_cast<float>(point.GetZ()));
                }
                return highest;
            }

            AZStd::vector<AZ::Vector3> m_lines;
        };
    } // namespace

    //! The heightfield collider caches the provider's grid rather than copying it every
    //! frame, so these cover how it finds out the grid has moved on.
    class JoltEditorHeightfieldDrawTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            m_entity = AZStd::make_unique<AZ::Entity>("HeightfieldDrawTest");
            m_entity->CreateComponent<AzFramework::TransformComponent>();
            m_entity->CreateComponent<EditorJoltHeightfieldColliderComponent>();

            m_entity->Init();
            m_entity->Activate();
            ASSERT_EQ(m_entity->GetState(), AZ::Entity::State::Active);

            m_provider = AZStd::make_unique<TestHeightfieldProvider>(m_entity->GetId(), 8, 8);
        }

        void TearDown() override
        {
            m_provider.reset();
            if (m_entity)
            {
                m_entity->Deactivate();
                m_entity.reset();
            }
        }

        //! One viewport frame; returns the highest point drawn.
        float Draw()
        {
            LineListRecorder recorder;
            AzFramework::EntityDebugDisplayEventBus::Event(
                m_entity->GetId(), &AzFramework::EntityDebugDisplayEvents::DisplayEntityViewport,
                AzFramework::ViewportInfo{ 0 }, recorder);
            return recorder.HighestPoint();
        }

        float DrawTimes(int frames)
        {
            float highest = 0.0f;
            for (int i = 0; i < frames; ++i)
            {
                highest = Draw();
            }
            return highest;
        }

        static constexpr float Tolerance = 1e-3f;

        AZStd::unique_ptr<AZ::Entity> m_entity;
        AZStd::unique_ptr<TestHeightfieldProvider> m_provider;
    };

    TEST_F(JoltEditorHeightfieldDrawTests, TheSurfaceIsDrawnFromTheProvidersGrid)
    {
        m_provider->RaiseTerrainAndAnnounce(3.0f);
        EXPECT_NEAR(Draw(), 3.0f, Tolerance);
    }

    TEST_F(JoltEditorHeightfieldDrawTests, AnAnnouncedChangeIsDrawnOnTheNextFrame)
    {
        m_provider->RaiseTerrainAndAnnounce(1.0f);
        ASSERT_NEAR(Draw(), 1.0f, Tolerance);

        // A provider that announces its edits needs no polling at all.
        m_provider->RaiseTerrainAndAnnounce(5.0f);
        EXPECT_NEAR(Draw(), 5.0f, Tolerance);
    }

    TEST_F(JoltEditorHeightfieldDrawTests, ASilentChangeIsPickedUpByThePeriodicReRead)
    {
        m_provider->RaiseTerrainAndAnnounce(1.0f);
        ASSERT_NEAR(Draw(), 1.0f, Tolerance);

        m_provider->RaiseTerrainSilently(7.0f);

        // Nothing announced it, so the next frame still shows the old surface: the
        // grid is cached precisely so it is not re-read every frame.
        EXPECT_NEAR(Draw(), 1.0f, Tolerance);

        // Within a bounded number of frames the re-read finds it anyway. This is the
        // safety net the runtime component gets from its per-tick poll.
        EXPECT_NEAR(DrawTimes(32), 7.0f, Tolerance);
    }

    TEST_F(JoltEditorHeightfieldDrawTests, AGridThatHasNotChangedIsNotRebuilt)
    {
        m_provider->RaiseTerrainAndAnnounce(2.0f);
        ASSERT_NEAR(Draw(), 2.0f, Tolerance);

        // Drawing a static terrain for a while re-reads it, finds it identical and
        // leaves the wireframe exactly as it was.
        EXPECT_NEAR(DrawTimes(64), 2.0f, Tolerance);
    }

    TEST_F(JoltEditorHeightfieldDrawTests, AnEntityWithNoProviderDrawsNothing)
    {
        m_provider.reset(); // disconnects the provider bus

        LineListRecorder recorder;
        AzFramework::EntityDebugDisplayEventBus::Event(
            m_entity->GetId(), &AzFramework::EntityDebugDisplayEvents::DisplayEntityViewport,
            AzFramework::ViewportInfo{ 0 }, recorder);

        EXPECT_TRUE(recorder.m_lines.empty());
    }

} // namespace JoltPhysics
