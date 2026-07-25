#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <Editor/Components/EditorJoltDebugDrawUtils.h>

namespace JoltPhysics
{
    //! Records the line segments a draw helper emits, so the geometry can be
    //! asserted without a viewport. Everything else DebugDisplayRequests offers is
    //! left at its default no-op; the helpers under test only draw lines.
    class RecordingDebugDisplay : public AzFramework::DebugDisplayRequests
    {
    public:
        struct Segment
        {
            AZ::Vector3 m_from;
            AZ::Vector3 m_to;
            AZ::Vector4 m_color;
        };

        void DrawLine(
            const AZ::Vector3& p1, const AZ::Vector3& p2, const AZ::Vector4& col1,
            [[maybe_unused]] const AZ::Vector4& col2) override
        {
            m_segments.push_back({ p1, p2, col1 });
        }

        //! Overridden purely so the coloured overload above does not hide it; none of
        //! the helpers under test call this one.
        void DrawLine(const AZ::Vector3& p1, const AZ::Vector3& p2) override
        {
            m_segments.push_back({ p1, p2, EditorDebugDraw::WireColor });
        }

        //! Every distinct endpoint that was drawn.
        AZStd::vector<AZ::Vector3> Points() const
        {
            AZStd::vector<AZ::Vector3> points;
            points.reserve(m_segments.size() * 2);
            for (const Segment& segment : m_segments)
            {
                points.push_back(segment.m_from);
                points.push_back(segment.m_to);
            }
            return points;
        }

        AZStd::vector<Segment> m_segments;
    };

    class JoltEditorDebugDrawTests : public ::testing::Test
    {
    protected:
        static constexpr float Tolerance = 1e-3f;

        RecordingDebugDisplay m_display;
    };

    TEST_F(JoltEditorDebugDrawTests, WireCapsuleStaysWithinItsRadiusAndHeight)
    {
        constexpr float Radius = 0.3f;
        constexpr float Height = 1.8f;

        EditorDebugDraw::DrawWireCapsule(m_display, AZ::Transform::CreateIdentity(), Radius, Height);

        ASSERT_FALSE(m_display.m_segments.empty());

        float highest = -AZ::Constants::FloatMax;
        float lowest = AZ::Constants::FloatMax;
        for (const AZ::Vector3& point : m_display.Points())
        {
            // The capsule is Z-aligned, so no point may sit further from that axis
            // than the radius.
            const float distanceFromAxis = AZ::Vector2(point.GetX(), point.GetY()).GetLength();
            EXPECT_LE(distanceFromAxis, Radius + Tolerance);

            highest = AZ::GetMax(highest, point.GetZ());
            lowest = AZ::GetMin(lowest, point.GetZ());
        }

        // Height is measured tip to tip, and the capsule is centred on the origin.
        EXPECT_NEAR(highest, Height * 0.5f, Tolerance);
        EXPECT_NEAR(lowest, -Height * 0.5f, Tolerance);
    }

    TEST_F(JoltEditorDebugDrawTests, WireCapsuleDegeneratesToASphereWhenHeightIsBelowTheDiameter)
    {
        constexpr float Radius = 0.5f;
        // A height under 2 * radius has no cylindrical section left.
        EditorDebugDraw::DrawWireCapsule(m_display, AZ::Transform::CreateIdentity(), Radius, 0.2f);

        for (const AZ::Vector3& point : m_display.Points())
        {
            EXPECT_LE(point.GetLength(), Radius + Tolerance);
        }
    }

    TEST_F(JoltEditorDebugDrawTests, WireCapsuleFollowsTheTransform)
    {
        const AZ::Vector3 translation(5.0f, -2.0f, 3.0f);
        EditorDebugDraw::DrawWireCapsule(
            m_display, AZ::Transform::CreateTranslation(translation), 0.25f, 2.0f);

        float highest = -AZ::Constants::FloatMax;
        for (const AZ::Vector3& point : m_display.Points())
        {
            const AZ::Vector3 local = point - translation;
            EXPECT_LE(AZ::Vector2(local.GetX(), local.GetY()).GetLength(), 0.25f + Tolerance);
            highest = AZ::GetMax(highest, local.GetZ());
        }
        EXPECT_NEAR(highest, 1.0f, Tolerance);
    }

    TEST_F(JoltEditorDebugDrawTests, JointFrameDrawsThreeAxesFromTheOrigin)
    {
        constexpr float Length = 0.5f;
        EditorDebugDraw::DrawJointFrame(m_display, AZ::Transform::CreateIdentity(), Length);

        ASSERT_EQ(m_display.m_segments.size(), 3u);
        const AZ::Vector3 expected[] = { AZ::Vector3::CreateAxisX(Length), AZ::Vector3::CreateAxisY(Length),
                                         AZ::Vector3::CreateAxisZ(Length) };
        const AZ::Vector4 expectedColors[] = { EditorDebugDraw::AxisColorX, EditorDebugDraw::AxisColorY,
                                               EditorDebugDraw::AxisColorZ };

        for (size_t i = 0; i < 3; ++i)
        {
            EXPECT_TRUE(m_display.m_segments[i].m_from.IsClose(AZ::Vector3::CreateZero(), Tolerance));
            EXPECT_TRUE(m_display.m_segments[i].m_to.IsClose(expected[i], Tolerance));
            EXPECT_TRUE(m_display.m_segments[i].m_color.IsClose(expectedColors[i], Tolerance));
        }
    }

    TEST_F(JoltEditorDebugDrawTests, LimitArcSweepsTheRequestedAngleRangeAboutX)
    {
        constexpr float Radius = 0.5f;
        // Measured off +Y, sweeping towards +Z.
        EditorDebugDraw::DrawLimitArc(m_display, AZ::Transform::CreateIdentity(), Radius, 0.0f, 90.0f);

        ASSERT_FALSE(m_display.m_segments.empty());

        bool sawStart = false;
        bool sawEnd = false;
        for (const AZ::Vector3& point : m_display.Points())
        {
            if (point.IsClose(AZ::Vector3::CreateZero(), Tolerance))
            {
                continue; // the spokes back to the joint origin
            }
            // The arc lies in the YZ plane at a fixed radius.
            EXPECT_NEAR(point.GetX(), 0.0f, Tolerance);
            EXPECT_NEAR(point.GetLength(), Radius, Tolerance);
            EXPECT_GE(point.GetY(), -Tolerance);
            EXPECT_GE(point.GetZ(), -Tolerance);

            sawStart = sawStart || point.IsClose(AZ::Vector3(0.0f, Radius, 0.0f), Tolerance);
            sawEnd = sawEnd || point.IsClose(AZ::Vector3(0.0f, 0.0f, Radius), Tolerance);
        }
        EXPECT_TRUE(sawStart);
        EXPECT_TRUE(sawEnd);
    }

    TEST_F(JoltEditorDebugDrawTests, LimitArcHandlesANegativeToPositiveRange)
    {
        constexpr float Radius = 1.0f;
        EditorDebugDraw::DrawLimitArc(m_display, AZ::Transform::CreateIdentity(), Radius, -45.0f, 45.0f);

        float lowest = AZ::Constants::FloatMax;
        float highest = -AZ::Constants::FloatMax;
        for (const AZ::Vector3& point : m_display.Points())
        {
            if (point.IsClose(AZ::Vector3::CreateZero(), Tolerance))
            {
                continue;
            }
            lowest = AZ::GetMin(lowest, point.GetZ());
            highest = AZ::GetMax(highest, point.GetZ());
        }
        // sin(+/-45 degrees) at unit radius.
        EXPECT_NEAR(lowest, -0.7071f, 1e-2f);
        EXPECT_NEAR(highest, 0.7071f, 1e-2f);
    }

    TEST_F(JoltEditorDebugDrawTests, LimitConeOpensAlongXAtTheRequestedHalfAngle)
    {
        constexpr float Length = 1.0f;
        constexpr float HalfAngle = 30.0f;
        EditorDebugDraw::DrawLimitCone(m_display, AZ::Transform::CreateIdentity(), Length, HalfAngle, HalfAngle);

        ASSERT_FALSE(m_display.m_segments.empty());

        for (const AZ::Vector3& point : m_display.Points())
        {
            if (point.IsClose(AZ::Vector3::CreateZero(), Tolerance))
            {
                continue; // spokes start at the apex
            }
            // Rim points sit on the cone: fixed length, fixed angle off +X.
            EXPECT_NEAR(point.GetLength(), Length, Tolerance);
            const float angle = AZ::RadToDeg(acosf(point.GetNormalized().Dot(AZ::Vector3::CreateAxisX())));
            EXPECT_NEAR(angle, HalfAngle, 0.1f);
        }
    }

    TEST_F(JoltEditorDebugDrawTests, LimitConeUsesIndependentHalfAnglesPerAxis)
    {
        constexpr float Length = 1.0f;
        // Wide about Y, narrow about Z.
        EditorDebugDraw::DrawLimitCone(m_display, AZ::Transform::CreateIdentity(), Length, 60.0f, 10.0f);

        float widestY = 0.0f;
        float widestZ = 0.0f;
        for (const AZ::Vector3& point : m_display.Points())
        {
            widestY = AZ::GetMax(widestY, fabsf(point.GetY()));
            widestZ = AZ::GetMax(widestZ, fabsf(point.GetZ()));
        }
        // sin(60) vs sin(10) on a unit-length rim.
        EXPECT_NEAR(widestY, 0.8660f, 1e-2f);
        EXPECT_NEAR(widestZ, 0.1736f, 1e-2f);
        EXPECT_GT(widestY, widestZ);
    }

    TEST_F(JoltEditorDebugDrawTests, LimitConeClampsAtOrBeyondAQuarterTurn)
    {
        // 90 degrees and beyond would send tan() to infinity; the helper clamps so
        // the cone stays finite rather than producing NaN vertices.
        EditorDebugDraw::DrawLimitCone(m_display, AZ::Transform::CreateIdentity(), 1.0f, 90.0f, 120.0f);

        ASSERT_FALSE(m_display.m_segments.empty());
        for (const AZ::Vector3& point : m_display.Points())
        {
            EXPECT_TRUE(point.IsFinite());
            if (point.IsClose(AZ::Vector3::CreateZero(), Tolerance))
            {
                continue; // spokes start at the apex
            }
            EXPECT_NEAR(point.GetLength(), 1.0f, Tolerance);
        }
    }

    TEST_F(JoltEditorDebugDrawTests, WireSphereKeepsEveryPointOnTheSurface)
    {
        constexpr float Radius = 2.0f;
        const AZ::Vector3 centre(1.0f, 2.0f, -3.0f);

        EditorDebugDraw::DrawWireSphere(
            m_display, AZ::Transform::CreateTranslation(centre), Radius, EditorDebugDraw::LimitColor);

        ASSERT_FALSE(m_display.m_segments.empty());
        for (const AZ::Vector3& point : m_display.Points())
        {
            EXPECT_NEAR((point - centre).GetLength(), Radius, Tolerance);
        }
        for (const RecordingDebugDisplay::Segment& segment : m_display.m_segments)
        {
            EXPECT_TRUE(segment.m_color.IsClose(EditorDebugDraw::LimitColor, Tolerance));
        }
    }

} // namespace JoltPhysics
