#include <AzTest/AzTest.h>

#include <Editor/Pipeline/JoltPrimitiveShapeFitter.h>

namespace JoltPhysics
{
    using namespace Pipeline;

    //! The PCA primitive fitter behind the Scene Settings "Primitive" export mode.
    class JoltEditorPrimitiveFitTests : public ::testing::Test
    {
    protected:
        //! The 8 corners of a box as a point cloud.
        static AZStd::vector<AZ::Vector3> MakeBoxCloud(const AZ::Vector3& center, const AZ::Vector3& halfExtents)
        {
            AZStd::vector<AZ::Vector3> points;
            for (int i = 0; i < 8; ++i)
            {
                points.emplace_back(
                    center.GetX() + ((i & 1) ? halfExtents.GetX() : -halfExtents.GetX()),
                    center.GetY() + ((i & 2) ? halfExtents.GetY() : -halfExtents.GetY()),
                    center.GetZ() + ((i & 4) ? halfExtents.GetZ() : -halfExtents.GetZ()));
            }
            return points;
        }

        //! Paired antipodal points on a sphere: mirrored, so the centroid is exactly
        //! the center (an unbiased point set the fitter must reproduce).
        static AZStd::vector<AZ::Vector3> MakeSphereCloud(const AZ::Vector3& center, float radius, int count)
        {
            AZStd::vector<AZ::Vector3> points;
            for (int i = 0; i < count / 2; ++i)
            {
                const float t = static_cast<float>(i) / static_cast<float>(count / 2);
                const float phi = t * 6.28318f * 2.39f;
                const float z = radius * (1.0f - 2.0f * t);
                const float ring = sqrtf(AZStd::max(radius * radius - z * z, 0.0f));
                const AZ::Vector3 point(ring * cosf(phi), ring * sinf(phi), z);
                points.push_back(center + point);
                points.push_back(center - point);
            }
            return points;
        }
    };

    TEST_F(JoltEditorPrimitiveFitTests, BoxFitCoversTheCloudInLocalSpace)
    {
        // Two boxes apart: the fitted box must contain every input point once the
        // cloud is read in the fitted frame.
        AZStd::vector<AZ::Vector3> points = MakeBoxCloud(AZ::Vector3(-2.0f, 0.0f, 0.0f), AZ::Vector3(0.5f, 0.5f, 0.5f));
        const AZStd::vector<AZ::Vector3> more = MakeBoxCloud(AZ::Vector3(2.0f, 1.0f, -1.0f), AZ::Vector3(0.5f, 0.25f, 1.0f));
        points.insert(points.end(), more.begin(), more.end());

        const AZStd::optional<PrimitiveFitResult> fit = FitPrimitiveToPoints(points, PrimitiveFitTarget::Box);
        ASSERT_TRUE(fit.has_value());
        ASSERT_EQ(fit->m_shapeConfig->GetShapeType(), Physics::ShapeType::Box);

        const auto& box = static_cast<const Physics::BoxShapeConfiguration&>(*fit->m_shapeConfig);
        const AZ::Transform inverse = fit->m_transform.GetInverse();
        for (const AZ::Vector3& point : points)
        {
            const AZ::Vector3 local = inverse.TransformPoint(point);
            EXPECT_LE(AZStd::abs(local.GetX()), box.m_dimensions.GetX() * 0.5f + 0.01f);
            EXPECT_LE(AZStd::abs(local.GetY()), box.m_dimensions.GetY() * 0.5f + 0.01f);
            EXPECT_LE(AZStd::abs(local.GetZ()), box.m_dimensions.GetZ() * 0.5f + 0.01f);
        }
    }

    TEST_F(JoltEditorPrimitiveFitTests, SphereFitCentersOnTheCloud)
    {
        const AZStd::vector<AZ::Vector3> points = MakeSphereCloud(AZ::Vector3(5.0f, 0.0f, 2.0f), 3.0f, 64);

        const AZStd::optional<PrimitiveFitResult> fit = FitPrimitiveToPoints(points, PrimitiveFitTarget::Sphere);
        ASSERT_TRUE(fit.has_value());
        ASSERT_EQ(fit->m_shapeConfig->GetShapeType(), Physics::ShapeType::Sphere);

        const auto& sphere = static_cast<const Physics::SphereShapeConfiguration&>(*fit->m_shapeConfig);
        EXPECT_NEAR(fit->m_transform.GetTranslation().GetX(), 5.0f, 0.05f);
        EXPECT_NEAR(fit->m_transform.GetTranslation().GetY(), 0.0f, 0.05f);
        EXPECT_NEAR(fit->m_transform.GetTranslation().GetZ(), 2.0f, 0.05f);
        EXPECT_NEAR(sphere.m_radius, 3.0f, 0.05f);
    }

    TEST_F(JoltEditorPrimitiveFitTests, CapsuleFitAlignsWithTheLongAxis)
    {
        // A cigar of points along the (1, 1, 0) diagonal: the capsule's long axis must
        // agree with it within a few degrees.
        const AZ::Vector3 axis = AZ::Vector3(1.0f, 1.0f, 0.0f).GetNormalized();
        AZStd::vector<AZ::Vector3> points;
        for (int i = 0; i < 32; ++i)
        {
            const float t = -4.0f + 8.0f * static_cast<float>(i) / 31.0f;
            for (int spoke = 0; spoke < 4; ++spoke)
            {
                const float angle = 1.5708f * spoke;
                const AZ::Vector3 radial(0.3f * cosf(angle), -0.3f * cosf(angle), 0.3f * sinf(angle));
                points.push_back(axis * t + radial);
            }
        }

        const AZStd::optional<PrimitiveFitResult> fit = FitPrimitiveToPoints(points, PrimitiveFitTarget::Capsule);
        ASSERT_TRUE(fit.has_value());
        ASSERT_EQ(fit->m_shapeConfig->GetShapeType(), Physics::ShapeType::Capsule);

        const auto& capsule = static_cast<const Physics::CapsuleShapeConfiguration&>(*fit->m_shapeConfig);
        const AZ::Vector3 fittedAxis = fit->m_transform.GetRotation().TransformVector(AZ::Vector3::CreateAxisZ());
        EXPECT_GT(AZStd::abs(fittedAxis.Dot(axis)), 0.995f); // within ~5 degrees
        // Height spans the segment (8 long) plus the two caps of the ~0.42 radius.
        EXPECT_NEAR(capsule.m_height, 8.0f + 2.0f * capsule.m_radius, 0.35f);
        EXPECT_NEAR(capsule.m_radius, 0.42f, 0.15f);
    }

    TEST_F(JoltEditorPrimitiveFitTests, BestFitPicksTheSmallestVolume)
    {
        // A roughly spherical cloud bounds tighter as a sphere than as a box.
        const AZStd::vector<AZ::Vector3> points = MakeSphereCloud(AZ::Vector3::CreateZero(), 2.0f, 48);

        const AZStd::optional<PrimitiveFitResult> fit = FitPrimitiveToPoints(points, PrimitiveFitTarget::BestFit);
        ASSERT_TRUE(fit.has_value());
        EXPECT_EQ(fit->m_shapeConfig->GetShapeType(), Physics::ShapeType::Sphere);
    }

    TEST_F(JoltEditorPrimitiveFitTests, DegenerateInputsFail)
    {
        const AZStd::vector<AZ::Vector3> tooFew = {
            AZ::Vector3::CreateZero(), AZ::Vector3::CreateAxisX(), AZ::Vector3::CreateAxisY()
        };
        EXPECT_FALSE(FitPrimitiveToPoints(tooFew, PrimitiveFitTarget::BestFit).has_value());

        const AZStd::vector<AZ::Vector3> singlePoint(8, AZ::Vector3(1.0f, 2.0f, 3.0f));
        EXPECT_FALSE(FitPrimitiveToPoints(singlePoint, PrimitiveFitTarget::BestFit).has_value());
    }

} // namespace JoltPhysics
