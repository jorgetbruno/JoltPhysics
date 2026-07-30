#include <Editor/Pipeline/JoltPrimitiveShapeFitter.h>

#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Math/Quaternion.h>

namespace JoltPhysics::Pipeline
{
    namespace
    {
        constexpr float cExtentEpsilon = 1e-5f;

        // Principal axes of the point cloud: eigenvectors of its covariance matrix,
        // found by power iteration with deflation. Easy to get right and deterministic
        // (no dependency on an eigensolver).
        struct PrincipalAxes
        {
            AZ::Vector3 m_centroid = AZ::Vector3::CreateZero();
            AZ::Vector3 m_axis[3]; //!< Columns of the rotation, sorted by descending variance.
            float m_variance[3] = { 0.0f, 0.0f, 0.0f };
        };

        AZ::Vector3 Multiply(const float m[3][3], const AZ::Vector3& v)
        {
            return AZ::Vector3(
                m[0][0] * v.GetX() + m[0][1] * v.GetY() + m[0][2] * v.GetZ(),
                m[1][0] * v.GetX() + m[1][1] * v.GetY() + m[1][2] * v.GetZ(),
                m[2][0] * v.GetX() + m[2][1] * v.GetY() + m[2][2] * v.GetZ());
        }

        // Largest-eigenvalue eigenvector of a symmetric PSD 3x3. Falls back to the
        // coordinate axes when the start direction is (near-)orthogonal to it.
        AZ::Vector3 PowerIterate(const float m[3][3], AZ::Vector3 start)
        {
            AZ::Vector3 v = start.GetNormalized();
            for (int iteration = 0; iteration < 64; ++iteration)
            {
                const AZ::Vector3 next = Multiply(m, v);
                const float length = next.GetLength();
                if (length < 1e-12f)
                {
                    break;
                }
                const AZ::Vector3 converged = next / length;
                if ((converged - v).GetLength() < 1e-7f)
                {
                    v = converged;
                    break;
                }
                v = converged;
            }
            return v.GetLength() > 1e-6f ? v.GetNormalized() : AZ::Vector3::CreateAxisX();
        }

        PrincipalAxes ComputePrincipalAxes(const AZStd::vector<AZ::Vector3>& points)
        {
            PrincipalAxes result;

            for (const AZ::Vector3& point : points)
            {
                result.m_centroid += point;
            }
            result.m_centroid /= static_cast<float>(points.size());

            // Covariance matrix (symmetric).
            float cov[3][3] = {};
            for (const AZ::Vector3& point : points)
            {
                const AZ::Vector3 d = point - result.m_centroid;
                const float v[3] = { d.GetX(), d.GetY(), d.GetZ() };
                for (int r = 0; r < 3; ++r)
                {
                    for (int c = 0; c < 3; ++c)
                    {
                        cov[r][c] += v[r] * v[c];
                    }
                }
            }

            // Three rounds of power iteration + deflation, in descending variance.
            float startSign = 1.0f;
            for (int axis = 0; axis < 3; ++axis)
            {
                // A fixed start that cannot be orthogonal to every direction; flipping
                // the sign between rounds keeps later axes off earlier ones.
                const AZ::Vector3 start(1.0f * startSign, 1.0f, axis == 1 ? -1.0f : 1.0f);
                startSign = -startSign;

                AZ::Vector3 eigenvector = PowerIterate(cov, start);
                const float eigenvalue = eigenvector.Dot(Multiply(cov, eigenvector));

                result.m_axis[axis] = eigenvector;
                result.m_variance[axis] = eigenvalue;

                // Deflate: remove this axis's contribution before the next round.
                for (int r = 0; r < 3; ++r)
                {
                    for (int c = 0; c < 3; ++c)
                    {
                        cov[r][c] -= eigenvalue * eigenvector.GetElement(r) * eigenvector.GetElement(c);
                    }
                }
            }

            // Deflation drift can skew the later axes a few degrees off perpendicular;
            // re-orthogonalize (Gram-Schmidt) so the frame is a proper rotation.
            result.m_axis[0] = result.m_axis[0].GetNormalized();
            result.m_axis[1] = (result.m_axis[1] - result.m_axis[0] * result.m_axis[1].Dot(result.m_axis[0])).GetNormalized();
            result.m_axis[2] = result.m_axis[0].Cross(result.m_axis[1]).GetNormalized();
            return result;
        }

        // The axes as a proper rotation (right-handed) with its translation at the centroid.
        AZ::Transform MakeAxesTransform(const PrincipalAxes& axes, const AZ::Vector3& translation)
        {
            AZ::Vector3 axis0 = axes.m_axis[0];
            AZ::Vector3 axis1 = axes.m_axis[1];
            AZ::Vector3 axis2 = axes.m_axis[2];
            if (axis0.Cross(axis1).Dot(axis2) < 0.0f)
            {
                axis2 = -axis2;
            }
            const AZ::Matrix3x3 rotation = AZ::Matrix3x3::CreateFromColumns(axis0, axis1, axis2);
            return AZ::Transform::CreateFromQuaternionAndTranslation(
                AZ::Quaternion::CreateFromMatrix3x3(rotation).GetNormalized(), translation);
        }

        // O3DE capsules are Z-up, so the capsule's frame puts the principal (long)
        // axis in the rotation's third column. The Gram-Schmidt'd frame is already
        // right-handed, and cycling the columns keeps it that way.
        AZ::Transform MakeCapsuleTransform(const PrincipalAxes& axes, const AZ::Vector3& translation)
        {
            const AZ::Matrix3x3 rotation =
                AZ::Matrix3x3::CreateFromColumns(axes.m_axis[1], axes.m_axis[2], axes.m_axis[0]);
            return AZ::Transform::CreateFromQuaternionAndTranslation(
                AZ::Quaternion::CreateFromMatrix3x3(rotation).GetNormalized(), translation);
        }

        PrimitiveFitResult FitBox(const PrincipalAxes& axes, const AZStd::vector<AZ::Vector3>& points)
        {
            float minProj[3] = { AZStd::numeric_limits<float>::max(), AZStd::numeric_limits<float>::max(), AZStd::numeric_limits<float>::max() };
            float maxProj[3] = { AZStd::numeric_limits<float>::lowest(), AZStd::numeric_limits<float>::lowest(), AZStd::numeric_limits<float>::lowest() };
            for (const AZ::Vector3& point : points)
            {
                for (int axis = 0; axis < 3; ++axis)
                {
                    const float projection = point.Dot(axes.m_axis[axis]);
                    minProj[axis] = AZStd::min(minProj[axis], projection);
                    maxProj[axis] = AZStd::max(maxProj[axis], projection);
                }
            }

            AZ::Vector3 center = AZ::Vector3::CreateZero();
            AZ::Vector3 dimensions = AZ::Vector3::CreateZero();
            for (int axis = 0; axis < 3; ++axis)
            {
                center += axes.m_axis[axis] * ((minProj[axis] + maxProj[axis]) * 0.5f);
                dimensions.SetElement(axis, AZStd::max(maxProj[axis] - minProj[axis], cExtentEpsilon));
            }

            PrimitiveFitResult result;
            result.m_shapeConfig = AZStd::make_shared<Physics::BoxShapeConfiguration>(dimensions);
            result.m_transform = MakeAxesTransform(axes, center);
            return result;
        }

        PrimitiveFitResult FitSphere(const PrincipalAxes& axes, const AZStd::vector<AZ::Vector3>& points)
        {
            float radius = 0.0f;
            for (const AZ::Vector3& point : points)
            {
                radius = AZStd::max(radius, (point - axes.m_centroid).GetLength());
            }

            PrimitiveFitResult result;
            result.m_shapeConfig = AZStd::make_shared<Physics::SphereShapeConfiguration>(AZStd::max(radius, cExtentEpsilon));
            result.m_transform = AZ::Transform::CreateTranslation(axes.m_centroid);
            return result;
        }

        // Fits a capsule along the principal axis: segment extent along it, radius from
        // the perpendicular spread. Falls back to a sphere when the cloud is not
        // meaningfully elongated (a degenerate capsule would be one anyway).
        AZStd::optional<PrimitiveFitResult> FitCapsule(const PrincipalAxes& axes, const AZStd::vector<AZ::Vector3>& points)
        {
            const AZ::Vector3& axis = axes.m_axis[0];
            float minProj = AZStd::numeric_limits<float>::max();
            float maxProj = AZStd::numeric_limits<float>::lowest();
            float radius = 0.0f;
            for (const AZ::Vector3& point : points)
            {
                const float projection = point.Dot(axis);
                minProj = AZStd::min(minProj, projection);
                maxProj = AZStd::max(maxProj, projection);
                radius = AZStd::max(radius, (point - axis * projection).GetLength());
            }

            const float segmentLength = maxProj - minProj;
            if (segmentLength < cExtentEpsilon)
            {
                return AZStd::nullopt;
            }

            const AZ::Vector3 center = axis * ((minProj + maxProj) * 0.5f);
            const float height = segmentLength + 2.0f * radius;

            PrimitiveFitResult result;
            result.m_shapeConfig = AZStd::make_shared<Physics::CapsuleShapeConfiguration>(height, AZStd::max(radius, cExtentEpsilon));
            result.m_transform = MakeCapsuleTransform(axes, center);
            return result;
        }

        float EstimateVolume(const PrimitiveFitResult& fit)
        {
            switch (fit.m_shapeConfig->GetShapeType())
            {
            case Physics::ShapeType::Box:
            {
                const auto& box = static_cast<const Physics::BoxShapeConfiguration&>(*fit.m_shapeConfig);
                return box.m_dimensions.GetX() * box.m_dimensions.GetY() * box.m_dimensions.GetZ();
            }
            case Physics::ShapeType::Sphere:
            {
                const auto& sphere = static_cast<const Physics::SphereShapeConfiguration&>(*fit.m_shapeConfig);
                return 4.18879f * sphere.m_radius * sphere.m_radius * sphere.m_radius;
            }
            case Physics::ShapeType::Capsule:
            {
                const auto& capsule = static_cast<const Physics::CapsuleShapeConfiguration&>(*fit.m_shapeConfig);
                const float segmentLength = AZStd::max(capsule.m_height - 2.0f * capsule.m_radius, 0.0f);
                return 3.14159f * capsule.m_radius * capsule.m_radius * segmentLength +
                    4.18879f * capsule.m_radius * capsule.m_radius * capsule.m_radius;
            }
            default:
                return AZStd::numeric_limits<float>::max();
            }
        }
    } // namespace

    AZStd::optional<PrimitiveFitResult> FitPrimitiveToPoints(
        const AZStd::vector<AZ::Vector3>& points, PrimitiveFitTarget target)
    {
        if (points.size() < 4)
        {
            return AZStd::nullopt;
        }

        const PrincipalAxes axes = ComputePrincipalAxes(points);
        if (axes.m_variance[0] < cExtentEpsilon * cExtentEpsilon)
        {
            // Zero variance in every direction: the cloud is a single point.
            return AZStd::nullopt;
        }

        switch (target)
        {
        case PrimitiveFitTarget::Box:
            return FitBox(axes, points);
        case PrimitiveFitTarget::Sphere:
            return FitSphere(axes, points);
        case PrimitiveFitTarget::Capsule:
        {
            if (AZStd::optional<PrimitiveFitResult> capsule = FitCapsule(axes, points))
            {
                return capsule;
            }
            return FitSphere(axes, points);
        }
        case PrimitiveFitTarget::BestFit:
        default:
        {
            PrimitiveFitResult best = FitBox(axes, points);
            float bestVolume = EstimateVolume(best);

            PrimitiveFitResult sphere = FitSphere(axes, points);
            if (const float sphereVolume = EstimateVolume(sphere); sphereVolume < bestVolume)
            {
                best = AZStd::move(sphere);
                bestVolume = sphereVolume;
            }

            if (AZStd::optional<PrimitiveFitResult> capsule = FitCapsule(axes, points))
            {
                if (const float capsuleVolume = EstimateVolume(*capsule); capsuleVolume < bestVolume)
                {
                    best = AZStd::move(*capsule);
                }
            }
            return best;
        }
        }
    }

} // namespace JoltPhysics::Pipeline
