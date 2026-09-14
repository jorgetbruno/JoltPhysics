#include <Shape/JoltPolygonTriangulation.h>

#include <AzCore/Math/MathUtils.h>

namespace JoltPhysics
{
    namespace
    {
        float Cross(const AZ::Vector2& a, const AZ::Vector2& b, const AZ::Vector2& c)
        {
            // Twice the signed area of triangle abc; positive when c is left of ab.
            return (b.GetX() - a.GetX()) * (c.GetY() - a.GetY()) - (b.GetY() - a.GetY()) * (c.GetX() - a.GetX());
        }

        //! Strictly inside, so a point ON an edge does not block an ear. Without that a
        //! vertex that lies exactly on the diagonal of an ear - which square-ish authored
        //! outlines produce all the time - would leave the polygon unclippable.
        bool PointStrictlyInsideTriangle(
            const AZ::Vector2& p, const AZ::Vector2& a, const AZ::Vector2& b, const AZ::Vector2& c, float orientation)
        {
            const float epsilon = 1e-6f;
            return Cross(a, b, p) * orientation > epsilon &&
                   Cross(b, c, p) * orientation > epsilon &&
                   Cross(c, a, p) * orientation > epsilon;
        }

        //! The outline with consecutive duplicates and an explicit closing vertex removed,
        //! as (original index) so the result still refers to the caller's vertices.
        AZStd::vector<AZ::u32> CleanOutline(const AZStd::vector<AZ::Vector2>& outline)
        {
            AZStd::vector<AZ::u32> indices;
            indices.reserve(outline.size());
            const float epsilonSq = 1e-10f;
            for (AZ::u32 i = 0; i < outline.size(); ++i)
            {
                if (!indices.empty() && outline[i].GetDistanceSq(outline[indices.back()]) <= epsilonSq)
                {
                    continue;
                }
                indices.push_back(i);
            }
            while (indices.size() > 1 && outline[indices.back()].GetDistanceSq(outline[indices.front()]) <= epsilonSq)
            {
                indices.pop_back();
            }
            return indices;
        }
    } // namespace

    float JoltPolygonTriangulation::SignedAreaTwice(const AZStd::vector<AZ::Vector2>& outline)
    {
        float area = 0.0f;
        for (size_t i = 0, n = outline.size(); i < n; ++i)
        {
            const AZ::Vector2& a = outline[i];
            const AZ::Vector2& b = outline[(i + 1) % n];
            area += a.GetX() * b.GetY() - b.GetX() * a.GetY();
        }
        return area;
    }

    bool JoltPolygonTriangulation::IsConvex(const AZStd::vector<AZ::Vector2>& outline)
    {
        const AZStd::vector<AZ::u32> indices = CleanOutline(outline);
        if (indices.size() < 3)
        {
            return false;
        }
        const float orientation = SignedAreaTwice(outline) >= 0.0f ? 1.0f : -1.0f;
        const float epsilon = 1e-6f;
        for (size_t i = 0, n = indices.size(); i < n; ++i)
        {
            const float turn = Cross(outline[indices[i]], outline[indices[(i + 1) % n]], outline[indices[(i + 2) % n]]);
            if (turn * orientation < -epsilon)
            {
                return false;
            }
        }
        return true;
    }

    AZStd::vector<AZ::u32> JoltPolygonTriangulation::EarClip(const AZStd::vector<AZ::Vector2>& outline)
    {
        AZStd::vector<AZ::u32> triangles;

        AZStd::vector<AZ::u32> remaining = CleanOutline(outline);
        if (remaining.size() < 3)
        {
            return triangles;
        }

        const float areaTwice = SignedAreaTwice(outline);
        if (AZ::IsClose(areaTwice, 0.0f, 1e-8f))
        {
            return triangles;
        }
        // +1 for a counter-clockwise outline, -1 for clockwise; every convexity test
        // below is multiplied by it so both windings clip the same way.
        const float orientation = areaTwice > 0.0f ? 1.0f : -1.0f;

        triangles.reserve((remaining.size() - 2) * 3);

        // Clip one ear per pass. An ear is a convex vertex whose triangle with its two
        // neighbours contains no other vertex of the polygon.
        while (remaining.size() > 3)
        {
            bool clipped = false;
            const size_t n = remaining.size();
            for (size_t i = 0; i < n; ++i)
            {
                const AZ::u32 previous = remaining[(i + n - 1) % n];
                const AZ::u32 current = remaining[i];
                const AZ::u32 next = remaining[(i + 1) % n];
                const AZ::Vector2& a = outline[previous];
                const AZ::Vector2& b = outline[current];
                const AZ::Vector2& c = outline[next];

                // Reflex or collinear: not an ear. Collinear is skipped rather than
                // clipped because it would be a zero-area triangle, which the hull
                // builder downstream would reject.
                if (Cross(a, b, c) * orientation <= 1e-6f)
                {
                    continue;
                }

                bool containsAnother = false;
                for (size_t j = 0; j < n && !containsAnother; ++j)
                {
                    const AZ::u32 other = remaining[j];
                    if (other == previous || other == current || other == next)
                    {
                        continue;
                    }
                    containsAnother = PointStrictlyInsideTriangle(outline[other], a, b, c, orientation);
                }
                if (containsAnother)
                {
                    continue;
                }

                triangles.push_back(previous);
                triangles.push_back(current);
                triangles.push_back(next);
                remaining.erase(remaining.begin() + i);
                clipped = true;
                break;
            }

            if (!clipped)
            {
                // No ear anywhere: the two-ears theorem says every simple polygon has
                // one, so this outline is not simple (it crosses itself) or is numerically
                // degenerate. Report nothing rather than a partial fan.
                triangles.clear();
                return triangles;
            }
        }

        // What is left is the last triangle - unless it is collinear, in which case the
        // outline was a sliver and there is nothing to add.
        if (Cross(outline[remaining[0]], outline[remaining[1]], outline[remaining[2]]) * orientation > 1e-6f)
        {
            triangles.push_back(remaining[0]);
            triangles.push_back(remaining[1]);
            triangles.push_back(remaining[2]);
        }
        return triangles;
    }
} // namespace JoltPhysics
