#include <AzTest/AzTest.h>

#include <AzCore/std/chrono/chrono.h>
#include <AzCore/std/parallel/thread.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>

#include <Configuration/JoltSettingsRegistryManager.h>
#include <Editor/EditorJoltConvexDecomposition.h>
#include <Shape/JoltMeshUtils.h>
#include <System/JoltSystem.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace JoltPhysics
{
    using namespace EditorConvexDecomposition;

    //! Automatic convex decomposition (VHACD) for the mesh collider's "Decomposed"
    //! bake mode. A JoltSystem is constructed for its side effect of registering the
    //! Jolt factory and shape types, which building any native shape needs.
    class JoltEditorConvexDecompositionTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            m_system = AZStd::make_unique<JoltSystem>(AZStd::make_unique<JoltSettingsRegistryManager>());
        }

        void TearDown() override
        {
            m_system.reset();
        }

        //! Appends a box's triangle soup (8 corners, 12 triangles).
        static void AppendBoxSoup(
            AZStd::vector<AZ::Vector3>& vertices,
            AZStd::vector<AZ::u32>& indices,
            const AZ::Vector3& center,
            const AZ::Vector3& halfExtents)
        {
            const AZ::u32 baseVertex = static_cast<AZ::u32>(vertices.size());
            for (int i = 0; i < 8; ++i)
            {
                vertices.emplace_back(
                    center.GetX() + ((i & 1) ? halfExtents.GetX() : -halfExtents.GetX()),
                    center.GetY() + ((i & 2) ? halfExtents.GetY() : -halfExtents.GetY()),
                    center.GetZ() + ((i & 4) ? halfExtents.GetZ() : -halfExtents.GetZ()));
            }
            const AZ::u32 boxIndices[36] = {
                0, 1, 3, 0, 3, 2, 4, 6, 7, 4, 7, 5, // z- / z+ caps
                0, 4, 1, 1, 4, 5, 2, 3, 6, 3, 7, 6, // y- / y+ sides
                0, 2, 4, 2, 6, 4, 1, 5, 3, 3, 5, 7, // x- / x+ sides
            };
            for (const AZ::u32 index : boxIndices)
            {
                indices.push_back(baseVertex + index);
            }
        }

        //! An L-shaped concave fixture: a long slab on X with a pillar rising at one end.
        static void MakeLShape(AZStd::vector<AZ::Vector3>& vertices, AZStd::vector<AZ::u32>& indices)
        {
            AppendBoxSoup(vertices, indices, AZ::Vector3(0.0f, 0.0f, 0.0f), AZ::Vector3(2.0f, 0.5f, 0.5f));
            AppendBoxSoup(vertices, indices, AZ::Vector3(1.5f, 0.0f, 2.0f), AZ::Vector3(0.5f, 0.5f, 2.0f));
        }

        static AZ::Aabb BoundsOf(const AZStd::vector<AZ::Vector3>& points)
        {
            AZ::Aabb bounds = AZ::Aabb::CreateNull();
            for (const AZ::Vector3& point : points)
            {
                bounds.AddPoint(point);
            }
            return bounds;
        }

        AZStd::unique_ptr<JoltSystem> m_system;
    };

    TEST_F(JoltEditorConvexDecompositionTests, DecomposesAConcaveSoupIntoMultipleHulls)
    {
        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        MakeLShape(vertices, indices);

        const DecompositionResult result = DecomposeToHullPointClouds(vertices, indices, DecompositionParams());
        ASSERT_TRUE(result.Succeeded());
        // A single hull cannot be concave, so a convex answer would mean no decomposition.
        EXPECT_GE(result.m_hulls.size(), 2u);
    }

    TEST_F(JoltEditorConvexDecompositionTests, DecompositionRespectsThePerHullVertexCap)
    {
        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        MakeLShape(vertices, indices);

        DecompositionParams params;
        params.m_maxVerticesPerHull = 16;
        const DecompositionResult result = DecomposeToHullPointClouds(vertices, indices, params);
        ASSERT_TRUE(result.Succeeded());
        for (const AZStd::vector<AZ::Vector3>& hull : result.m_hulls)
        {
            EXPECT_LE(hull.size(), params.m_maxVerticesPerHull);
        }
    }

    TEST_F(JoltEditorConvexDecompositionTests, HullsCoverTheInputGeometry)
    {
        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        MakeLShape(vertices, indices);

        const DecompositionResult result = DecomposeToHullPointClouds(vertices, indices, DecompositionParams());
        ASSERT_TRUE(result.Succeeded());

        AZStd::vector<AZ::Vector3> allHullPoints;
        for (const AZStd::vector<AZ::Vector3>& hull : result.m_hulls)
        {
            allHullPoints.insert(allHullPoints.end(), hull.begin(), hull.end());
        }

        // VHACD works on a voxel grid, so the hull union hugs the input within a few
        // voxels rather than exactly; it must not swallow the L's inside corner either.
        const AZ::Aabb inputBounds = BoundsOf(vertices);
        const AZ::Aabb hullBounds = BoundsOf(allHullPoints);
        const float tolerance = (inputBounds.GetMax() - inputBounds.GetMin()).GetMaxElement() * 0.25f;
        EXPECT_TRUE(hullBounds.GetMin().IsClose(inputBounds.GetMin(), tolerance));
        EXPECT_TRUE(hullBounds.GetMax().IsClose(inputBounds.GetMax(), tolerance));
    }

    TEST_F(JoltEditorConvexDecompositionTests, DecompositionIsDeterministic)
    {
        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        MakeLShape(vertices, indices);

        const DecompositionParams params;
        const DecompositionResult first = DecomposeToHullPointClouds(vertices, indices, params);
        const DecompositionResult second = DecomposeToHullPointClouds(vertices, indices, params);
        ASSERT_TRUE(first.Succeeded());
        ASSERT_TRUE(second.Succeeded());
        EXPECT_EQ(first.m_hulls.size(), second.m_hulls.size());
    }

    TEST_F(JoltEditorConvexDecompositionTests, DecomposedHullsBakeIntoAWorkingCompound)
    {
        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        MakeLShape(vertices, indices);

        const DecompositionResult result = DecomposeToHullPointClouds(vertices, indices, DecompositionParams());
        ASSERT_GE(result.m_hulls.size(), 2u);

        // The clouds pack into the same hull-group blob the per-node mode produces.
        const AZStd::vector<AZ::u8> blob = JoltMeshUtils::PackConvexHulls(result.m_hulls);
        ASSERT_FALSE(blob.empty());
        const JPH::RefConst<JPH::Shape> shape = JoltMeshUtils::CreateConvexShapeFromCookedData(blob);
        ASSERT_NE(shape, nullptr);
        ASSERT_EQ(shape->GetSubType(), JPH::EShapeSubType::StaticCompound);
        EXPECT_EQ(
            static_cast<const JPH::CompoundShape*>(shape.GetPtr())->GetNumSubShapes(), result.m_hulls.size());
    }

    TEST_F(JoltEditorConvexDecompositionTests, EmptyOrDegenerateInputYieldsNoHulls)
    {
        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        EXPECT_FALSE(DecomposeToHullPointClouds(vertices, indices, DecompositionParams()).Succeeded());

        // A lone triangle has indices but no volume.
        AppendBoxSoup(vertices, indices, AZ::Vector3::CreateZero(), AZ::Vector3::CreateOne());
        vertices.resize(3);
        indices.resize(3);
        EXPECT_FALSE(DecomposeToHullPointClouds(vertices, indices, DecompositionParams()).Succeeded());
    }

    TEST_F(JoltEditorConvexDecompositionTests, ASessionRunsOnItsOwnThreadAndReportsProgressAsItIsPolled)
    {
        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        MakeLShape(vertices, indices);

        DecompositionSession session(vertices, indices, DecompositionParams());
        ASSERT_TRUE(session.IsValid());

        // Polling is what dispatches VHACD's progress messages, which is the whole reason
        // the editor can report a percentage without touching the worker thread.
        while (!session.IsFinished())
        {
            AZStd::this_thread::sleep_for(AZStd::chrono::milliseconds(1));
        }
        EXPECT_GT(session.GetProgress(), 0.0f);

        const DecompositionResult result = session.TakeResult();
        EXPECT_GE(result.m_hulls.size(), 2u);

        // The result moves out, so a second call has nothing left to give.
        EXPECT_FALSE(session.TakeResult().Succeeded());
    }

    TEST_F(JoltEditorConvexDecompositionTests, CancellingASessionStopsTheRunRatherThanAbandoningIt)
    {
        // Enough resolution that the run cannot plausibly finish before the cancel below.
        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        MakeLShape(vertices, indices);

        DecompositionParams params;
        params.m_voxelResolution = 16000000;

        DecompositionSession session(vertices, indices, params);
        ASSERT_TRUE(session.IsValid());

        // Cancel returns only once VHACD's thread has exited, so the run is over - not
        // detached and still burning a core - by the time this line is passed. Cancelling
        // this early is the case worth pinning: VHACD clears its own cancel flag as the
        // run starts, so a signal raised before that used to be swallowed and this call
        // sat through the whole ~20 s run instead of stopping it.
        const auto start = AZStd::chrono::steady_clock::now();
        session.Cancel();
        const auto cancelDuration = AZStd::chrono::steady_clock::now() - start;

        EXPECT_TRUE(session.IsFinished());
        EXPECT_LT(cancelDuration, AZStd::chrono::seconds(5))
            << "cancel should stop the run in well under the time it takes to finish it";

        // Cancelling twice, and cancelling a stopped run, are both fine.
        session.Cancel();
    }

    TEST_F(JoltEditorConvexDecompositionTests, ASessionOverDegenerateInputStartsNothing)
    {
        AZStd::vector<AZ::Vector3> vertices;
        AZStd::vector<AZ::u32> indices;
        AppendBoxSoup(vertices, indices, AZ::Vector3::CreateZero(), AZ::Vector3::CreateOne());
        vertices.resize(3);
        indices.resize(3);

        DecompositionSession session(vertices, indices, DecompositionParams());
        EXPECT_FALSE(session.IsValid());
        // An invalid session still answers, so a caller polling it is not left hanging.
        EXPECT_TRUE(session.IsFinished());
        EXPECT_FALSE(session.TakeResult().Succeeded());
    }

} // namespace JoltPhysics
