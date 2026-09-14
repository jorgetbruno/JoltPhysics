#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/containers/vector.h>

#include <Configuration/JoltSettingsRegistryManager.h>
#include <Debug/JoltDebugRenderer.h>
#include <System/JoltSystem.h>
#include <Shape/JoltMeshUtils.h>
#include <Utils/Conversions.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace JoltPhysics
{
    // jolt_Debug used to cost 50 ms a frame on a city level: Jolt's simple renderer unpacked
    // every shape's cached triangles on every frame and drew each triangle's three edges, so
    // every interior edge went out twice. The renderer that replaced it keeps each shape's
    // unique edges once and only transforms them per frame. What that must not change is the
    // picture: the same edges, in the same places, just not repeated.
    //
    // No DebugRendererSimple reference to compare against, deliberately. A shape caches the
    // batch the first renderer built for it and every renderer casts it back to its own type,
    // so two renderer types cannot draw the same shape. The expected edges come from the mesh
    // data instead, which also keeps the test independent of how Jolt draws.
    class JoltDebugRendererTests : public ::testing::Test
    {
    protected:
        // A JoltSystem for Jolt's allocator and type registration: without one, the first
        // allocation - building a mesh, or the unit primitives in the renderer's constructor -
        // jumps through a null pointer. No scene; nothing here is simulated.
        void SetUp() override
        {
            m_system = AZStd::make_unique<JoltSystem>(AZStd::make_unique<JoltSettingsRegistryManager>());
            JoltSystemConfiguration config;
            m_system->Initialize(&config);
        }

        void TearDown() override
        {
            m_system->Shutdown();
            m_system.reset();
        }

        AZStd::unique_ptr<JoltSystem> m_system;

        static constexpr int GridCells = 4;
        static constexpr float Tolerance = 1e-4f;

        //! A flat GridCells x GridCells grid of quads, two triangles each.
        struct Grid
        {
            AZStd::vector<AZ::Vector3> m_vertices;
            AZStd::vector<AZ::u32> m_indices;
        };

        static Grid MakeGrid()
        {
            Grid grid;
            for (int y = 0; y <= GridCells; ++y)
            {
                for (int x = 0; x <= GridCells; ++x)
                {
                    grid.m_vertices.emplace_back(static_cast<float>(x), static_cast<float>(y), 0.0f);
                }
            }
            const auto at = [](int x, int y) { return static_cast<AZ::u32>(y * (GridCells + 1) + x); };
            for (int y = 0; y < GridCells; ++y)
            {
                for (int x = 0; x < GridCells; ++x)
                {
                    grid.m_indices.insert(grid.m_indices.end(), { at(x, y), at(x + 1, y), at(x + 1, y + 1) });
                    grid.m_indices.insert(grid.m_indices.end(), { at(x, y), at(x + 1, y + 1), at(x, y + 1) });
                }
            }
            return grid;
        }

        //! Rows and columns of the grid plus one diagonal per cell.
        static constexpr int UniqueEdgeCount = 2 * GridCells * (GridCells + 1) + GridCells * GridCells;

        static JPH::RefConst<JPH::Shape> MakeMesh(const Grid& grid)
        {
            const AZStd::vector<AZ::u8> blob = JoltMeshUtils::PackTriangleMesh(
                grid.m_vertices.data(), static_cast<AZ::u32>(grid.m_vertices.size()),
                grid.m_indices.data(), static_cast<AZ::u32>(grid.m_indices.size()));
            return JoltMeshUtils::CreateMeshShapeFromCookedData(blob);
        }

        struct Segment
        {
            AZ::Vector3 m_a;
            AZ::Vector3 m_b;
        };

        static bool SameSegment(const Segment& lhs, const Segment& rhs)
        {
            return (lhs.m_a.IsClose(rhs.m_a, Tolerance) && lhs.m_b.IsClose(rhs.m_b, Tolerance)) ||
                   (lhs.m_a.IsClose(rhs.m_b, Tolerance) && lhs.m_b.IsClose(rhs.m_a, Tolerance));
        }

        //! The grid's unique edges, transformed, straight from the index data.
        static AZStd::vector<Segment> ExpectedEdges(const Grid& grid, const AZ::Transform& transform)
        {
            AZStd::vector<Segment> edges;
            for (size_t t = 0; t < grid.m_indices.size(); t += 3)
            {
                for (size_t corner = 0; corner < 3; ++corner)
                {
                    const Segment edge{
                        transform.TransformPoint(grid.m_vertices[grid.m_indices[t + corner]]),
                        transform.TransformPoint(grid.m_vertices[grid.m_indices[t + (corner + 1) % 3]]) };
                    bool seen = false;
                    for (const Segment& existing : edges)
                    {
                        seen = seen || SameSegment(existing, edge);
                    }
                    if (!seen)
                    {
                        edges.push_back(edge);
                    }
                }
            }
            return edges;
        }

        static AZStd::vector<Segment> DrawnSegments(const JoltDebugDrawSink& sink)
        {
            AZStd::vector<Segment> segments;
            for (const auto& [color, points] : sink.m_linesByColor)
            {
                for (size_t i = 0; i + 1 < points.size(); i += 2)
                {
                    segments.push_back({ points[i], points[i + 1] });
                }
            }
            return segments;
        }

        static void DrawWireframe(
            JoltDebugRenderer& renderer, const JPH::Shape& shape, const AZ::Transform& transform)
        {
            shape.Draw(&renderer, JPH::RMat44(Conversions::ToJolt(transform)) * JPH::Mat44::sTranslation(shape.GetCenterOfMass()),
                JPH::Vec3::sReplicate(1.0f), JPH::Color::sWhite, false, true);
        }

        //! Every expected edge drawn exactly once, and nothing drawn that is not an edge.
        static void ExpectExactlyTheEdges(const AZStd::vector<Segment>& drawn, const AZStd::vector<Segment>& expected)
        {
            EXPECT_EQ(drawn.size(), expected.size())
                << "drew " << drawn.size() << " lines for " << expected.size() << " unique edges";
            for (const Segment& edge : expected)
            {
                int copies = 0;
                for (const Segment& segment : drawn)
                {
                    copies += SameSegment(segment, edge) ? 1 : 0;
                }
                EXPECT_EQ(copies, 1) << "an edge from (" << edge.m_a.GetX() << "," << edge.m_a.GetY() << ") to ("
                                     << edge.m_b.GetX() << "," << edge.m_b.GetY() << ") was drawn " << copies << " times";
            }
        }
    };

    TEST_F(JoltDebugRendererTests, AMeshDrawsEachOfItsEdgesOnce)
    {
        // 32 triangles are 96 triangle-edges but only 56 distinct lines. The simple renderer
        // drew all 96; every interior line twice.
        const Grid grid = MakeGrid();
        const JPH::RefConst<JPH::Shape> mesh = MakeMesh(grid);
        ASSERT_NE(mesh, nullptr);

        JoltDebugRenderer renderer;
        JoltDebugDrawSink sink;
        renderer.SetTarget(nullptr, &sink);
        DrawWireframe(renderer, *mesh, AZ::Transform::CreateIdentity());

        EXPECT_EQ(renderer.GetGeometryTriangleCount(), static_cast<AZ::u64>(2 * GridCells * GridCells));
        ExpectExactlyTheEdges(DrawnSegments(sink), ExpectedEdges(grid, AZ::Transform::CreateIdentity()));
        EXPECT_EQ(DrawnSegments(sink).size(), static_cast<size_t>(UniqueEdgeCount));
    }

    TEST_F(JoltDebugRendererTests, TheEdgesFollowTheShapesTransform)
    {
        // The edges are kept in the shape's space and transformed per frame; a body that has
        // moved or turned must be drawn where it is, not where its batch was built.
        const Grid grid = MakeGrid();
        const JPH::RefConst<JPH::Shape> mesh = MakeMesh(grid);
        ASSERT_NE(mesh, nullptr);

        JoltDebugRenderer renderer;
        JoltDebugDrawSink sink;
        renderer.SetTarget(nullptr, &sink);

        const AZ::Transform first = AZ::Transform::CreateIdentity();
        DrawWireframe(renderer, *mesh, first);

        // Same shape, same cached batch, a different pose on the next frame.
        sink.Clear();
        const AZ::Transform moved = AZ::Transform::CreateFromQuaternionAndTranslation(
            AZ::Quaternion::CreateRotationZ(0.7f) * AZ::Quaternion::CreateRotationX(0.3f), AZ::Vector3(10.0f, -4.0f, 2.5f));
        DrawWireframe(renderer, *mesh, moved);

        ExpectExactlyTheEdges(DrawnSegments(sink), ExpectedEdges(grid, moved));
    }

    TEST_F(JoltDebugRendererTests, WithoutASinkTheLinesReachTheCallerCallbacks)
    {
        // Physics::SystemDebugRequestBus::DebugDrawPhysics hands a caller's callbacks in; the
        // fast path must not be the only path.
        const Grid grid = MakeGrid();
        const JPH::RefConst<JPH::Shape> mesh = MakeMesh(grid);
        ASSERT_NE(mesh, nullptr);

        AZStd::vector<Segment> received;
        Physics::DebugDrawSettings settings;
        settings.m_isWireframe = true;
        settings.m_udata = &received;
        settings.m_drawLineCB = [](const Physics::DebugDrawVertex& from, const Physics::DebugDrawVertex& to,
                                   [[maybe_unused]] const AZStd::shared_ptr<AzPhysics::SimulatedBody>& body,
                                   [[maybe_unused]] float thickness, void* udata)
        {
            static_cast<AZStd::vector<Segment>*>(udata)->push_back({ from.m_position, to.m_position });
        };

        JoltDebugRenderer renderer;
        renderer.SetTarget(&settings, nullptr);
        DrawWireframe(renderer, *mesh, AZ::Transform::CreateIdentity());

        ExpectExactlyTheEdges(received, ExpectedEdges(grid, AZ::Transform::CreateIdentity()));
    }

    TEST_F(JoltDebugRendererTests, AShapeStillDrawsAfterTheRendererThatBuiltItsBatchIsGone)
    {
        // The renderer lives with the physics system component and is rebuilt when that
        // component is reactivated, but shapes outlive it and keep the batch it built. That
        // batch is refcounted and owns its data, so a second renderer of the same type can
        // draw it - which is what a level reload does.
        const Grid grid = MakeGrid();
        const JPH::RefConst<JPH::Shape> mesh = MakeMesh(grid);
        ASSERT_NE(mesh, nullptr);

        {
            JoltDebugRenderer first;
            JoltDebugDrawSink sink;
            first.SetTarget(nullptr, &sink);
            DrawWireframe(first, *mesh, AZ::Transform::CreateIdentity());
            ASSERT_EQ(DrawnSegments(sink).size(), static_cast<size_t>(UniqueEdgeCount));
        }

        // Jolt asserts there is only ever one renderer, so the second is only legal now.
        JoltDebugRenderer second;
        JoltDebugDrawSink sink;
        second.SetTarget(nullptr, &sink);
        DrawWireframe(second, *mesh, AZ::Transform::CreateIdentity());
        ExpectExactlyTheEdges(DrawnSegments(sink), ExpectedEdges(grid, AZ::Transform::CreateIdentity()));
    }

    TEST_F(JoltDebugRendererTests, ClearingTheSinkKeepsItsCapacity)
    {
        // A warm frame should allocate nothing: the sink is reused, and clearing it must not
        // throw away the buffers the previous frame grew.
        JoltDebugDrawSink sink;
        const AZ::u32 key = AZ::Color(1.0f, 0.0f, 0.0f, 1.0f).ToU32();
        sink.m_linesByColor[key].resize(1000);
        const size_t capacity = sink.m_linesByColor[key].capacity();

        sink.Clear();
        EXPECT_TRUE(sink.m_linesByColor[key].empty());
        EXPECT_EQ(sink.m_linesByColor[key].capacity(), capacity);
    }
} // namespace JoltPhysics
