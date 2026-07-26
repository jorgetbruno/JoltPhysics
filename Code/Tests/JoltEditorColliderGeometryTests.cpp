#include <AzTest/AzTest.h>

#include <AzCore/std/smart_ptr/unique_ptr.h>

#include <Configuration/JoltSettingsRegistryManager.h>
#include <Editor/Components/EditorJoltColliderGeometryUtils.h>
#include <Shape/JoltHeightfieldUtils.h>
#include <Shape/JoltMeshUtils.h>
#include <System/JoltSystem.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace JoltPhysics
{
    using namespace EditorColliderGeometry;

    //! The geometry behind the editor colliders' viewport wireframes and their selection
    //! bounds. A JoltSystem is constructed for its side effect of registering the Jolt
    //! factory and shape types, which building any native shape needs.
    class JoltEditorColliderGeometryTests : public ::testing::Test
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

        //! A grid whose heights vary in both directions, so a transposed or mirrored
        //! mapping cannot pass by symmetry.
        static HeightfieldGrid MakeGrid(AZ::u32 columns, AZ::u32 rows, const AZ::Vector2& spacing)
        {
            HeightfieldGrid grid;
            grid.m_columns = columns;
            grid.m_rows = rows;
            grid.m_spacing = spacing;
            grid.m_heights.reserve(static_cast<size_t>(columns) * rows);
            for (AZ::u32 row = 0; row < rows; ++row)
            {
                for (AZ::u32 column = 0; column < columns; ++column)
                {
                    grid.m_heights.push_back(0.25f * static_cast<float>(column) + static_cast<float>(row));
                }
            }
            return grid;
        }

        //! The heightfield as Jolt actually collides with it: the native shape, wrapped
        //! Z-up exactly as the runtime collider builds it.
        static JPH::RefConst<JPH::Shape> MakeNativeHeightfield(const HeightfieldGrid& grid)
        {
            JPH::PhysicsMaterialList materials;
            materials.push_back(JPH::PhysicsMaterial::sDefault);

            const JPH::RefConst<JPH::Shape> heightfield = JoltHeightfieldUtils::CreateHeightFieldShape(
                grid.m_columns, grid.m_rows, grid.m_spacing, grid.m_heights, {}, materials);
            return heightfield ? JoltHeightfieldUtils::WrapZUp(heightfield.GetPtr()) : nullptr;
        }

        //! Surface vertices of a wrapped heightfield, in entity-local space.
        //!
        //! The Z-up wrapper is a decorated shape and refuses to emit triangles itself, so
        //! its own position and rotation are handed to the heightfield inside it. The
        //! mapping under test is therefore read back out of the production wrapper rather
        //! than restated here.
        static AZStd::vector<AZ::Vector3> ExtractSurfaceVertices(const JPH::Shape* wrappedShape)
        {
            AZStd::vector<AZ::Vector3> vertices;
            const auto* wrapper = static_cast<const JPH::RotatedTranslatedShape*>(wrappedShape);
            const JPH::Shape* heightfield = wrapper->GetInnerShape();

            JPH::Shape::GetTrianglesContext context;
            heightfield->GetTrianglesStart(
                context, JPH::AABox::sBiggest(), wrapper->GetPosition(), wrapper->GetRotation(),
                JPH::Vec3::sReplicate(1.0f));

            constexpr int batchSize = JPH::Shape::cGetTrianglesMinTrianglesRequested;
            JPH::Float3 buffer[batchSize * 3];
            int triangleCount = 0;
            while ((triangleCount = heightfield->GetTrianglesNext(context, batchSize, buffer)) > 0)
            {
                for (int i = 0; i < triangleCount * 3; ++i)
                {
                    vertices.emplace_back(buffer[i].x, buffer[i].y, buffer[i].z);
                }
            }
            return vertices;
        }

        static bool HasVertexNear(const AZStd::vector<AZ::Vector3>& vertices, const AZ::Vector3& target, float tolerance)
        {
            for (const AZ::Vector3& vertex : vertices)
            {
                if (vertex.IsClose(target, tolerance))
                {
                    return true;
                }
            }
            return false;
        }

        AZStd::unique_ptr<JoltSystem> m_system;
    };

    TEST_F(JoltEditorColliderGeometryTests, HeightfieldSamplePositionsSitOnTheJoltCollisionSurface)
    {
        // Non-square, non-uniform spacing: the column and row axes cannot be confused,
        // and neither can their scales.
        const HeightfieldGrid grid = MakeGrid(5, 4, AZ::Vector2(2.0f, 3.0f));

        const JPH::RefConst<JPH::Shape> shape = MakeNativeHeightfield(grid);
        ASSERT_NE(shape, nullptr);

        const AZStd::vector<AZ::Vector3> surfaceVertices = ExtractSurfaceVertices(shape.GetPtr());
        ASSERT_FALSE(surfaceVertices.empty());

        // Every sample the wireframe would draw has to be a point Jolt actually collides
        // at. Jolt stores heights as 16-bit values over a 2000 m range, so Z lands within
        // a quantization step; a wrong axis mapping would be metres out, not centimetres.
        constexpr float quantizationTolerance = 0.05f;
        for (AZ::u32 row = 0; row < grid.m_rows; ++row)
        {
            for (AZ::u32 column = 0; column < grid.m_columns; ++column)
            {
                const AZ::Vector3 sample = HeightfieldSamplePosition(grid, column, row);
                EXPECT_TRUE(HasVertexNear(surfaceVertices, sample, quantizationTolerance))
                    << "sample (" << column << ", " << row << ") at " << sample.GetX() << ", " << sample.GetY()
                    << ", " << sample.GetZ() << " is not on the collision surface";
            }
        }
    }

    TEST_F(JoltEditorColliderGeometryTests, HeightfieldBoundsSpanTheGridAndItsHeights)
    {
        const HeightfieldGrid grid = MakeGrid(5, 4, AZ::Vector2(2.0f, 3.0f));
        const AZ::Aabb bounds = ComputeHeightfieldBounds(grid);
        ASSERT_TRUE(bounds.IsValid());

        // The grid runs along +X and -Y from the entity origin (Jolt heightfields are
        // Y-up and get rotated into place), across 4 columns and 3 rows of spacing.
        EXPECT_NEAR(bounds.GetMin().GetX(), 0.0f, 1e-4f);
        EXPECT_NEAR(bounds.GetMax().GetX(), 8.0f, 1e-4f);
        EXPECT_NEAR(bounds.GetMin().GetY(), -9.0f, 1e-4f);
        EXPECT_NEAR(bounds.GetMax().GetY(), 0.0f, 1e-4f);
        // Heights run 0.25 * column + row, so 0 at the near corner and 4 at the far one.
        EXPECT_NEAR(bounds.GetMin().GetZ(), 0.0f, 1e-4f);
        EXPECT_NEAR(bounds.GetMax().GetZ(), 4.0f, 1e-4f);
    }

    TEST_F(JoltEditorColliderGeometryTests, HeightfieldBoundsEncloseTheNativeShape)
    {
        const HeightfieldGrid grid = MakeGrid(5, 4, AZ::Vector2(2.0f, 3.0f));

        const JPH::RefConst<JPH::Shape> shape = MakeNativeHeightfield(grid);
        ASSERT_NE(shape, nullptr);

        const AZStd::vector<AZ::Vector3> surfaceVertices = ExtractSurfaceVertices(shape.GetPtr());
        ASSERT_FALSE(surfaceVertices.empty());

        // What the viewport picks against must cover what the physics actually occupies,
        // or part of the collider would be unclickable.
        const AZ::Aabb bounds = ComputeHeightfieldBounds(grid).GetExpanded(AZ::Vector3(0.05f));
        for (const AZ::Vector3& vertex : surfaceVertices)
        {
            EXPECT_TRUE(bounds.Contains(vertex));
        }
    }

    TEST_F(JoltEditorColliderGeometryTests, SmallHeightfieldDrawsEveryGridLine)
    {
        const HeightfieldGrid grid = MakeGrid(5, 4, AZ::Vector2(2.0f, 3.0f));
        const AZStd::vector<AZ::Vector3> lines = BuildHeightfieldWireframe(grid, 64);

        // 4 rows of 4 spans plus 5 columns of 3 spans, two points each.
        const size_t expectedSegments = 4 * 4 + 5 * 3;
        EXPECT_EQ(lines.size(), expectedSegments * 2);
    }

    TEST_F(JoltEditorColliderGeometryTests, LargeHeightfieldIsStridedButStillReachesItsFarCorner)
    {
        // Terrain-sized: drawing every line would be over half a million segments.
        const HeightfieldGrid grid = MakeGrid(513, 513, AZ::Vector2(1.0f, 1.0f));
        constexpr AZ::u32 maxLinesPerAxis = 64;
        const AZStd::vector<AZ::Vector3> lines = BuildHeightfieldWireframe(grid, maxLinesPerAxis);

        // Two directions, at most (maxLinesPerAxis + 1) lines each, each of at most
        // (maxLinesPerAxis + 1) segments, two points per segment.
        const size_t worstCase = 2 * (maxLinesPerAxis + 1) * (maxLinesPerAxis + 1) * 2;
        EXPECT_LE(lines.size(), worstCase);
        EXPECT_FALSE(lines.empty());

        // Striding must not shrink the drawn surface: the far corner is still drawn.
        const AZ::Vector3 farCorner = HeightfieldSamplePosition(grid, grid.m_columns - 1, grid.m_rows - 1);
        EXPECT_TRUE(HasVertexNear(lines, farCorner, 1e-3f));
    }

    TEST_F(JoltEditorColliderGeometryTests, AnUnevenStrideStillEndsOnTheLastSample)
    {
        // 10 spans against a budget of 4 lines gives a stride of 3, which does not
        // divide the grid - the last sample has to be added explicitly.
        const HeightfieldGrid grid = MakeGrid(11, 11, AZ::Vector2(1.0f, 1.0f));
        const AZStd::vector<AZ::Vector3> lines = BuildHeightfieldWireframe(grid, 4);

        const AZ::Vector3 farCorner = HeightfieldSamplePosition(grid, 10, 10);
        EXPECT_TRUE(HasVertexNear(lines, farCorner, 1e-3f));
    }

    TEST_F(JoltEditorColliderGeometryTests, AProviderWithNoHeightfieldDrawsNothing)
    {
        // What an entity without a heightfield provider yields: no grid, so no wireframe
        // and no selection bounds rather than a degenerate one at the origin.
        const HeightfieldGrid empty;
        EXPECT_FALSE(empty.IsValid());
        EXPECT_TRUE(BuildHeightfieldWireframe(empty, 64).empty());
        EXPECT_FALSE(ComputeHeightfieldBounds(empty).IsValid());

        // A single row is a line, not a surface, and Jolt rejects it too.
        HeightfieldGrid degenerate = MakeGrid(5, 1, AZ::Vector2(1.0f, 1.0f));
        EXPECT_FALSE(degenerate.IsValid());
        EXPECT_TRUE(BuildHeightfieldWireframe(degenerate, 64).empty());
    }

    TEST_F(JoltEditorColliderGeometryTests, CookedMeshWireframeBoundsMatchTheBakedGeometry)
    {
        // The mesh collider's selection bounds come from the same triangle walk that
        // draws it, so they have to match the baked geometry rather than stay null.
        const AZ::Vector3 corners[8] = {
            AZ::Vector3(-1.0f, -2.0f, -3.0f), AZ::Vector3(1.0f, -2.0f, -3.0f),
            AZ::Vector3(-1.0f, 2.0f, -3.0f),  AZ::Vector3(1.0f, 2.0f, -3.0f),
            AZ::Vector3(-1.0f, -2.0f, 3.0f),  AZ::Vector3(1.0f, -2.0f, 3.0f),
            AZ::Vector3(-1.0f, 2.0f, 3.0f),   AZ::Vector3(1.0f, 2.0f, 3.0f),
        };
        const AZStd::vector<AZ::u8> cookedData = JoltMeshUtils::PackConvexMesh(corners, 8);
        ASSERT_FALSE(cookedData.empty());

        const JPH::RefConst<JPH::Shape> shape = JoltMeshUtils::CreateConvexShapeFromCookedData(cookedData);
        ASSERT_NE(shape, nullptr);

        AZStd::vector<AZ::Vector3> lines;
        AZ::Aabb bounds = AZ::Aabb::CreateNull();
        BuildShapeWireframe(shape.GetPtr(), lines, bounds);

        EXPECT_FALSE(lines.empty());
        EXPECT_EQ(lines.size() % 2, 0u); // a line list, so point pairs
        ASSERT_TRUE(bounds.IsValid());
        // Jolt shrinks convex hulls by the convex radius; the bounds still track the box.
        EXPECT_TRUE(bounds.GetMin().IsClose(AZ::Vector3(-1.0f, -2.0f, -3.0f), 0.06f));
        EXPECT_TRUE(bounds.GetMax().IsClose(AZ::Vector3(1.0f, 2.0f, 3.0f), 0.06f));
    }

    TEST_F(JoltEditorColliderGeometryTests, ANullShapeYieldsNoWireframeAndNoBounds)
    {
        AZStd::vector<AZ::Vector3> lines{ AZ::Vector3::CreateOne() };
        AZ::Aabb bounds = AZ::Aabb::CreateFromPoint(AZ::Vector3::CreateOne());

        // Both outputs are cleared, so a collider whose data failed to load reports
        // nothing rather than whatever it drew last.
        BuildShapeWireframe(nullptr, lines, bounds);
        EXPECT_TRUE(lines.empty());
        EXPECT_FALSE(bounds.IsValid());
    }

} // namespace JoltPhysics
