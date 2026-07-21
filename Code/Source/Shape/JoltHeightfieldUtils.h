#pragma once

#include <AzFramework/Physics/HeightfieldProviderBus.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>

namespace JoltPhysics
{
    class JoltHeightfieldUtils
    {
    public:
        //! Builds a Jolt heightfield from grid data (row-major heights, numRows x numColumns
        //! vertices). Jolt heightfields must be square with a block count that is a power of
        //! two; out-of-range samples are padded as no-collision holes.
        static JPH::RefConst<JPH::Shape> CreateHeightFieldShape(
            AZ::u32 numColumns,
            AZ::u32 numRows,
            const AZ::Vector2& gridSpacing,
            const AZStd::vector<float>& heights,
            const AZStd::vector<AZ::u8>& materialIndices,
            const JPH::PhysicsMaterialList& materials);

        //! Wraps a heightfield shape rotated from Jolt's Y-up convention to O3DE's Z-up.
        static JPH::RefConst<JPH::Shape> WrapZUp(const JPH::Shape* heightFieldShape);

        //! Returns the HeightFieldShape from a shape that is one directly or wrapped by WrapZUp.
        static const JPH::HeightFieldShape* UnwrapHeightField(const JPH::Shape* shape);

        //! Pads per-sample material indices (numRows x numColumns) into the square,
        //! per-square layout Jolt expects ((sampleCount - 1)^2 entries, no-collision
        //! squares marked with 0xff).
        static AZStd::vector<AZ::u8> PadMaterialIndices(
            AZ::u32 numColumns,
            AZ::u32 numRows,
            const AZStd::vector<AZ::u8>& perSampleMaterialIndices,
            AZ::u32 sampleCount);

        //! Smallest valid Jolt sample count (square, power-of-two block count) covering
        //! numColumns x numRows.
        static AZ::u32 ComputeSampleCount(AZ::u32 numColumns, AZ::u32 numRows);
    };

} // namespace JoltPhysics
