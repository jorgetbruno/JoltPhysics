#include <Shape/JoltHeightfieldUtils.h>

#include <AzCore/std/numeric.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>

namespace JoltPhysics
{
    AZ::u32 JoltHeightfieldUtils::ComputeSampleCount(AZ::u32 numColumns, AZ::u32 numRows)
    {
        AZ::u32 sampleCount = 2;
        const AZ::u32 needed = AZStd::max(numColumns, numRows);
        while (sampleCount < needed)
        {
            sampleCount *= 2;
        }
        return sampleCount;
    }

    AZStd::vector<AZ::u8> JoltHeightfieldUtils::PadMaterialIndices(
        AZ::u32 numColumns,
        AZ::u32 numRows,
        const AZStd::vector<AZ::u8>& perSampleMaterialIndices,
        AZ::u32 sampleCount)
    {
        // Jolt material indices are per square, (sampleCount - 1)^2 entries;
        // quad (x, y) takes the index of its bottom-left sample.
        AZStd::vector<AZ::u8> padded((sampleCount - 1) * (sampleCount - 1), 0);
        if (perSampleMaterialIndices.empty())
        {
            return padded;
        }

        for (AZ::u32 y = 0; y + 1 < numRows; ++y)
        {
            for (AZ::u32 x = 0; x + 1 < numColumns; ++x)
            {
                const size_t sourceIndex = y * numColumns + x;
                if (sourceIndex < perSampleMaterialIndices.size())
                {
                    padded[y * (sampleCount - 1) + x] = perSampleMaterialIndices[sourceIndex];
                }
            }
        }
        return padded;
    }

    JPH::RefConst<JPH::Shape> JoltHeightfieldUtils::CreateHeightFieldShape(
        AZ::u32 numColumns,
        AZ::u32 numRows,
        const AZ::Vector2& gridSpacing,
        const AZStd::vector<float>& heights,
        const AZStd::vector<AZ::u8>& materialIndices,
        const JPH::PhysicsMaterialList& materials)
    {
        if (numColumns < 2 || numRows < 2 || heights.size() != numColumns * numRows)
        {
            return nullptr;
        }

        // Jolt requires a square grid where (sampleCount / blockSize) is a power of two.
        constexpr AZ::u32 blockSize = 2;
        const AZ::u32 sampleCount = ComputeSampleCount(numColumns, numRows);

        AZStd::vector<float> paddedHeights(sampleCount * sampleCount, FLT_MAX /* cNoCollisionValue */);
        for (AZ::u32 y = 0; y < numRows; ++y)
        {
            for (AZ::u32 x = 0; x < numColumns; ++x)
            {
                paddedHeights[y * sampleCount + x] = heights[y * numColumns + x];
            }
        }

        AZStd::vector<AZ::u8> paddedMaterialIndices = PadMaterialIndices(numColumns, numRows, materialIndices, sampleCount);
        for (auto& index : paddedMaterialIndices)
        {
            if (index >= materials.size())
            {
                index = 0;
            }
        }

        JPH::HeightFieldShapeSettings settings;
        settings.mOffset = JPH::Vec3::sZero();
        settings.mScale = JPH::Vec3(gridSpacing.GetX(), 1.0f, gridSpacing.GetY());
        settings.mSampleCount = sampleCount;
        settings.mBlockSize = blockSize;
        // Allow runtime SetHeights updates across a wide range (SetHeights clamps to
        // [mMinHeightValue, mMaxHeightValue]); 16 bits per sample for precision.
        settings.mMinHeightValue = -1000.0f;
        settings.mMaxHeightValue = 1000.0f;
        settings.mHeightSamples.assign(paddedHeights.begin(), paddedHeights.end());
        settings.mMaterialIndices.assign(paddedMaterialIndices.begin(), paddedMaterialIndices.end());
        settings.mMaterials = materials;

        JPH::ShapeSettings::ShapeResult result = settings.Create();
        if (result.HasError())
        {
            AZ_Error("JoltPhysics", false, "Failed to create heightfield shape: %s", result.GetError().c_str());
            return nullptr;
        }
        return result.Get();
    }

    const JPH::HeightFieldShape* JoltHeightfieldUtils::UnwrapHeightField(const JPH::Shape* shape)
    {
        if (!shape)
        {
            return nullptr;
        }
        if (shape->GetSubType() == JPH::EShapeSubType::HeightField)
        {
            return static_cast<const JPH::HeightFieldShape*>(shape);
        }
        if (shape->GetSubType() == JPH::EShapeSubType::RotatedTranslated)
        {
            const auto* rotatedShape = static_cast<const JPH::RotatedTranslatedShape*>(shape);
            if (rotatedShape->GetInnerShape()->GetSubType() == JPH::EShapeSubType::HeightField)
            {
                return static_cast<const JPH::HeightFieldShape*>(rotatedShape->GetInnerShape());
            }
        }
        return nullptr;
    }

    JPH::RefConst<JPH::Shape> JoltHeightfieldUtils::WrapZUp(const JPH::Shape* heightFieldShape)
    {
        // Jolt heightfields are Y-up (surface = offset + scale * (x, height, y));
        // O3DE is Z-up: rotate +90 degrees around X so (x, height, y) maps to (x, -y, height).
        const JPH::Quat yToZRotation(0.70710678f, 0.0f, 0.0f, 0.70710678f);
        return new JPH::RotatedTranslatedShape(
            JPH::Vec3::sZero(),
            yToZRotation,
            heightFieldShape);
    }

} // namespace JoltPhysics
