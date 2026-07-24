#include <Shape/JoltMeshUtils.h>
#include <Utils/Conversions.h>

#include <AzCore/Math/Vector3.h>
#include <AzCore/std/algorithm.h>

#include <Jolt/Jolt.h>
#include <Jolt/Math/Float3.h>
#include <Jolt/Geometry/IndexedTriangle.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>

namespace JoltPhysics
{
    namespace
    {
        // Simple packed format for the "cooked" blob. Jolt needs no real cooking pass
        // (MeshShapeSettings::Create() builds its BVH from the raw triangle list), so
        // this is just a header followed by the vertex and index arrays.
        struct MeshBlobHeader
        {
            AZ::u32 m_magic = 0x4C4A4D48; // 'HMJL' ("Jolt Mesh")
            AZ::u32 m_version = 1;
            AZ::u32 m_vertexCount = 0;
            AZ::u32 m_indexCount = 0; // always a multiple of 3
        };

        // Distinct magic from MeshBlobHeader so a convex blob fed to the triangle-mesh
        // decoder (or vice versa) fails loudly instead of misinterpreting the bytes.
        struct ConvexBlobHeader
        {
            AZ::u32 m_magic = 0x4A435648; // 'HVCJ' ("Jolt Convex Hull")
            AZ::u32 m_version = 1;
            AZ::u32 m_vertexCount = 0;
        };
    }

    AZStd::vector<AZ::u8> JoltMeshUtils::PackTriangleMesh(
        const AZ::Vector3* vertices,
        AZ::u32 vertexCount,
        const AZ::u32* indices,
        AZ::u32 indexCount)
    {
        AZStd::vector<AZ::u8> result;
        if (!vertices || vertexCount == 0 || !indices || indexCount == 0 || indexCount % 3 != 0)
        {
            return result;
        }

        MeshBlobHeader header;
        header.m_vertexCount = vertexCount;
        header.m_indexCount = indexCount;

        const size_t vertexBytes = static_cast<size_t>(vertexCount) * 3 * sizeof(float);
        const size_t indexBytes = static_cast<size_t>(indexCount) * sizeof(AZ::u32);
        result.reserve(sizeof(MeshBlobHeader) + vertexBytes + indexBytes);

        const auto* headerBytes = reinterpret_cast<const AZ::u8*>(&header);
        result.insert(result.end(), headerBytes, headerBytes + sizeof(MeshBlobHeader));

        for (AZ::u32 i = 0; i < vertexCount; ++i)
        {
            const float xyz[3] = { vertices[i].GetX(), vertices[i].GetY(), vertices[i].GetZ() };
            const auto* bytes = reinterpret_cast<const AZ::u8*>(xyz);
            result.insert(result.end(), bytes, bytes + sizeof(xyz));
        }

        const auto* indexBytesPtr = reinterpret_cast<const AZ::u8*>(indices);
        result.insert(result.end(), indexBytesPtr, indexBytesPtr + indexBytes);

        return result;
    }

    JPH::RefConst<JPH::Shape> JoltMeshUtils::CreateMeshShapeFromCookedData(const AZStd::vector<AZ::u8>& cookedData)
    {
        if (cookedData.size() < sizeof(MeshBlobHeader))
        {
            return nullptr;
        }

        MeshBlobHeader header;
        memcpy(&header, cookedData.data(), sizeof(MeshBlobHeader));

        if (header.m_magic != MeshBlobHeader().m_magic || header.m_version != MeshBlobHeader().m_version ||
            header.m_vertexCount == 0 || header.m_indexCount == 0 || header.m_indexCount % 3 != 0)
        {
            AZ_Error("JoltPhysics", false,
                "JoltMeshUtils: cooked mesh blob is malformed or from an incompatible version "
                "(magic=0x%08X version=%u vertexCount=%u indexCount=%u blobSize=%zu)",
                header.m_magic, header.m_version, header.m_vertexCount, header.m_indexCount, cookedData.size());
            return nullptr;
        }

        const size_t vertexBytes = static_cast<size_t>(header.m_vertexCount) * 3 * sizeof(float);
        const size_t indexBytes = static_cast<size_t>(header.m_indexCount) * sizeof(AZ::u32);
        const size_t expectedSize = sizeof(MeshBlobHeader) + vertexBytes + indexBytes;
        if (cookedData.size() != expectedSize)
        {
            AZ_Error("JoltPhysics", false, "JoltMeshUtils: cooked mesh blob size mismatch (expected %zu, got %zu)",
                expectedSize, cookedData.size());
            return nullptr;
        }

        const AZ::u8* vertexCursor = cookedData.data() + sizeof(MeshBlobHeader);
        const AZ::u8* indexCursor = vertexCursor + vertexBytes;

        JPH::VertexList joltVertices;
        joltVertices.reserve(header.m_vertexCount);
        for (AZ::u32 i = 0; i < header.m_vertexCount; ++i)
        {
            float xyz[3];
            memcpy(xyz, vertexCursor + static_cast<size_t>(i) * 3 * sizeof(float), sizeof(xyz));
            joltVertices.push_back(JPH::Float3(xyz[0], xyz[1], xyz[2]));
        }

        JPH::IndexedTriangleList joltTriangles;
        const AZ::u32 triangleCount = header.m_indexCount / 3;
        joltTriangles.reserve(triangleCount);
        AZStd::vector<AZ::u32> rawIndices(header.m_indexCount);
        memcpy(rawIndices.data(), indexCursor, indexBytes);
        for (AZ::u32 t = 0; t < triangleCount; ++t)
        {
            joltTriangles.push_back(JPH::IndexedTriangle(
                rawIndices[t * 3 + 0], rawIndices[t * 3 + 1], rawIndices[t * 3 + 2], /*materialIndex*/ 0));
        }

        JPH::MeshShapeSettings settings(joltVertices, joltTriangles);
        JPH::ShapeSettings::ShapeResult result = settings.Create();
        if (result.HasError())
        {
            AZ_Error("JoltPhysics", false, "Failed to create mesh shape: %s", result.GetError().c_str());
            return nullptr;
        }
        return result.Get();
    }

    AZStd::vector<AZ::u8> JoltMeshUtils::PackConvexMesh(const AZ::Vector3* vertices, AZ::u32 vertexCount)
    {
        AZStd::vector<AZ::u8> result;
        if (!vertices || vertexCount == 0)
        {
            return result;
        }

        ConvexBlobHeader header;
        header.m_vertexCount = vertexCount;

        const size_t vertexBytes = static_cast<size_t>(vertexCount) * 3 * sizeof(float);
        result.reserve(sizeof(ConvexBlobHeader) + vertexBytes);

        const auto* headerBytes = reinterpret_cast<const AZ::u8*>(&header);
        result.insert(result.end(), headerBytes, headerBytes + sizeof(ConvexBlobHeader));

        for (AZ::u32 i = 0; i < vertexCount; ++i)
        {
            const float xyz[3] = { vertices[i].GetX(), vertices[i].GetY(), vertices[i].GetZ() };
            const auto* bytes = reinterpret_cast<const AZ::u8*>(xyz);
            result.insert(result.end(), bytes, bytes + sizeof(xyz));
        }

        return result;
    }

    JPH::RefConst<JPH::Shape> JoltMeshUtils::CreateConvexShapeFromCookedData(const AZStd::vector<AZ::u8>& cookedData)
    {
        if (cookedData.size() < sizeof(ConvexBlobHeader))
        {
            return nullptr;
        }

        ConvexBlobHeader header;
        memcpy(&header, cookedData.data(), sizeof(ConvexBlobHeader));

        if (header.m_magic != ConvexBlobHeader().m_magic || header.m_version != ConvexBlobHeader().m_version ||
            header.m_vertexCount == 0)
        {
            AZ_Error("JoltPhysics", false,
                "JoltMeshUtils: cooked convex blob is malformed or from an incompatible version "
                "(magic=0x%08X version=%u vertexCount=%u blobSize=%zu)",
                header.m_magic, header.m_version, header.m_vertexCount, cookedData.size());
            return nullptr;
        }

        const size_t vertexBytes = static_cast<size_t>(header.m_vertexCount) * 3 * sizeof(float);
        const size_t expectedSize = sizeof(ConvexBlobHeader) + vertexBytes;
        if (cookedData.size() != expectedSize)
        {
            AZ_Error("JoltPhysics", false, "JoltMeshUtils: cooked convex blob size mismatch (expected %zu, got %zu)",
                expectedSize, cookedData.size());
            return nullptr;
        }

        const AZ::u8* vertexCursor = cookedData.data() + sizeof(ConvexBlobHeader);

        JPH::Array<JPH::Vec3> points;
        points.reserve(header.m_vertexCount);
        for (AZ::u32 i = 0; i < header.m_vertexCount; ++i)
        {
            float xyz[3];
            memcpy(xyz, vertexCursor + static_cast<size_t>(i) * 3 * sizeof(float), sizeof(xyz));
            points.push_back(JPH::Vec3(xyz[0], xyz[1], xyz[2]));
        }

        JPH::ConvexHullShapeSettings settings(points);
        JPH::ShapeSettings::ShapeResult result = settings.Create();
        if (result.HasError())
        {
            AZ_Error("JoltPhysics", false, "Failed to create convex hull shape: %s", result.GetError().c_str());
            return nullptr;
        }
        return result.Get();
    }

} // namespace JoltPhysics
