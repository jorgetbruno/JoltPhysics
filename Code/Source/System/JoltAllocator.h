#pragma once

#include <AzCore/Memory/SystemAllocator.h>

#include <Jolt/Jolt.h>
#include <Jolt/Core/Memory.h>

namespace JoltPhysics
{
    class JoltAllocator
    {
    public:
        static void* Allocate(size_t size);
        static void* Reallocate(void* block, size_t oldSize, size_t newSize);
        static void Free(void* block);
        static void* AlignedAllocate(size_t size, size_t alignment);
        static void AlignedFree(void* block);

        static void Install();
        static void Uninstall();
    };

} // namespace JoltPhysics
