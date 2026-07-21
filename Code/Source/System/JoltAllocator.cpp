#include <System/JoltAllocator.h>

#include <AzCore/Memory/SystemAllocator.h>

namespace JoltPhysics
{
    void* JoltAllocator::Allocate(size_t size)
    {
        return AZ::AllocatorInstance<AZ::SystemAllocator>::Get().allocate(size, 16);
    }

    void* JoltAllocator::Reallocate(void* block, [[maybe_unused]] size_t oldSize, size_t newSize)
    {
        if (block)
        {
            return AZ::AllocatorInstance<AZ::SystemAllocator>::Get().reallocate(block, newSize, 16);
        }
        return Allocate(newSize);
    }

    void JoltAllocator::Free(void* block)
    {
        if (block)
        {
            AZ::AllocatorInstance<AZ::SystemAllocator>::Get().deallocate(block);
        }
    }

    void* JoltAllocator::AlignedAllocate(size_t size, size_t alignment)
    {
        return AZ::AllocatorInstance<AZ::SystemAllocator>::Get().allocate(size, alignment);
    }

    void JoltAllocator::AlignedFree(void* block)
    {
        if (block)
        {
            AZ::AllocatorInstance<AZ::SystemAllocator>::Get().deallocate(block);
        }
    }

    void JoltAllocator::Install()
    {
        JPH::Allocate = &JoltAllocator::Allocate;
        JPH::Reallocate = &JoltAllocator::Reallocate;
        JPH::Free = &JoltAllocator::Free;
        JPH::AlignedAllocate = &JoltAllocator::AlignedAllocate;
        JPH::AlignedFree = &JoltAllocator::AlignedFree;
    }

    void JoltAllocator::Uninstall()
    {
        // Intentionally not clearing the JPH allocation hooks: Jolt allocations
        // (shapes, RefConst<T> users) may outlive JoltSystem (e.g. held by callers
        // past Shutdown) and freeing through a null pointer would crash. The hooks
        // only forward to AZ::SystemAllocator, which is available for the process
        // lifetime, so leaving them installed is safe.
    }

} // namespace JoltPhysics
