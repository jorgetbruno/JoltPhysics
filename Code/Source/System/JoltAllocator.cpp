#include <System/JoltAllocator.h>

#include <AzCore/Memory/SystemAllocator.h>

namespace JoltPhysics
{
    void* JoltAllocator::Allocate(size_t size)
    {
        return AZ::AllocatorInstance<AZ::SystemAllocator>::Get().allocate(size, 16);
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
        JPH::Free = &JoltAllocator::Free;
        JPH::AlignedAllocate = &JoltAllocator::AlignedAllocate;
        JPH::AlignedFree = &JoltAllocator::AlignedFree;
    }

    void JoltAllocator::Uninstall()
    {
        JPH::Allocate = nullptr;
        JPH::Free = nullptr;
        JPH::AlignedAllocate = nullptr;
        JPH::AlignedFree = nullptr;
    }

} // namespace JoltPhysics
