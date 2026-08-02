#pragma once

//! Per-module setup of Jolt's global state, for gems that link Jolt through
//! Gem::JoltPhysics.API.
//!
//! **Every module that calls into Jolt must install these, including this gem's own.**
//! Jolt is statically linked into each module that uses it, so each one owns a private
//! copy of Jolt's globals - and the ones that matter start null:
//!
//! - the five allocation function pointers (`JPH::Allocate` and friends)
//! - `JPH::Factory::sInstance`
//!
//! The physics gem installing *its* copy does nothing for anyone else's. Missing the
//! hooks is a jump through a null pointer on a physics job thread the first time Jolt
//! allocates anything - historically inside `PhysicsSystem::AddStepListener`, a long way
//! from anything that looks like the cause. Missing the factory is an access violation
//! inside `JPH::Factory::Find`. `JPH::Trace` and `JPH::AssertFailed` are the exception:
//! those default to harmless stubs, so they are a diagnostic improvement rather than a
//! hazard.
//!
//! This header exists because that ritual was being hand-copied into every extension gem
//! (JoltBuoyancy, JoltHair), each rediscovering it by crashing. Header-only and inline so
//! it works through the INTERFACE API target with nothing to link.
//!
//! **The allocator is not a free choice.** These hooks forward to `AZ::SystemAllocator`,
//! identical to the physics gem's, because allocations cross the module boundary: an
//! extension gem's `AddStepListener` grows an array that the physics gem's
//! `~PhysicsSystem` later frees. `JPH::RegisterDefaultAllocator` compiles and runs and
//! routes to malloc, handing the physics gem a block `AZ::SystemAllocator` never issued.
//!
//! Typical use, from a module's constructor and destructor:
//!
//!     MyGemModule::MyGemModule()
//!     {
//!         JoltPhysics::InstallJoltModuleGlobals();
//!         ...
//!     }
//!     MyGemModule::~MyGemModule()
//!     {
//!         JoltPhysics::UninstallJoltModuleGlobals();
//!     }
//!
//! Tearing down from the module destructor rather than leaving it to static destruction
//! is deliberate: Jolt's globals must come down while the AZ allocators are still alive,
//! or the process aborts silently on exit.

#include <AzCore/Debug/Trace.h>
#include <AzCore/Memory/SystemAllocator.h>

#include <Jolt/Jolt.h>
#include <Jolt/Core/Core.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/Memory.h>
#include <Jolt/RegisterTypes.h>

#include <cstdarg>
#include <cstdio>

namespace JoltPhysics
{
    namespace ModuleGlobals
    {
        inline void* Allocate(size_t size)
        {
            return AZ::AllocatorInstance<AZ::SystemAllocator>::Get().allocate(size, 16);
        }

        inline void* Reallocate(void* block, [[maybe_unused]] size_t oldSize, size_t newSize)
        {
            if (block)
            {
                return AZ::AllocatorInstance<AZ::SystemAllocator>::Get().reallocate(block, newSize, 16);
            }
            return Allocate(newSize);
        }

        inline void Free(void* block)
        {
            if (block)
            {
                AZ::AllocatorInstance<AZ::SystemAllocator>::Get().deallocate(block);
            }
        }

        inline void* AlignedAllocate(size_t size, size_t alignment)
        {
            return AZ::AllocatorInstance<AZ::SystemAllocator>::Get().allocate(size, alignment);
        }

        inline void AlignedFree(void* block)
        {
            if (block)
            {
                AZ::AllocatorInstance<AZ::SystemAllocator>::Get().deallocate(block);
            }
        }

        inline void TraceCallback(const char* format, ...)
        {
            char buffer[1024];
            va_list args;
            va_start(args, format);
            vsnprintf(buffer, sizeof(buffer), format, args);
            va_end(args);
            AZ_TracePrintf("JoltPhysics", "%s", buffer);
        }

#ifdef JPH_ENABLE_ASSERTS
        inline bool AssertFailedCallback(const char* expression, const char* message, const char* file, unsigned int line)
        {
            AZ_Error("JoltPhysics", false, "Jolt Assert: %s (%s) at %s:%u", expression, message ? message : "", file, line);
            return true;
        }
#endif
    } // namespace ModuleGlobals

    //! Points this module's Jolt allocation hooks at AZ::SystemAllocator. Idempotent, and
    //! the first thing to run - registering types allocates.
    inline void InstallJoltAllocationHooks()
    {
        JPH::Allocate = &ModuleGlobals::Allocate;
        JPH::Reallocate = &ModuleGlobals::Reallocate;
        JPH::Free = &ModuleGlobals::Free;
        JPH::AlignedAllocate = &ModuleGlobals::AlignedAllocate;
        JPH::AlignedFree = &ModuleGlobals::AlignedFree;
    }

    //! Routes this module's Jolt traces and asserts into the AZ trace system. Optional -
    //! Jolt's defaults are harmless stubs - but without it a Jolt assert in this module
    //! passes silently.
    inline void InstallJoltTraceHandlers()
    {
        JPH::Trace = &ModuleGlobals::TraceCallback;
#ifdef JPH_ENABLE_ASSERTS
        JPH::AssertFailed = &ModuleGlobals::AssertFailedCallback;
#endif
    }

    //! Creates this module's Jolt factory and registers the built-in types, needed by any
    //! module that constructs shapes or other factory-created Jolt objects.
    //!
    //! Returns true when it created the factory, false when this module already had one.
    //! Extension registrations that must follow `RegisterTypes` - Jolt's `RegisterHair`,
    //! for instance - belong behind that return value, for the same reason the type
    //! registration is: `JPH::RegisterTypes` has no re-entry guard of its own and simply
    //! re-registers, growing the factory's tables every call.
    inline bool InstallJoltFactory()
    {
        if (JPH::Factory::sInstance != nullptr)
        {
            // Replacing a live factory would strand every type already registered in it.
            return false;
        }
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
        return true;
    }

    //! All three, in the order they depend on each other. Safe to call more than once.
    inline void InstallJoltModuleGlobals()
    {
        InstallJoltAllocationHooks();
        InstallJoltTraceHandlers();
        InstallJoltFactory();
    }

    //! Drops this module's factory and type registrations.
    //!
    //! Call it from the module destructor, not from static destruction: these have to come
    //! down while the AZ allocators are still alive.
    //!
    //! The allocation hooks are deliberately left installed. Jolt objects (shapes,
    //! `RefConst` holders) can outlive this call, and freeing one through a null hook
    //! would crash; the hooks only forward to `AZ::SystemAllocator`, which lives for the
    //! whole process, so leaving them in place costs nothing.
    inline void UninstallJoltModuleGlobals()
    {
        if (JPH::Factory::sInstance != nullptr)
        {
            JPH::UnregisterTypes();
            delete JPH::Factory::sInstance;
            JPH::Factory::sInstance = nullptr;
        }
    }
} // namespace JoltPhysics
