# Get the platform specific folder for the current platform
o3de_pal_dir(pal_dir ${CMAKE_CURRENT_LIST_DIR}/Platform/${PAL_PLATFORM_NAME} "${gem_restricted_path}" "${gem_path}" "${gem_parent_relative_path}")

include(${pal_dir}/PAL_${PAL_PLATFORM_NAME_LOWERCASE}.cmake)

if(NOT PAL_TRAIT_JOLTPHYSICS_SUPPORTED)
    return()
endif()

# JoltPhysics version configuration
set(JOLTPHYSICS_GIT_REPO "https://github.com/jrouwe/JoltPhysics")
set(JOLTPHYSICS_GIT_TAG "v5.6.0")

option(JOLTPHYSICS_USE_EXTERNAL "Use externally provided JoltPhysics library instead of FetchContent" OFF)
option(JOLTPHYSICS_EXTERNAL_PATH "Path to external JoltPhysics installation" "")

if(NOT JOLTPHYSICS_USE_EXTERNAL)
    include(FetchContent)
    FetchContent_Declare(
        JoltPhysics
        GIT_REPOSITORY ${JOLTPHYSICS_GIT_REPO}
        GIT_TAG ${JOLTPHYSICS_GIT_TAG}
        SOURCE_SUBDIR "Build"
    )

    message(STATUS "JoltPhysics Gem: Fetching ${JOLTPHYSICS_GIT_REPO} @ ${JOLTPHYSICS_GIT_TAG} (License: MIT)")

    set(CPP_RTTI_ENABLED ON)
    set(OBJECT_LAYER_BITS 32)
    set(CROSS_PLATFORM_DETERMINISTIC OFF)
    set(USE_ASSERTS ON)
    set(USE_STATIC_MSVC_RUNTIME_LIBRARY OFF)
    set(INTERPROCEDURAL_OPTIMIZATION OFF)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF)
    set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

    get_property(
        compile_options
        DIRECTORY
        PROPERTY COMPILE_OPTIONS
    )

    set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)

    set(SAVED_CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    if(MSVC)
        string(REGEX REPLACE "/diagnostics:[a-z]+" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    endif()

    FetchContent_MakeAvailable(JoltPhysics)

    set(CMAKE_CXX_FLAGS "${SAVED_CMAKE_CXX_FLAGS}")

    target_compile_definitions(Jolt PUBLIC "$<$<CONFIG:RelWithDebInfo>:JPH_PROFILE_ENABLED;JPH_DEBUG_RENDERER>")
    target_compile_definitions(Jolt PUBLIC "$<$<CONFIG:profile>:JPH_PROFILE_ENABLED;JPH_DEBUG_RENDERER>")
    target_compile_definitions(Jolt PUBLIC "$<$<CONFIG:Debug>:JPH_PROFILE_ENABLED;JPH_DEBUG_RENDERER>")

    if(FLOATING_POINT_EXCEPTIONS_ENABLED AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
        target_compile_definitions(Jolt PUBLIC "$<$<CONFIG:RelWithDebInfo>:JPH_FLOATING_POINT_EXCEPTIONS_ENABLED>")
        target_compile_definitions(Jolt PUBLIC "$<$<CONFIG:Debug>:JPH_FLOATING_POINT_EXCEPTIONS_ENABLED>")
    endif()

    set_property(
        DIRECTORY
        PROPERTY COMPILE_OPTIONS ${compile_options}
    )
    unset(compile_options)

    set(CMAKE_WARN_DEPRECATED ON CACHE BOOL "" FORCE)
else()
    if(NOT JOLTPHYSICS_EXTERNAL_PATH)
        message(FATAL_ERROR "JOLTPHYSICS_EXTERNAL_PATH must be set when using JOLTPHYSICS_USE_EXTERNAL")
    endif()
    find_package(Jolt REQUIRED PATHS ${JOLTPHYSICS_EXTERNAL_PATH})
endif()

# API target - public interface for other gems
ly_add_target(
    NAME ${gem_name}.API INTERFACE
    NAMESPACE Gem
    FILES_CMAKE
        joltphysics_api_files.cmake
        ${pal_dir}/joltphysics_api_files.cmake
    INCLUDE_DIRECTORIES
        INTERFACE
            Include
    BUILD_DEPENDENCIES
        INTERFACE
            AZ::AzCore
            AZ::AzFramework
            Jolt::Jolt
)

# Private Object target - static library with implementation
ly_add_target(
    NAME ${gem_name}.Private.Object STATIC
    NAMESPACE Gem
    FILES_CMAKE
        joltphysics_private_files.cmake
        ${pal_dir}/joltphysics_private_files.cmake
    COMPILE_DEFINITIONS
        PUBLIC
            JOLT_SETREG_GEM_NAME="JoltPhysics"
            $<$<NOT:$<CONFIG:Release>>:AZ_PHYSICS_DEBUG_ENABLED>
    TARGET_PROPERTIES
        O3DE_PRIVATE_TARGET TRUE
    INCLUDE_DIRECTORIES
        PRIVATE
            Include
            Source
    BUILD_DEPENDENCIES
        PUBLIC
            AZ::AzCore
            AZ::AzFramework
            Legacy::CryCommon
            Jolt::Jolt
        PRIVATE
            Gem::LmbrCentral.API
)

# Main Gem module
ly_add_target(
    NAME ${gem_name} ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
    NAMESPACE Gem
    FILES_CMAKE
        joltphysics_shared_files.cmake
        ${pal_dir}/joltphysics_shared_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
    BUILD_DEPENDENCIES
        PUBLIC
            Gem::${gem_name}.API
            Gem::LmbrCentral.API
        PRIVATE
            Gem::${gem_name}.Private.Object
    RUNTIME_DEPENDENCIES
        Gem::LmbrCentral
)

ly_add_source_properties(
    SOURCES
        Source/JoltPhysicsModule.cpp
        Source/JoltPhysicsModule.h
    PROPERTY COMPILE_DEFINITIONS
        VALUES
            O3DE_GEM_NAME=${gem_name}
            O3DE_GEM_VERSION=${gem_version}
)

# Create aliases for different build variants
ly_create_alias(NAME ${gem_name}.Clients NAMESPACE Gem TARGETS Gem::${gem_name})
ly_create_alias(NAME ${gem_name}.Servers NAMESPACE Gem TARGETS Gem::${gem_name})
ly_create_alias(NAME ${gem_name}.Unified NAMESPACE Gem TARGETS Gem::${gem_name})

ly_create_alias(NAME ${gem_name}.Clients.API NAMESPACE Gem TARGETS Gem::${gem_name}.API)
ly_create_alias(NAME ${gem_name}.Servers.API NAMESPACE Gem TARGETS Gem::${gem_name}.API)
ly_create_alias(NAME ${gem_name}.Unified.API NAMESPACE Gem TARGETS Gem::${gem_name}.API)

o3de_add_variant_dependencies_for_gem_dependencies(GEM_NAME ${gem_name} VARIANTS Clients Servers Unified)

# Editor/Tools targets
if(PAL_TRAIT_BUILD_HOST_TOOLS)
    # v-hacd (VHACD convex decomposition), editor-only. The engine's 3rdParty package is
    # wired for the PhysX gem and its SDK-side Find module only creates a stub target,
    # so - like Jolt above - fetch the sources at the same commit that package was built
    # from and build the small static lib here. Upstream's own CMake cannot run from a
    # subdirectory (it mutates global flags and probes OpenCL/OpenMP), so this target
    # lists the sources directly; the SOURCE_SUBDIR below does not exist, which makes
    # FetchContent_MakeAvailable populate the sources without adding upstream's build.
    include(FetchContent)
    FetchContent_Declare(
        vhacd_src
        GIT_REPOSITORY https://github.com/kmammou/v-hacd.git
        GIT_TAG 1a49edf29c69039df15286181f2f27e17ceb9aef # v4.1.0, same commit as the engine's 3rdParty package (License: BSD-3-Clause)
        SOURCE_SUBDIR "no-such-dir"
    )
    message(STATUS "JoltPhysics Gem: Fetching v-hacd @ 1a49edf (License: BSD-3-Clause)")
    FetchContent_MakeAvailable(vhacd_src)

    set(vhacd_lib_dir "${vhacd_src_SOURCE_DIR}/src/VHACD_Lib")
    add_library(joltphysics_vhacd STATIC
        ${vhacd_lib_dir}/src/VHACD.cpp
        ${vhacd_lib_dir}/src/VHACD-ASYNC.cpp
        ${vhacd_lib_dir}/src/btAlignedAllocator.cpp
        ${vhacd_lib_dir}/src/btConvexHullComputer.cpp
        ${vhacd_lib_dir}/src/vhacdICHull.cpp
        ${vhacd_lib_dir}/src/vhacdManifoldMesh.cpp
        ${vhacd_lib_dir}/src/vhacdMesh.cpp
        ${vhacd_lib_dir}/src/vhacdRaycastMesh.cpp
        ${vhacd_lib_dir}/src/vhacdVolume.cpp
        ${vhacd_lib_dir}/src/FloatMath.cpp
    )
    target_include_directories(joltphysics_vhacd SYSTEM
        PUBLIC ${vhacd_lib_dir}/public
        PRIVATE ${vhacd_lib_dir}/inc
    )
    # Third-party C++ at O3DE warning levels is noisy; silence it on its own target.
    if(MSVC)
        target_compile_options(joltphysics_vhacd PRIVATE /W0)
    else()
        target_compile_options(joltphysics_vhacd PRIVATE -w)
    endif()

    ly_add_target(
        NAME ${gem_name}.Editor.API INTERFACE
        NAMESPACE Gem
        FILES_CMAKE
            joltphysics_editor_api_files.cmake
            ${pal_dir}/joltphysics_editor_api_files.cmake
        INCLUDE_DIRECTORIES
            INTERFACE
                Include
        BUILD_DEPENDENCIES
            INTERFACE
                AZ::AzToolsFramework
                Jolt::Jolt
    )

    ly_add_target(
        NAME ${gem_name}.Editor.Private.Object STATIC
        NAMESPACE Gem
        AUTOMOC
        FILES_CMAKE
            joltphysics_editor_private_files.cmake
        COMPILE_DEFINITIONS
            PUBLIC
                JOLT_EDITOR
        TARGET_PROPERTIES
            O3DE_PRIVATE_TARGET TRUE
        INCLUDE_DIRECTORIES
            PRIVATE
                Include
                Source
        BUILD_DEPENDENCIES
            PUBLIC
                AZ::AzCore
                AZ::AzFramework
                AZ::AzToolsFramework
                Gem::LmbrCentral.Editor
                ${gem_name}.Private.Object
                Jolt::Jolt
                joltphysics_vhacd # editor-time convex decomposition (VHACD); runtime stays free of it
    )

    ly_add_target(
        NAME ${gem_name}.Editor GEM_MODULE
        NAMESPACE Gem
        AUTOMOC
        FILES_CMAKE
            joltphysics_editor_shared_files.cmake
        INCLUDE_DIRECTORIES
            PRIVATE
                Source
            PUBLIC
                Include
        BUILD_DEPENDENCIES
            PUBLIC
                Gem::${gem_name}.Editor.API
            PRIVATE
                Gem::${gem_name}.Editor.Private.Object
    )

    ly_create_alias(NAME ${gem_name}.Tools NAMESPACE Gem TARGETS Gem::${gem_name}.Editor)
    ly_create_alias(NAME ${gem_name}.Builders NAMESPACE Gem TARGETS Gem::${gem_name}.Editor)

    ly_create_alias(NAME ${gem_name}.Tools.API NAMESPACE Gem TARGETS Gem::${gem_name}.Editor.API)
    ly_create_alias(NAME ${gem_name}.Builders.API NAMESPACE Gem TARGETS Gem::${gem_name}.Editor.API)

    o3de_add_variant_dependencies_for_gem_dependencies(GEM_NAME ${gem_name} VARIANTS Tools Builders)
endif()

# Tests
if(PAL_TRAIT_BUILD_TESTS_SUPPORTED)
    if(PAL_TRAIT_JOLTPHYSICS_TEST_SUPPORTED)
        ly_add_target(
            NAME ${gem_name}.Tests ${PAL_TRAIT_TEST_TARGET_TYPE}
            NAMESPACE Gem
            FILES_CMAKE
                joltphysics_tests_files.cmake
            INCLUDE_DIRECTORIES
                PRIVATE
                    Tests
                    Source
                    Include
            BUILD_DEPENDENCIES
                PRIVATE
                    AZ::AzTest
                    AZ::AzFramework
                    Gem::${gem_name}.Private.Object
        )

        ly_add_googletest(
            NAME Gem::${gem_name}.Tests
        )
    endif()

    if(PAL_TRAIT_BUILD_HOST_TOOLS)
        if(PAL_TRAIT_JOLTPHYSICS_EDITOR_TEST_SUPPORTED)
            ly_add_target(
                NAME ${gem_name}.Editor.Tests ${PAL_TRAIT_TEST_TARGET_TYPE}
                NAMESPACE Gem
                FILES_CMAKE
                    joltphysics_editor_tests_files.cmake
                INCLUDE_DIRECTORIES
                    PRIVATE
                        Tests
                        Source
                        Include
                BUILD_DEPENDENCIES
                    PRIVATE
                        AZ::AzTest
                        Gem::${gem_name}.Editor.Private.Object
            )

            ly_add_googletest(
                NAME Gem::${gem_name}.Editor.Tests
            )
        endif()
    endif()
endif()
