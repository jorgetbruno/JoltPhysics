#pragma once

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

#include <AzFramework/Physics/Configuration/SystemConfiguration.h>
#include <AzFramework/Physics/Configuration/CollisionConfiguration.h>

namespace JoltPhysics
{
    class JoltSystemConfiguration : public AzPhysics::SystemConfiguration
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltSystemConfiguration, AZ::SystemAllocator);
        AZ_RTTI(JoltSystemConfiguration, "{7C8E3D5F-2A4B-4E9C-8D1F-6A3B5C7E9D2F}", AzPhysics::SystemConfiguration);
        static void Reflect(AZ::ReflectContext* context);

        JoltSystemConfiguration() = default;
        ~JoltSystemConfiguration() override = default;

        static constexpr unsigned int DefaultMaxBodies = 65536;
        static constexpr unsigned int DefaultNumBodyMutexes = 128;
        static constexpr unsigned int DefaultMaxBodyPairs = 65536;
        static constexpr unsigned int DefaultMaxContactConstraints = 16384;
        static constexpr unsigned int DefaultTempAllocatorSize = 256 * 1024 * 1024;

        unsigned int m_maxBodies = DefaultMaxBodies;
        unsigned int m_numBodyMutexes = DefaultNumBodyMutexes;
        unsigned int m_maxBodyPairs = DefaultMaxBodyPairs;
        unsigned int m_maxContactConstraints = DefaultMaxContactConstraints;
        unsigned int m_tempAllocatorSize = DefaultTempAllocatorSize;
        unsigned int m_maxJobThreads = 0;
    };

    class JoltSceneConfiguration : public AzPhysics::SceneConfiguration
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltSceneConfiguration, AZ::SystemAllocator);
        AZ_RTTI(JoltSceneConfiguration, "{A2B4C6D8-E1F3-5A7B-9C2D-4E6F8A1B3C5D}", AzPhysics::SceneConfiguration);
        static void Reflect(AZ::ReflectContext* context);

        JoltSceneConfiguration() = default;
        ~JoltSceneConfiguration() = default;

        int m_collisionSteps = 1;
    };

} // namespace JoltPhysics
