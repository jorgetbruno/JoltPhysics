#include <ForceRegion/JoltWindProvider.h>

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>

#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBodyEvents.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <AzFramework/Physics/PhysicsSystem.h>

#include <ForceRegion/JoltForceRegionComponent.h>

namespace JoltPhysics
{
    namespace
    {
        //! Every activated force region, so the wind interface can answer without walking
        //! the entity system. Regions are few and long-lived, and wind is asked for far
        //! more often than regions appear.
        AZStd::vector<JoltForceRegionComponent*>& GetRegisteredForceRegions()
        {
            static AZStd::vector<JoltForceRegionComponent*> s_regions;
            return s_regions;
        }
    }

    void RegisterForceRegionForWind(JoltForceRegionComponent* region)
    {
        if (region != nullptr)
        {
            GetRegisteredForceRegions().push_back(region);
        }
    }

    void UnregisterForceRegionForWind(JoltForceRegionComponent* region)
    {
        auto& regions = GetRegisteredForceRegions();
        regions.erase(AZStd::remove(regions.begin(), regions.end(), region), regions.end());
    }

    JoltWindProvider::JoltWindProvider()
    {
        Physics::WindRequestsBus::Handler::BusConnect();
    }

    JoltWindProvider::~JoltWindProvider()
    {
        Physics::WindRequestsBus::Handler::BusDisconnect();
    }

    void JoltWindProvider::SetWindTags(const AZStd::string& globalWindTag, const AZStd::string& localWindTag)
    {
        m_globalWindTag = globalWindTag;
        m_localWindTag = localWindTag;
    }

    AZ::Vector3 JoltWindProvider::GetGlobalWind() const
    {
        AZ::Vector3 wind = AZ::Vector3::CreateZero();
        for (const JoltForceRegionComponent* region : GetRegisteredForceRegions())
        {
            if (region != nullptr && !m_globalWindTag.empty() && region->GetWindTag() == m_globalWindTag)
            {
                wind += region->GetForceRegion().GetWindVelocity();
            }
        }
        return wind;
    }

    template<typename BoundsTest>
    AZ::Vector3 JoltWindProvider::AccumulateLocalWind(const BoundsTest& acceptsBounds) const
    {
        // Global wind blows everywhere, so it is the baseline every local sample adds to.
        AZ::Vector3 wind = GetGlobalWind();
        if (m_localWindTag.empty())
        {
            return wind;
        }

        for (const JoltForceRegionComponent* region : GetRegisteredForceRegions())
        {
            if (region == nullptr || region->GetWindTag() != m_localWindTag)
            {
                continue;
            }

            // The region's extent is its trigger body's bounds - the volume the author
            // drew - so wind stops exactly where the region does.
            AZ::Aabb regionBounds = AZ::Aabb::CreateNull();
            Physics::RigidBodyRequestBus::EventResult(
                regionBounds, region->GetEntityId(), &Physics::RigidBodyRequests::GetAabb);
            if (regionBounds.IsValid() && acceptsBounds(regionBounds))
            {
                wind += region->GetForceRegion().GetWindVelocity();
            }
        }
        return wind;
    }

    AZ::Vector3 JoltWindProvider::GetWind(const AZ::Vector3& worldPosition) const
    {
        return AccumulateLocalWind(
            [&worldPosition](const AZ::Aabb& bounds)
            {
                return bounds.Contains(worldPosition);
            });
    }

    AZ::Vector3 JoltWindProvider::GetWind(const AZ::Aabb& aabb) const
    {
        return AccumulateLocalWind(
            [&aabb](const AZ::Aabb& bounds)
            {
                return bounds.Overlaps(aabb);
            });
    }
} // namespace JoltPhysics
