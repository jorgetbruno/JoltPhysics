#pragma once

#include <AzCore/Interface/Interface.h>
#include <AzCore/Math/Aabb.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/string/string.h>

#include <AzFramework/Physics/WindBus.h>

namespace JoltPhysics
{
    class JoltForceRegionComponent;

    //! Force regions announce themselves so the wind interface can answer cheaply.
    void RegisterForceRegionForWind(JoltForceRegionComponent* region);
    void UnregisterForceRegionForWind(JoltForceRegionComponent* region);

    //! Supplies the engine's wind interface from wind-tagged force regions.
    //!
    //! `Physics::WindRequests` is declared by AzFramework but implemented by whichever
    //! physics gem is running, and in 26.05 the only implementation ships with PhysX -
    //! which a Jolt project has to disable. Without this, everything that reads wind
    //! (cloth, vegetation, the sibling JoltHair gem) sees a null interface and no wind at
    //! all, with nothing to explain why.
    //!
    //! Tagging follows PhysX's model so content transfers: a force region carrying the
    //! configured global tag contributes wind everywhere, one carrying the local tag
    //! contributes wind inside its own bounds.
    class JoltWindProvider final
        : public AZ::Interface<Physics::WindRequests>::Registrar
        , private Physics::WindRequestsBus::Handler
    {
    public:
        JoltWindProvider();
        ~JoltWindProvider() override;

        // Physics::WindRequests
        AZ::Vector3 GetGlobalWind() const override;
        AZ::Vector3 GetWind(const AZ::Vector3& worldPosition) const override;
        AZ::Vector3 GetWind(const AZ::Aabb& aabb) const override;

        //! Tags the provider matches force regions against. Taken from the Jolt
        //! configuration so a project can rename them, exactly as PhysX allows.
        void SetWindTags(const AZStd::string& globalWindTag, const AZStd::string& localWindTag);

    private:
        //! Sums the wind of every region whose tag matches and whose bounds accept the
        //! sample, which is the shape both GetWind overloads share.
        template<typename BoundsTest>
        AZ::Vector3 AccumulateLocalWind(const BoundsTest& acceptsBounds) const;

        AZStd::string m_globalWindTag = "global_wind";
        AZStd::string m_localWindTag = "wind";
    };
} // namespace JoltPhysics
