#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/string/string.h>

#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>

#include <ForceRegion/JoltForceRegionForces.h>

namespace JoltPhysics
{
    //! Applies forces to every rigid body inside a trigger collider: fans, conveyors,
    //! updrafts, water current, gravity wells.
    //!
    //! Mirrors PhysX's ForceRegionComponent, which a migrating project cannot otherwise
    //! bring across - the entity keeps its data and there is nothing to attach it to.
    //!
    //! Bodies are tracked through the scene's trigger events rather than re-queried each
    //! frame, so a region costs the enter/exit it actually sees plus one impulse per
    //! occupant per tick.
    class JoltForceRegionComponent
        : public AZ::Component
        , protected AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(JoltForceRegionComponent, "{8C1D3E5F-2A7B-4E9C-A0D6-1B4F7E2C8A93}");

        static void Reflect(AZ::ReflectContext* context);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        JoltForceRegionComponent() = default;
        JoltForceRegionComponent(const JoltForceRegion& forceRegion, const AZStd::string& windTag);

        const JoltForceRegion& GetForceRegion() const
        {
            return m_forceRegion;
        }

        //! Wind tag this region is published under, empty when it is not wind. Compared
        //! against the tags in the Jolt configuration to decide global versus local wind.
        const AZStd::string& GetWindTag() const
        {
            return m_windTag;
        }

        //! Net force this region would apply to a body with the given state, in newtons.
        //! Public so the behaviour is testable without a scene full of entities.
        AZ::Vector3 CalculateNetForce(const JoltForceRegionEntityParams& entity) const;

    protected:
        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        int GetTickOrder() override;

    private:
        //! State of the region entity itself, resolved once per tick rather than per body.
        JoltForceRegionParams BuildRegionParams() const;

        JoltForceRegion m_forceRegion;
        AZStd::string m_windTag;

        AzPhysics::SceneHandle m_sceneHandle = AzPhysics::InvalidSceneHandle;
        AzPhysics::SimulatedBodyHandle m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;

        //! Bodies currently inside the region, from the trigger events.
        AZStd::vector<AzPhysics::SimulatedBodyHandle> m_bodiesInRegion;

        AzPhysics::SceneEvents::OnSceneTriggersEvent::Handler m_triggerHandler;
    };
} // namespace JoltPhysics
