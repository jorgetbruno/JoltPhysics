#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>

#include <AzFramework/Physics/Common/PhysicsTypes.h>

#include <JoltPhysics/JoltPhysicsBus.h>
#include <Vehicle/JoltVehicleConfiguration.h>

namespace JoltPhysics
{
    class JoltVehicle;

    //! Vehicle component: turns the entity's dynamic rigid body (the chassis) into a
    //! wheeled vehicle using JPH::VehicleConstraint. Mirrors the PhysXVehicle gem's
    //! component role; controlled at runtime through JoltVehicleRequestBus.
    class JoltVehicleComponent
        : public AZ::Component
        , private AZ::TickBus::Handler
        , private JoltVehicleRequestBus::Handler
    {
    public:
        AZ_COMPONENT(JoltVehicleComponent, "{C9D0E1F2-A3B4-4567-C8D9-E0F1A2B3C4D5}");

        static void Reflect(AZ::ReflectContext* context);

        JoltVehicleComponent() = default;
        ~JoltVehicleComponent() override;

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

    protected:
        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // JoltVehicleRequestBus
        void SetDriverInput(float forward, float right, float brake, float handbrake) override;
        float GetSpeed() const override;
        float GetEngineRpm() const override;
        int GetCurrentGear() const override;

    private:
        void CreateVehicle();
        void DestroyVehicle();

        JoltVehicleConfiguration m_configuration;
        JoltVehicle* m_vehicle = nullptr;
        AzPhysics::SceneHandle m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
    };
} // namespace JoltPhysics
