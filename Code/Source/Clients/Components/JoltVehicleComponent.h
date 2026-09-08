#pragma once

#include <AzCore/std/containers/array.h>
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

        //! Serialized identifier for the DPE inspector (empty on plain AZ::Component).
        AZStd::string GetSerializedIdentifier() const override;

        JoltVehicleComponent() = default;
        ~JoltVehicleComponent() override;

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        JoltVehicleConfiguration& GetConfiguration()
        {
            return m_configuration;
        }
        const JoltVehicleConfiguration& GetConfiguration() const
        {
            return m_configuration;
        }

    protected:
        // AZ::Component
        void OnAfterEntitySet() override;
        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // JoltVehicleRequestBus
        void SetDriverInput(float forward, float right, float brake, float handbrake) override;
        void SetForwardInput(float forward) override;
        void SetSteeringInput(float right) override;
        void SetBrakeInput(float brake) override;
        void SetHandBrakeInput(float handbrake) override;
        float GetSpeed() const override;
        float GetEngineRpm() const override;
        int GetCurrentGear() const override;
        void SetGear(int gear) override;
        void SetTransmissionAutomatic(bool automatic) override;
        bool IsTransmissionAutomatic() const override;
        float GetLeanAngle() const override;
        void SetLeanControllerEnabled(bool enabled) override;
        void SetLeanSteeringLimitEnabled(bool enabled) override;
        AZ::u32 GetWheelCount() const override;
        AZ::Transform GetWheelTransform(AZ::u32 wheelIndex) const override;
        float GetSuspensionLength(AZ::u32 wheelIndex) const override;
        bool IsWheelOnGround(AZ::u32 wheelIndex) const override;
        float GetWheelAngularVelocity(AZ::u32 wheelIndex) const override;
        float GetWheelSteerAngle(AZ::u32 wheelIndex) const override;
        float GetWheelLongitudinalSlip(AZ::u32 wheelIndex) const override;
        float GetWheelLateralSlip(AZ::u32 wheelIndex) const override;
        AZ::Vector3 GetWheelContactPoint(AZ::u32 wheelIndex) const override;
        AZ::Vector3 GetWheelContactNormal(AZ::u32 wheelIndex) const override;
        bool IsWheelSuspensionBottomedOut(AZ::u32 wheelIndex) const override;
        void OverrideVehicleGravity(const AZ::Vector3& gravity) override;
        void ResetVehicleGravityOverride() override;
        void RecreateVehicle() override;
        JoltVehicleConfiguration GetVehicleConfiguration() const override;
        void SetVehicleConfiguration(const JoltVehicleConfiguration& configuration) override;
        void SetCombineFriction(CombineFrictionFunction combineFriction) override;
        void SetTireMaxImpulse(TireMaxImpulseFunction tireMaxImpulse) override;

    private:
        void CreateVehicle();
        void DestroyVehicle();

        JoltVehicleConfiguration m_configuration;
        AZStd::string m_serializedIdentifier;

        JoltVehicle* m_vehicle = nullptr;

        //! Driver input carried across a RecreateVehicle: forward, steering, brake,
        //! handbrake. A configuration change rebuilds the vehicle, and the driver did not
        //! ask for the throttle to drop while that happened.
        AZStd::array<float, 4> m_pendingDriverInput = { 0.0f, 0.0f, 0.0f, 0.0f };
        bool m_hasPendingDriverInput = false;

        //! The per-wheel callbacks, kept here rather than only on the vehicle. They are
        //! set on the constraint, and RecreateVehicle builds a new one - a car that
        //! quietly reverted to Jolt's tyre model because something edited its
        //! configuration would be a genuinely hard fault to account for.
        CombineFrictionFunction m_combineFriction;
        TireMaxImpulseFunction m_tireMaxImpulse;
        AzPhysics::SceneHandle m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
    };
} // namespace JoltPhysics
