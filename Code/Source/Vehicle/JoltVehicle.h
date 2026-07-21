#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>

namespace JPH
{
    class WheeledVehicleController;
}

namespace JoltPhysics
{
    class JoltScene;
    struct JoltVehicleConfiguration;

    //! Owns a JPH::VehicleConstraint (wheeled-vehicle controller) attached to an
    //! existing chassis rigid body. The chassis remains a normal dynamic rigid body
    //! in the scene; this class only drives the wheels/engine.
    class JoltVehicle
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltVehicle, AZ::SystemAllocator);

        JoltVehicle(const JoltVehicleConfiguration& configuration, JoltScene* scene, JPH::Body* chassisBody);
        ~JoltVehicle();

        bool IsValid() const { return m_constraint != nullptr; }

        //! forward: throttle [-1..1], right: steering [-1..1], brake/handbrake [0..1].
        void SetDriverInput(float forward, float right, float brake, float handbrake);

        //! Chassis speed along its forward axis (m/s).
        float GetSpeed() const;
        float GetEngineRpm() const;
        int GetCurrentGear() const;

        JPH::VehicleConstraint* GetConstraint() const { return m_constraint; }

    private:
        JoltScene* m_scene = nullptr;
        JPH::Ref<JPH::VehicleConstraint> m_constraint;
        JPH::WheeledVehicleController* m_controller = nullptr;
        JPH::Body* m_chassisBody = nullptr;
    };
} // namespace JoltPhysics
