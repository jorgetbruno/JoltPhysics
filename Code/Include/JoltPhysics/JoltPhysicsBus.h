#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/containers/span.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/utility/pair.h>

#include <AzFramework/Physics/Collision/CollisionGroups.h>
#include <AzFramework/Physics/Collision/CollisionLayers.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>

namespace JPH
{
    class PhysicsSystem;
}

namespace JoltPhysics
{
    class JoltPhysicsRequests
    {
    public:
        AZ_RTTI(JoltPhysicsRequests, "{E9F7A5B3-4C2D-4E8F-9A1B-3C5D7E8F9A2B}");
        virtual ~JoltPhysicsRequests() = default;
    };

    class JoltPhysicsBusTraits
        : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
    };

    using JoltPhysicsRequestBus = AZ::EBus<JoltPhysicsRequests, JoltPhysicsBusTraits>;
    using JoltPhysicsInterface = AZ::Interface<JoltPhysicsRequests>;

    //! Runtime control of joints (mirrors the PhysX gem's JointRequestBus surface so
    //! gameplay code can read joint state and drive motors per joint component).
    class JoltJointRequests
        : public AZ::ComponentBus
    {
    public:
        virtual ~JoltJointRequests() = default;

        //! Current joint position: hinge angle (radians) or slider displacement (meters).
        virtual float GetPosition() const = 0;
        //! Current joint velocity: hinge angular velocity (rad/s) or slider speed (m/s).
        virtual float GetVelocity() const = 0;
        //! The joint frame in world space.
        virtual AZ::Transform GetTransform() const = 0;
        //! Drives the joint motor at the given velocity (hinge: rad/s, slider: m/s).
        virtual void SetVelocity(float velocity) = 0;
        //! Sets the maximum force/torque the motor may apply.
        virtual void SetMaximumForce(float force) = 0;
        //! The configured limits (hinge: radians, slider: meters).
        virtual AZStd::pair<float, float> GetLimits() const = 0;

        //! The same limits one at a time. Script has no useful handling for a pair, so
        //! these are what the bus reflects; C++ callers can keep using GetLimits.
        float GetLowerLimit() const
        {
            return GetLimits().first;
        }
        float GetUpperLimit() const
        {
            return GetLimits().second;
        }
    };

    using JoltJointRequestBus = AZ::EBus<JoltJointRequests>;

    //! Notifications from a joint component, addressed by its entity id.
    class JoltJointNotifications
        : public AZ::ComponentBus
    {
    public:
        virtual ~JoltJointNotifications() = default;

        //! The joint broke: its reaction exceeded the configured break force/torque and
        //! it has been removed from the scene. It will not be recreated.
        virtual void OnJointBroken() = 0;
    };

    using JoltJointNotificationBus = AZ::EBus<JoltJointNotifications>;

    //! Runtime control of vehicles (AzPhysics has no vehicle interfaces in O3DE 26.05,
    //! so the vehicle surface lives on this gem's own bus).
    class JoltVehicleRequests
        : public AZ::ComponentBus
    {
    public:
        virtual ~JoltVehicleRequests() = default;

        //! Applies driver input: throttle [-1..1], steering [-1..1], brake [0..1], handbrake [0..1].
        //! A tracked vehicle converts the steering into a left/right track speed ratio
        //! (full lock pivots it on the spot) and folds the handbrake into the brake.
        virtual void SetDriverInput(float forward, float right, float brake, float handbrake) = 0;
        //! Chassis speed along its forward axis (m/s).
        virtual float GetSpeed() const = 0;
        virtual float GetEngineRpm() const = 0;
        virtual int GetCurrentGear() const = 0;
        //! How far a motorcycle is leaned over, in radians (0 for the other vehicle types).
        virtual float GetLeanAngle() const = 0;

        //! Wheels the vehicle ended up with: the authored count, or the vehicle type's
        //! default layout when none were authored.
        virtual AZ::u32 GetWheelCount() const = 0;
        //! World transform of a wheel, for driving a visual wheel: it carries the
        //! suspension position, the steer angle and the rolling of the tyre. Identity
        //! for an out-of-range index.
        virtual AZ::Transform GetWheelTransform(AZ::u32 wheelIndex) const = 0;
        //! Current suspension extension in metres, for driving a visual suspension.
        virtual float GetSuspensionLength(AZ::u32 wheelIndex) const = 0;
        //! Whether the wheel found ground on the last step; one in the air neither
        //! drives nor steers.
        virtual bool IsWheelOnGround(AZ::u32 wheelIndex) const = 0;
    };

    using JoltVehicleRequestBus = AZ::EBus<JoltVehicleRequests>;

    //! System-level access to the Jolt backend, for gems that extend it with features
    //! Jolt provides but this gem does not wrap (buoyancy, soft bodies and so on).
    //!
    //! Going through this bus rather than casting a scene's native pointer is what makes
    //! that safe: the pointer is only handed out when the scene really is a Jolt scene,
    //! so an extension gem does no harm in a project running a different physics backend.
    class JoltPhysicsSystemRequests
        : public AZ::EBusTraits
    {
    public:
        AZ_RTTI(JoltPhysicsSystemRequests, "{F1E2D3C4-B5A6-4978-8A9B-0C1D2E3F4A5B}");

        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;

        virtual ~JoltPhysicsSystemRequests() = default;

        //! The JPH::PhysicsSystem backing the given scene, or null when that scene is not
        //! one of this gem's. The returned system is owned by the scene: it stays valid
        //! until the scene is removed.
        virtual JPH::PhysicsSystem* GetNativePhysicsSystem(AzPhysics::SceneHandle sceneHandle) = 0;

        //! Snapshots a scene's complete simulation state - body positions and
        //! velocities (including soft body vertices), contacts, constraints and the
        //! characters - into an opaque blob, appended to outState. For rollback and
        //! replay, not persistence: the blob only restores into a scene with the same
        //! bodies, joints and characters in the same slots, and a deterministic replay
        //! additionally requires the same binary. Returns false for an unknown scene.
        virtual bool SaveSimulationState(AzPhysics::SceneHandle sceneHandle, AZStd::vector<AZ::u8>& outState) = 0;

        //! Restores a snapshot taken by SaveSimulationState into the same scene.
        //! Returns false when the blob does not match the scene's current contents -
        //! and the scene may then be PARTIALLY restored, since state is applied as it
        //! is read: treat false as "restore from a good snapshot or rebuild", not
        //! "carry on". On success entity transforms sync immediately.
        virtual bool RestoreSimulationState(AzPhysics::SceneHandle sceneHandle, AZStd::span<const AZ::u8> state) = 0;

        //! The Jolt object layer for an AzPhysics collision layer and group, for an
        //! extension gem that creates its own bodies (soft bodies and the like).
        //!
        //! Object layers cannot be chosen by an extension gem on its own: this gem gives
        //! every distinct (layer, group, motion class) combination its own Jolt object
        //! layer, because AzPhysics filtering is per body and not a function of two layer
        //! indices. A body created with a layer this gem never registered would be
        //! filtered against the wrong entry, so ask for one here rather than passing a
        //! literal. Must be called from the main thread, as it may register a new
        //! combination.
        //!
        //! Returned as AZ::u32 rather than JPH::ObjectLayer so this header needs no Jolt
        //! include; this gem builds Jolt with OBJECT_LAYER_BITS 32, so the two match.
        virtual AZ::u32 AcquireObjectLayer(
            const AzPhysics::CollisionLayer& collisionLayer,
            const AzPhysics::CollisionGroups::Id& collisionGroupId,
            bool isMoving) = 0;
    };

    using JoltPhysicsSystemRequestBus = AZ::EBus<JoltPhysicsSystemRequests>;

} // namespace JoltPhysics
