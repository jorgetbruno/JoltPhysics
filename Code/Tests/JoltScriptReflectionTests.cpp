#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/RTTI/BehaviorContext.h>

namespace JoltPhysics
{
    // The gem's own gameplay buses are only reachable from Lua and ScriptCanvas if the
    // components reflect them to the behavior context. Nothing else in the gem fails when
    // a reflection is dropped - the C++ callers keep working - so it is pinned here.
    //
    // The context comes from the test application, which registers the gem's component
    // descriptors exactly as the runtime module does, so this exercises the real path
    // (including the eight joint components sharing one bus).
    class JoltScriptReflectionTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            AZ::ComponentApplicationBus::BroadcastResult(
                m_behaviorContext, &AZ::ComponentApplicationRequests::GetBehaviorContext);
            ASSERT_NE(m_behaviorContext, nullptr) << "No application behavior context";
        }

        const AZ::BehaviorClass* FindClass(const char* name) const
        {
            const auto it = m_behaviorContext->m_classes.find(name);
            return it != m_behaviorContext->m_classes.end() ? it->second : nullptr;
        }

        const AZ::BehaviorEBus* FindBus(const char* name) const
        {
            const auto it = m_behaviorContext->m_ebuses.find(name);
            return it != m_behaviorContext->m_ebuses.end() ? it->second : nullptr;
        }

        void ExpectBusHasEvents(const char* busName, const AZStd::vector<const char*>& eventNames) const
        {
            const AZ::BehaviorEBus* bus = FindBus(busName);
            ASSERT_NE(bus, nullptr) << busName << " is not reflected to script";
            for (const char* eventName : eventNames)
            {
                EXPECT_NE(bus->m_events.find(eventName), bus->m_events.end())
                    << busName << " is missing the event " << eventName;
            }
        }

        AZ::BehaviorContext* m_behaviorContext = nullptr;
    };

    TEST_F(JoltScriptReflectionTests, VehicleConfigClassesExposeTheirHandlingCurves)
    {
        // A scripter reads the config, edits it and writes it back. Unreflected fields
        // survive the round trip untouched - so the omissions did not corrupt anything,
        // they just made the biggest handling knobs unauthorable while DIVERGENCES claimed
        // the configuration was scriptable.
        const AZ::BehaviorClass* wheelClass = FindClass("JoltWheelConfiguration");
        ASSERT_NE(wheelClass, nullptr);
        for (const char* propertyName :
             { "LongitudinalFrictionCurve", "LateralFrictionCurve", "SuspensionSpringMode",
               "SuspensionForcePoint", "EnableSuspensionForcePoint" })
        {
            EXPECT_NE(wheelClass->m_properties.find(propertyName), wheelClass->m_properties.end())
                << "JoltWheelConfiguration is missing the script property " << propertyName;
        }

        const AZ::BehaviorClass* vehicleClass = FindClass("JoltVehicleConfiguration");
        ASSERT_NE(vehicleClass, nullptr);
        EXPECT_NE(vehicleClass->m_properties.find("EngineTorqueCurve"), vehicleClass->m_properties.end());
    }

    TEST_F(JoltScriptReflectionTests, RigidBodyBusIsReflectedWithTheGameplayControlSurface)
    {
        // AzFramework declares Physics::RigidBodyRequestBus but leaves the script binding
        // to whichever backend is running, and in 26.05 the only one that reflects it is
        // the PhysX gem - which a Jolt project disables. Without this the most-used
        // physics script surface is simply absent: no impulses, no velocities, no mass.
        ExpectBusHasEvents("RigidBodyRequestBus",
            { "GetMass", "SetMass", "GetLinearVelocity", "SetLinearVelocity", "GetAngularVelocity",
              "SetAngularVelocity", "ApplyLinearImpulse", "ApplyLinearImpulseAtWorldPoint",
              "ApplyAngularImpulse", "IsKinematic", "SetKinematic", "SetKinematicTarget",
              "IsGravityEnabled", "SetGravityEnabled", "ForceAwake", "ForceAsleep", "GetAabb" });
    }

    TEST_F(JoltScriptReflectionTests, VehicleBusIsReflectedWithItsDrivingAndWheelEvents)
    {
        ExpectBusHasEvents("JoltVehicleRequestBus",
            { "SetDriverInput", "GetSpeed", "GetEngineRpm", "GetCurrentGear", "GetLeanAngle",
              "GetWheelCount", "GetWheelTransform", "GetSuspensionLength", "IsWheelOnGround" });
    }

    TEST_F(JoltScriptReflectionTests, SoftBodyBusIsReflectedWithItsSettingsAndVertexEvents)
    {
        ExpectBusHasEvents("JoltSoftBodyRequestBus",
            { "SetPressure", "GetPressure", "SetLinearDamping", "GetLinearDamping",
              "SetGravityFactor", "GetGravityFactor", "SetNumIterations", "GetNumIterations",
              "SetFriction", "GetFriction", "SetRestitution", "GetRestitution",
              "SetEnabled", "IsEnabled", "GetVertexCount", "GetVertexPosition", "GetWorldBounds",
              // The bulk reads and runtime particle control, without which script-driven
              // rendering and cloth grabbing are C++-only.
              "GetVertexPositions", "GetTriangleIndices",
              "SetVertexPinned", "IsVertexPinned", "SetVertexVelocity", "GetVertexVelocity" });
    }

    TEST_F(JoltScriptReflectionTests, SoftBodyNotificationBusIsReflectedWithAHandler)
    {
        // Without a handler, script can drive a soft body but cannot be told its cloth
        // touched something.
        const AZ::BehaviorEBus* notificationBus = FindBus("JoltSoftBodyNotificationBus");
        ASSERT_NE(notificationBus, nullptr);
        EXPECT_NE(notificationBus->m_createHandler, nullptr);

        // The per-particle contact payload has to be readable from script too.
        AZ::BehaviorContext* behaviorContext = m_behaviorContext;
        EXPECT_NE(behaviorContext->m_classes.find("JoltSoftBodyParticleContact"), behaviorContext->m_classes.end());
    }

    TEST_F(JoltScriptReflectionTests, JointBusIsReflectedWithSingleValueLimitAccessors)
    {
        ExpectBusHasEvents("JoltJointRequestBus",
            { "GetPosition", "GetVelocity", "GetTransform", "SetVelocity", "SetMaximumForce",
              "GetLowerLimit", "GetUpperLimit" });

        // GetLimits returns a pair, which script handles poorly; the two single-value
        // accessors above are reflected in its place.
        const AZ::BehaviorEBus* jointBus = FindBus("JoltJointRequestBus");
        ASSERT_NE(jointBus, nullptr);
        EXPECT_EQ(jointBus->m_events.find("GetLimits"), jointBus->m_events.end());

        // All eight joint components call JoltJointComponentBase::Reflect, and the bus
        // must come out registered exactly once (Internal::ReflectEBusOnce).
        EXPECT_EQ(jointBus->m_events.size(), 7u);
    }

    TEST_F(JoltScriptReflectionTests, JointNotificationBusIsReflectedWithAHandler)
    {
        // Without a handler, script can call into a joint but cannot be told it broke.
        const AZ::BehaviorEBus* notificationBus = FindBus("JoltJointNotificationBus");
        ASSERT_NE(notificationBus, nullptr);
        EXPECT_NE(notificationBus->m_createHandler, nullptr);
    }

    TEST_F(JoltScriptReflectionTests, CharacterGameplayBusIsReflected)
    {
        ExpectBusHasEvents("JoltCharacterGameplayRequestBus", { "IsOnGround", "GetGroundNormal" });
    }
} // namespace JoltPhysics
