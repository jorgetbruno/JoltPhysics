#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/RTTI/AttributeReader.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Script/ScriptContextAttributes.h>
#include <AzCore/std/string/string.h>

#include <Utils/ReflectionUtils.h>

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

        //! The scope a bus was reflected with. Absent means Jolt's - sorry, O3DE's -
        //! default, which is Launcher only.
        AZ::Script::Attributes::ScopeFlags ScopeOf(const char* busName) const
        {
            const AZ::BehaviorEBus* bus = FindBus(busName);
            EXPECT_NE(bus, nullptr) << busName << " is not reflected to script";
            AZ::Script::Attributes::ScopeFlags scope = AZ::Script::Attributes::ScopeFlags::Launcher;
            if (bus != nullptr)
            {
                if (AZ::Attribute* attribute = AZ::FindAttribute(AZ::Script::Attributes::Scope, bus->m_attributes))
                {
                    AZ::AttributeReader(nullptr, attribute).Read<AZ::Script::Attributes::ScopeFlags>(scope);
                }
            }
            return scope;
        }

        //! The module a bus was reflected into, empty when it was given none.
        AZStd::string ModuleOf(const char* busName) const
        {
            const AZ::BehaviorEBus* bus = FindBus(busName);
            EXPECT_NE(bus, nullptr) << busName << " is not reflected to script";
            AZStd::string module;
            if (bus != nullptr)
            {
                if (AZ::Attribute* attribute = AZ::FindAttribute(AZ::Script::Attributes::Module, bus->m_attributes))
                {
                    AZ::AttributeReader(nullptr, attribute).Read<AZStd::string>(module);
                }
            }
            return module;
        }

        //! The scope a reflected class was given; the default hides it from editor python
        //! exactly as it does a bus, which makes its module irrelevant.
        AZ::Script::Attributes::ScopeFlags ClassScopeOf(const char* className) const
        {
            const AZ::BehaviorClass* behaviorClass = FindClass(className);
            EXPECT_NE(behaviorClass, nullptr) << className << " is not reflected to script";
            AZ::Script::Attributes::ScopeFlags scope = AZ::Script::Attributes::ScopeFlags::Launcher;
            if (behaviorClass != nullptr)
            {
                if (AZ::Attribute* attribute =
                        AZ::FindAttribute(AZ::Script::Attributes::Scope, behaviorClass->m_attributes))
                {
                    AZ::AttributeReader(nullptr, attribute).Read<AZ::Script::Attributes::ScopeFlags>(scope);
                }
            }
            return scope;
        }

        //! The module a reflected class was put in, empty when it was given none.
        AZStd::string ClassModuleOf(const char* className) const
        {
            const AZ::BehaviorClass* behaviorClass = FindClass(className);
            EXPECT_NE(behaviorClass, nullptr) << className << " is not reflected to script";
            AZStd::string module;
            if (behaviorClass != nullptr)
            {
                if (AZ::Attribute* attribute =
                        AZ::FindAttribute(AZ::Script::Attributes::Module, behaviorClass->m_attributes))
                {
                    AZ::AttributeReader(nullptr, attribute).Read<AZStd::string>(module);
                }
            }
            return module;
        }

        AZ::BehaviorContext* m_behaviorContext = nullptr;
    };

    TEST_F(JoltScriptReflectionTests, EveryScriptNameIsWhereTheDocumentationSaysItIs)
    {
        // Scope decides whether editor python can see a bus at all; Module decides what it
        // is CALLED once it can. Getting the second wrong fails identically to getting the
        // first wrong - "'NoneType' object is not callable", naming nothing - which is why
        // fixing only the scope left the same symptom and a second round of diagnosis.
        //
        // The fallbacks are not even consistent with each other: an unhomed BUS lands in
        // azlmbr.bus and an unhomed CLASS in azlmbr.default. Neither is guessable, so
        // everything the gem owns is homed in azlmbr.joltphysics - the gem name lowercased,
        // which is the name a scripter tries first.
        for (const char* busName :
             { "JoltVehicleRequestBus", "JoltCharacterGameplayRequestBus", "JoltJointRequestBus",
               "JoltJointNotificationBus", "JoltSoftBodyRequestBus", "JoltSoftBodyNotificationBus" })
        {
            EXPECT_EQ(ModuleOf(busName), Internal::ScriptModule)
                << busName << " is not at azlmbr." << Internal::ScriptModule << "."  << busName;
        }
        for (const char* className :
             { "JoltVehicleConfiguration", "JoltWheelConfiguration", "JoltVehicleAntiRollBar",
               "JoltVehicleDifferential", "JoltSoftBodyParticleContact" })
        {
            EXPECT_EQ(ClassModuleOf(className), Internal::ScriptModule)
                << className << " is not at azlmbr." << Internal::ScriptModule << "." << className;
            // Module without Scope names something editor python cannot see: the class
            // defaults to Launcher exactly as a bus does, and the whole name is absent.
            // Homing these without this line looked correct in the behavior context and
            // was still unreachable from a script.
            EXPECT_EQ(ClassScopeOf(className), AZ::Script::Attributes::ScopeFlags::Common)
                << className << " is Launcher-only, so its module makes no difference";
        }

        // Two exceptions, and both are deliberate: neither type is this gem's to name.
        //
        // RigidBodyRequestBus is AzFramework's - it declares the bus and leaves the binding
        // to whichever backend is running - so it goes beside its siblings in azlmbr.physics,
        // which is also where a script written against PhysX looks for it.
        EXPECT_EQ(ModuleOf("RigidBodyRequestBus"), "physics")
            << "the rigid body bus is not at azlmbr.physics.RigidBodyRequestBus";

        // EntityComponentIdPair is AzCore's, and the gem reflects it only for the case
        // where nothing else does - guarded, so another registration wins harmlessly. In
        // the editor the engine's own wins, and the name a script uses is
        // azlmbr.entity.EntityComponentIdPair (measured: constructible there and nowhere
        // else). Homing THIS copy would make the name depend on who won that race, so it
        // carries no module; this pins the gem's own registration, which is what the
        // test application sees.
        EXPECT_EQ(ClassModuleOf("EntityComponentIdPair"), "")
            << "EntityComponentIdPair was homed; its name would then depend on which gem "
               "reflected it first, since that registration is guarded";
        // It still has to be VISIBLE, though: it exists so a script can construct a joint
        // bus address, and without it JoltJointRequestBus is unusable from automation no
        // matter how well the bus itself is reflected.
        EXPECT_EQ(ClassScopeOf("EntityComponentIdPair"), AZ::Script::Attributes::ScopeFlags::Common)
            << "EntityComponentIdPair is Launcher-only, so no script can address a joint";
    }

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
