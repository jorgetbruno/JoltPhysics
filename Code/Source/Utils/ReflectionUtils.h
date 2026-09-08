#pragma once

#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace JoltPhysics
{
    namespace Internal
    {
        //! The Python module the gem's own script surface lives in: editor Python reaches
        //! it as azlmbr.joltphysics.<name>, and Lua and Script Canvas are unaffected.
        //!
        //! Every EBus and class the gem OWNS carries this. Without it a bus lands in
        //! azlmbr.bus and a class in azlmbr.default - two different fallbacks, neither
        //! guessable - and calling the wrong name fails as "'NoneType' object is not
        //! callable", naming nothing, which is indistinguishable from a bus that was
        //! never reflected at all.
        //!
        //! Two things deliberately do NOT carry it, because they are not this gem's to
        //! name: Physics::RigidBodyRequestBus, which is AzFramework's and is homed in
        //! "physics" beside its siblings, and AZ::EntityComponentIdPair, which is
        //! AzCore's and whose registration here is guarded - homing it would make its
        //! name depend on which gem won the race to reflect it.
        inline constexpr const char* ScriptModule = "joltphysics";

        //! Runs the callback with the behavior context, but only when no EBus of that name
        //! has been reflected yet. All eight joint components share one request bus and
        //! each of them calls JoltJointComponentBase::Reflect, so without this the second
        //! one through would register the bus again.
        template<typename ReflectFn>
        void ReflectEBusOnce(AZ::ReflectContext* context, const char* ebusName, ReflectFn&& reflectFn)
        {
            if (auto* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
            {
                if (behaviorContext->m_ebuses.find(ebusName) == behaviorContext->m_ebuses.end())
                {
                    reflectFn(behaviorContext);
                }
            }
        }

        //! Reflects T with the serialize context only if it has not been reflected yet.
        //! Used for AzPhysics/AzFramework configuration classes that may already have
        //! been reflected by another module in the application.
        template<typename T>
        void ReflectOnce(AZ::ReflectContext* context)
        {
            if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
            {
                if (serializeContext->FindClassData(azrtti_typeid<T>()) == nullptr)
                {
                    T::Reflect(serializeContext);
                }
            }
        }
    } // namespace Internal
} // namespace JoltPhysics
