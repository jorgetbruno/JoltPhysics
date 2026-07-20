#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

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

} // namespace JoltPhysics
