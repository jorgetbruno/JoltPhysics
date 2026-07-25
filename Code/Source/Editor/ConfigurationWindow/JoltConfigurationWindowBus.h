#pragma once

#include <AzCore/EBus/EBus.h>

namespace JoltPhysics::Editor
{
    //! Name the configuration pane is registered under; also its Tools menu entry.
    inline constexpr const char* ConfigurationWindowName = "Jolt Physics Configuration";

    //! Talks to the Jolt Physics Configuration window. Open it first via
    //! EditorRequests::OpenViewPane(ConfigurationWindowName); with the pane closed
    //! there is no handler and these calls do nothing.
    class JoltConfigurationWindowRequests : public AZ::EBusTraits
    {
    public:
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static const AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;

        virtual void ShowGlobalSettingsTab() = 0;
        virtual void ShowCollisionLayersTab() = 0;
        virtual void ShowCollisionGroupsTab() = 0;
    };

    using JoltConfigurationWindowRequestBus = AZ::EBus<JoltConfigurationWindowRequests>;
} // namespace JoltPhysics::Editor
