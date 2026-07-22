#pragma once

#include <AzCore/std/string/string.h>

namespace AZ
{
    class Component;
}

namespace JoltPhysics::Internal
{
    //! Generates the serialized identifier for a component ("{TypeName}" or
    //! "{TypeName}_{N}" when the entity already has a component of the same type),
    //! mirroring what AzToolsFramework's EditorComponentBase does for editor components.
    //! The DPE inspector's PrefabComponentAdapter requires a non-empty identifier;
    //! plain AZ::Component returns an empty string by default.
    AZStd::string GenerateSerializedIdentifier(AZ::Component* component);
} // namespace JoltPhysics::Internal
