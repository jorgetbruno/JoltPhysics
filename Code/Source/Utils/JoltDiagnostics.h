#pragma once

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/string/string_view.h>

namespace JoltPhysics::Internal
{
    //! Renders " on 'Player'" for a named body, or an empty string when the caller has no
    //! name to give, so one message format reads correctly either way. A diagnostic that
    //! cannot say which entity it came from is close to useless in a level of any size,
    //! and the gem's shape and body code is several calls removed from the component that
    //! knows the name - hence the clause being passed down rather than looked up.
    inline AZStd::string NameClause(AZStd::string_view name)
    {
        return name.empty() ? AZStd::string() : AZStd::string::format(" on '" AZ_STRING_FORMAT "'", AZ_STRING_ARG(name));
    }

    //! Same, resolving the name through the component application. Falls back to the raw
    //! id when the entity cannot be named (already destroyed, or no application running as
    //! in a unit test).
    inline AZStd::string NameClause(AZ::EntityId entityId)
    {
        if (!entityId.IsValid())
        {
            return {};
        }

        AZStd::string name;
        AZ::ComponentApplicationBus::BroadcastResult(name, &AZ::ComponentApplicationRequests::GetEntityName, entityId);
        return name.empty()
            ? AZStd::string::format(" on entity %s", entityId.ToString().c_str())
            : AZStd::string::format(" on '%s'", name.c_str());
    }
} // namespace JoltPhysics::Internal
