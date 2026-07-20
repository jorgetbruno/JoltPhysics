#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/std/containers/list.h>

namespace JoltPhysics
{
    AZStd::list<AZ::ComponentDescriptor*> GetDescriptors();

} // namespace JoltPhysics
