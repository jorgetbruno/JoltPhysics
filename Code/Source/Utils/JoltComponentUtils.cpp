#include <Utils/JoltComponentUtils.h>

#include <AzCore/Component/Component.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/string/conversions.h>

namespace JoltPhysics::Internal
{
    AZStd::string GenerateSerializedIdentifier(AZ::Component* component)
    {
        AZ::Entity* entity = component ? component->GetEntity() : nullptr;
        if (!entity)
        {
            return {};
        }

        // Strip the namespace from the RTTI type name ("JoltPhysics::JoltBoxColliderComponent"
        // -> "JoltBoxColliderComponent"), matching how the type name appears in prefabs.
        AZStd::string typeName = component->RTTI_GetTypeName();
        if (const size_t separator = typeName.rfind("::"); separator != AZStd::string::npos)
        {
            typeName = typeName.substr(separator + 2);
        }

        AZStd::unordered_set<AZStd::string> existingIdentifiers;
        for (AZ::Component* other : entity->GetComponents())
        {
            if (other != component && other->RTTI_GetType() == component->RTTI_GetType())
            {
                if (AZStd::string existingIdentifier = other->GetSerializedIdentifier();
                    !existingIdentifier.empty())
                {
                    existingIdentifiers.emplace(AZStd::move(existingIdentifier));
                }
            }
        }

        AZStd::string serializedIdentifier = typeName;
        AZ::u64 suffix = 1;
        while (existingIdentifiers.find(serializedIdentifier) != existingIdentifiers.end())
        {
            serializedIdentifier = AZStd::string::format("%s_%llu", typeName.c_str(), ++suffix);
        }
        return serializedIdentifier;
    }
} // namespace JoltPhysics::Internal
