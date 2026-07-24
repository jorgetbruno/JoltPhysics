#pragma once

#include <Clients/Components/JoltJointComponentBase.h>

#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

namespace JoltPhysics
{
    //! Editor-side base for Jolt joint components (PhysX-style editor/runtime
    //! split). Holds the shared joint configuration (lead/follower entities and
    //! local frame); derived classes hold their type-specific properties and
    //! spawn the matching runtime joint component via BuildGameEntity.
    class EditorJoltJointComponentBase
        : public AzToolsFramework::Components::EditorComponentBase
    {
    public:
        AZ_RTTI(EditorJoltJointComponentBase, "{F2A3B4C5-D6E7-4890-E1F2-A3B4C5D6E7F8}", AzToolsFramework::Components::EditorComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

    protected:
        JoltJointComponentConfiguration m_configuration;
    };
} // namespace JoltPhysics
