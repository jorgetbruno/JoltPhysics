#pragma once

#include <Editor/Components/EditorJoltJointComponentBase.h>

#include <Joint/JoltJointConfiguration.h>

namespace JoltPhysics
{
    //! Editor Jolt Fixed Joint: edit-time counterpart of JoltFixedJointComponent.
    class EditorJoltFixedJointComponent
        : public EditorJoltJointComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltFixedJointComponent, "{03A4B5C6-D7E8-4901-F2A3-B4C5D6E7F8A9}", EditorJoltJointComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        JointGenericProperties m_genericProperties;
    };
} // namespace JoltPhysics
