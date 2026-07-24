#pragma once

#include <Editor/Components/EditorJoltJointComponentBase.h>

#include <Joint/JoltJointConfiguration.h>

namespace JoltPhysics
{
    //! Editor Jolt Ball Joint: edit-time counterpart of JoltBallJointComponent.
    class EditorJoltBallJointComponent
        : public EditorJoltJointComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltBallJointComponent, "{14B5C6D7-E8F9-4012-A3B4-C5D6E7F8A9B0}", EditorJoltJointComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        JointGenericProperties m_genericProperties;
        JointLimitProperties m_limitProperties;
    };
} // namespace JoltPhysics
