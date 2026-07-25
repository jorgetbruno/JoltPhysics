#pragma once

#include <Editor/Components/EditorJoltJointComponentBase.h>

#include <Joint/JoltJointConfiguration.h>

namespace JoltPhysics
{
    //! Editor Jolt Hinge Joint: edit-time counterpart of JoltHingeJointComponent.
    class EditorJoltHingeJointComponent
        : public EditorJoltJointComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltHingeJointComponent, "{25C6D7E8-F9A0-4123-B4C5-D6E7F8A9B0C1}", EditorJoltJointComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        // EditorJoltJointComponentBase
        void DrawJointLimits(
            AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& jointTransform) const override;

        JointGenericProperties m_genericProperties;
        JointLimitProperties m_limitProperties;
        JointMotorProperties m_motorProperties;
    };
} // namespace JoltPhysics
