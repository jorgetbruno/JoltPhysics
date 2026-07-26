#pragma once

#include <Editor/Components/EditorJoltJointComponentBase.h>

namespace JoltPhysics
{
    //! Editor Jolt D6 Joint: edit-time counterpart of JoltD6JointComponent.
    class EditorJoltD6JointComponent
        : public EditorJoltJointComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltD6JointComponent, "{47E8F9A0-B1C2-4345-D6E7-F8A9B0C1D2E3}", EditorJoltJointComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void Activate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        // EditorJoltJointComponentBase
        void DrawJointLimits(
            AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& jointTransform) const override;

        float m_swingLimitY = 45.0f;
        float m_swingLimitZ = 45.0f;
        float m_twistLimitLower = -45.0f;
        float m_twistLimitUpper = 45.0f;
    };
} // namespace JoltPhysics
