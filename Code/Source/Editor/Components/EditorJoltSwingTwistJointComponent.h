#pragma once

#include <Editor/Components/EditorJoltJointComponentBase.h>

#include <Joint/JoltJointConfiguration.h>

namespace JoltPhysics
{
    //! Editor Jolt Swing-Twist Joint: edit-time counterpart of JoltSwingTwistJointComponent.
    class EditorJoltSwingTwistJointComponent
        : public EditorJoltJointComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltSwingTwistJointComponent, "{B8C9DAEB-FC0D-47E8-F1A2-B3C4D5E6F7A8}", EditorJoltJointComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        // EditorJoltJointComponentBase
        void DrawJointLimits(
            AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& jointTransform) const override;

        JointGenericProperties m_genericProperties;
        float m_normalHalfConeAngle = 45.0f;
        float m_planeHalfConeAngle = 45.0f;
        float m_twistLower = -45.0f;
        float m_twistUpper = 45.0f;
    };
} // namespace JoltPhysics
