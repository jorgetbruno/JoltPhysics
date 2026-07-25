#pragma once

#include <Editor/Components/EditorJoltJointComponentBase.h>

#include <Joint/JoltJointConfiguration.h>

namespace JoltPhysics
{
    //! Editor Jolt Distance Joint: edit-time counterpart of JoltDistanceJointComponent.
    class EditorJoltDistanceJointComponent
        : public EditorJoltJointComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltDistanceJointComponent, "{F6A7B8C9-DAEB-45C6-D7E8-F1A2B3C4D5E6}", EditorJoltJointComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        // EditorJoltJointComponentBase
        void DrawJointLimits(
            AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& jointTransform) const override;

        JointGenericProperties m_genericProperties;
        float m_minDistance = 0.0f;
        float m_maxDistance = 1.0f;
        float m_springFrequency = 0.0f;
        float m_springDamping = 0.0f;
    };
} // namespace JoltPhysics
