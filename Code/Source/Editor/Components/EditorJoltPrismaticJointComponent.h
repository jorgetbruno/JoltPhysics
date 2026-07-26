#pragma once

#include <Editor/Components/EditorJoltJointComponentBase.h>

#include <Joint/JoltJointConfiguration.h>

namespace JoltPhysics
{
    //! Editor Jolt Prismatic Joint: edit-time counterpart of JoltPrismaticJointComponent.
    class EditorJoltPrismaticJointComponent
        : public EditorJoltJointComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltPrismaticJointComponent, "{36D7E8F9-A0B1-4234-C5D6-E7F8A9B0C1D2}", EditorJoltJointComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void Activate() override;
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
