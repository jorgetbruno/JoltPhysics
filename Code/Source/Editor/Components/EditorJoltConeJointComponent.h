#pragma once

#include <Editor/Components/EditorJoltJointComponentBase.h>

#include <Joint/JoltJointConfiguration.h>

namespace JoltPhysics
{
    //! Editor Jolt Cone Joint: edit-time counterpart of JoltConeJointComponent.
    class EditorJoltConeJointComponent
        : public EditorJoltJointComponentBase
    {
    public:
        AZ_COMPONENT(EditorJoltConeJointComponent, "{A7B8C9DA-EBFC-46D7-E8F1-A2B3C4D5E6F7}", EditorJoltJointComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        JointGenericProperties m_genericProperties;
        float m_halfConeAngle = 45.0f;
    };
} // namespace JoltPhysics
