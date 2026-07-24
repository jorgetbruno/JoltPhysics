#include <Character/JoltRagdollNode.h>

namespace JoltPhysics
{
    void JoltRagdollNode::Setup(JoltScene* scene, const JPH::BodyID& bodyId, AZ::EntityId entityId, const bool* simulatedFlag)
    {
        m_rigidBody.AdoptBody(scene, bodyId, entityId);
        m_simulatedFlag = simulatedFlag;
    }
} // namespace JoltPhysics
