#pragma once

#include <AzFramework/Physics/SimulatedBodies/StaticRigidBody.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>

namespace JoltPhysics
{
    class JoltScene;

    class JoltStaticRigidBody : public AzPhysics::StaticRigidBody
    {
    public:
        AZ_CLASS_ALLOCATOR_DECL
        AZ_RTTI(JoltStaticRigidBody, "{B1C2D3E4-F5A6-7890-BCDE-F12345678901}", AzPhysics::StaticRigidBody)

        JoltStaticRigidBody() = default;
        explicit JoltStaticRigidBody(const AzPhysics::StaticRigidBodyConfiguration& configuration);
        ~JoltStaticRigidBody() override;

        void CreateInScene(JoltScene* scene);

        AZ::Vector3 GetPosition() const override;
        AZ::Quaternion GetOrientation() const override;
        AZ::Aabb GetAabb() const override;
        AZ::EntityId GetEntityId() const override;

        AZ::Transform GetTransform() const override;
        void SetTransform(const AZ::Transform& transform) override;

        void* GetNativePointer() const override;
        AzPhysics::SceneQueryHit RayCast(const AzPhysics::RayCastRequest& request) override;

    private:
        AzPhysics::StaticRigidBodyConfiguration m_configuration;
        JoltScene* m_scene = nullptr;
        JPH::BodyID m_bodyId;
    };

} // namespace JoltPhysics
