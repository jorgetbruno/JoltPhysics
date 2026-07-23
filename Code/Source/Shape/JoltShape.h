#pragma once

#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace JoltPhysics
{
    //! Standalone Physics::Shape wrapper around a native Jolt shape.
    //! Used for callers that go through the generic Physics::SystemRequests::CreateShape
    //! entry point (e.g. the WhiteBox gem) rather than through a Jolt collider component.
    //! Note: this is a fairly minimal implementation. In particular it does not integrate
    //! with the AzFramework physics material system (SetMaterial/GetMaterial are no-ops);
    //! collision layer/group and local pose are stored and reflected into the native shape
    //! only at creation time (Jolt shapes are otherwise immutable geometry).
    class JoltShape final
        : public Physics::Shape
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltShape, AZ::SystemAllocator);
        AZ_RTTI(JoltShape, "{2E6A9C74-4C1D-4B7A-9E2F-7A6B5C4D3E2F}", Physics::Shape);

        JoltShape(
            const Physics::ColliderConfiguration& colliderConfiguration,
            const Physics::ShapeConfiguration& shapeConfiguration,
            JPH::RefConst<JPH::Shape> nativeShape);
        ~JoltShape() override = default;

        // Physics::Shape
        void SetMaterial(const AZStd::shared_ptr<Physics::Material>& material) override;
        AZStd::shared_ptr<Physics::Material> GetMaterial() const override;
        Physics::MaterialId GetMaterialId() const override;

        void SetCollisionLayer(const AzPhysics::CollisionLayer& layer) override;
        AzPhysics::CollisionLayer GetCollisionLayer() const override;

        void SetCollisionGroup(const AzPhysics::CollisionGroup& group) override;
        AzPhysics::CollisionGroup GetCollisionGroup() const override;

        void SetName(const char* name) override;

        void SetLocalPose(const AZ::Vector3& offset, const AZ::Quaternion& rotation) override;
        AZStd::pair<AZ::Vector3, AZ::Quaternion> GetLocalPose() const override;

        float GetRestOffset() const override;
        float GetContactOffset() const override;
        void SetRestOffset(float restOffset) override;
        void SetContactOffset(float contactOffset) override;

        void* GetNativePointer() override;
        const void* GetNativePointer() const override;

        AZ::Crc32 GetTag() const override;

        void AttachedToActor(void* actor) override;
        void DetachedFromActor() override;

        AzPhysics::SceneQueryHit RayCast(
            const AzPhysics::RayCastRequest& worldSpaceRequest, const AZ::Transform& worldTransform) override;
        AzPhysics::SceneQueryHit RayCastLocal(const AzPhysics::RayCastRequest& localSpaceRequest) override;

        AZ::Aabb GetAabb(const AZ::Transform& worldTransform) const override;
        AZ::Aabb GetAabbLocal() const override;

        AZStd::shared_ptr<Physics::ShapeConfiguration> GetShapeConfiguration() const override;

        void GetGeometry(
            AZStd::vector<AZ::Vector3>& vertices,
            AZStd::vector<AZ::u32>& indices,
            const AZ::Aabb* optionalBounds = nullptr) const override;

    private:
        JPH::RefConst<JPH::Shape> m_nativeShape;
        AZStd::shared_ptr<Physics::ColliderConfiguration> m_colliderConfiguration;
        AZStd::shared_ptr<Physics::ShapeConfiguration> m_shapeConfiguration;

        AZ::Vector3 m_localPosition = AZ::Vector3::CreateZero();
        AZ::Quaternion m_localRotation = AZ::Quaternion::CreateIdentity();

        AzPhysics::CollisionLayer m_collisionLayer;
        AzPhysics::CollisionGroup m_collisionGroup;

        float m_restOffset = 0.0f;
        float m_contactOffset = 0.02f;

        void* m_attachedActor = nullptr;
    };

} // namespace JoltPhysics
