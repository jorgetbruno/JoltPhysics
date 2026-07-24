#pragma once

#include <AzFramework/Physics/SimulatedBodies/StaticRigidBody.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>

#include <Material/JoltMaterialManager.h>

#include <Jolt/Jolt.h>
#include <Jolt/Core/Reference.h>
#include <Jolt/Physics/Body/BodyID.h>

namespace JPH
{
    class Shape;
    class HeightFieldShape;
}

namespace Physics
{
    class Material;
}

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

        //! Removes the Jolt body from the physics world immediately (the object
        //! itself is deleted later by the scene's deferred deletion).
        void RemoveFromJoltWorld();

        //! Removes the body from / re-adds it to the physics world without destroying
        //! it (Scene::DisableSimulationOfBody / EnableSimulationOfBody).
        void SetSimulationEnabled(bool enabled);

        const JPH::BodyID& GetBodyId() const { return m_bodyId; }
        bool IsSensor() const { return m_isSensor; }

        AZ::Vector3 GetPosition() const override;
        AZ::Quaternion GetOrientation() const override;
        AZ::Aabb GetAabb() const override;
        AZ::EntityId GetEntityId() const override;

        AZ::Transform GetTransform() const override;
        void SetTransform(const AZ::Transform& transform) override;

        void AddShape(AZStd::shared_ptr<Physics::Shape> shape) override;

        //! Number of colliders the body was built from (compound sub-shape order), or the
        //! number of provider material slots for heightfields.
        size_t GetColliderCount() const;
        //! The material of the collider (or heightfield material slot) at the given index,
        //! read live so runtime material changes apply to contacts of existing bodies.
        //! Null when the index is out of range.
        AZStd::shared_ptr<Physics::Material> GetColliderMaterial(size_t colliderIndex) const;
        //! Per-square material indices for heightfield bodies (empty otherwise).
        const AZStd::vector<AZ::u8>& GetHeightfieldMaterialIndices() const { return m_heightfieldMaterialIndices; }

        AZ::Crc32 GetNativeType() const override;
        void* GetNativePointer() const override;
        AzPhysics::SceneQueryHit RayCast(const AzPhysics::RayCastRequest& request) override;

    private:
        AzPhysics::StaticRigidBodyConfiguration m_configuration;
        JoltScene* m_scene = nullptr;
        JPH::BodyID m_bodyId;
        AZ::EntityId m_entityId;
        bool m_isSensor = false;
        bool m_removedFromWorld = false;
        //! Per-collider (or per heightfield material slot) materials, in compound
        //! sub-shape order (see JoltColliderMaterial).
        AZStd::vector<JoltColliderMaterial> m_colliderMaterials;
        //! Shapes attached after creation via AddShape, in the order they occupy in the
        //! mutable compound (i.e. after the colliders the body was created with).
        AZStd::vector<AZStd::shared_ptr<Physics::Shape>> m_attachedShapes;
        //! The body's geometry; replaced by a MutableCompoundShape once shapes are attached.
        JPH::RefConst<JPH::Shape> m_baseShape;
        AZStd::vector<AZ::u8> m_heightfieldMaterialIndices;

        void ResolveHeightfieldMaterialData(const JPH::HeightFieldShape* heightFieldShape);
    };

} // namespace JoltPhysics
