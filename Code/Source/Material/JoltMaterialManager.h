#pragma once

#include <AzCore/Interface/Interface.h>
#include <AzCore/Memory/SystemAllocator.h>

#include <AzFramework/Physics/Material/PhysicsMaterialManager.h>
#include <AzFramework/Physics/Shape.h>

namespace Physics
{
    class ColliderConfiguration;
}

namespace JoltPhysics
{
    //! One collider's material as tracked on a body, in compound sub-shape order.
    //! When m_shape is set (the body was built from, or had attached, a prebuilt
    //! Physics::Shape) the material is read through it, so Physics::Shape::SetMaterial
    //! applies to the live body; otherwise the resolved material is used directly.
    struct JoltColliderMaterial
    {
        AZStd::shared_ptr<Physics::Shape> m_shape;
        AZStd::shared_ptr<Physics::Material> m_material;

        //! Every slot of the collider's material list, resolved once when the body is
        //! built. A trimesh collider carries one slot per painted surface and the
        //! contact callback needs whichever the touched triangle names - but that
        //! callback runs on Jolt's narrowphase job threads, and resolving a slot there
        //! meant calling FindOrCreateMaterial, which inserts into a process-wide map
        //! with no lock. Two workers meeting an unseen slot in the same step raced on
        //! it. Resolved here instead, the callback only indexes this vector.
        AZStd::vector<AZStd::shared_ptr<Physics::Material>> m_slotMaterials;

        AZStd::shared_ptr<Physics::Material> Get() const
        {
            return m_shape ? m_shape->GetMaterial() : m_material;
        }

        //! The material for one slot of this collider, clamped to the slots that exist,
        //! falling back to the collider's own material when it has no slot list.
        AZStd::shared_ptr<Physics::Material> GetSlot(size_t slotIndex) const
        {
            if (m_slotMaterials.empty())
            {
                return Get();
            }
            return m_slotMaterials[AZStd::min(slotIndex, m_slotMaterials.size() - 1)];
        }
    };

    //! Physics::MaterialManager implementation for the Jolt backend.
    //! Self-registers on AZ::Interface<Physics::MaterialManager> (mirrors PhysX::MaterialManager).
    class JoltMaterialManager : public AZ::Interface<Physics::MaterialManager>::Registrar
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltMaterialManager, AZ::SystemAllocator);
        AZ_RTTI(JoltMaterialManager, "{B8C9D0E1-F2A3-4B5C-9D0E-1F2A3B4C5D6E}", Physics::MaterialManager);

        //! Resolves the material for a collider configuration: the material from the
        //! given material slot (clamped to the slot count), or the default material
        //! when no asset is assigned. Returns nullptr when no material manager is
        //! registered.
        static AZStd::shared_ptr<Physics::Material> ResolveMaterial(
            const Physics::ColliderConfiguration& colliderConfiguration, size_t slotIndex = 0);

        //! The same resolution against a bare slot list, for the things that carry one
        //! without a collider configuration around it - a character, for instance.
        static AZStd::shared_ptr<Physics::Material> ResolveMaterialFromSlots(
            const Physics::MaterialSlots& materialSlots, size_t slotIndex = 0);

        //! Every slot of a collider's material list, resolved in one pass. Empty when the
        //! collider has no slots. Call it while building a body, never from a contact
        //! callback: it can create materials, and the manager's map is not thread safe.
        static AZStd::vector<AZStd::shared_ptr<Physics::Material>> ResolveMaterialSlots(
            const Physics::ColliderConfiguration& colliderConfiguration);

        //! Reads the current {friction, restitution} values from a material
        //! (defaults when the material is null or not a JoltMaterial).
        static AZStd::pair<float, float> GetFrictionRestitution(const Physics::Material* material);

        //! Bumped whenever any material's properties change. Contact resolution is
        //! deliberately live - editing a material at runtime has to reach bodies that
        //! already exist - and this is what lets that happen without re-resolving every
        //! material on every manifold: bodies refresh their baked values when the number
        //! moves, which is approximately never.
        static AZ::u32 GetMaterialGeneration();
        static void BumpMaterialGeneration();

        //! Resolves the {friction, restitution} pair for a collider configuration:
        //! ResolveMaterial followed by GetFrictionRestitution.
        static AZStd::pair<float, float> ResolveFrictionRestitution(
            const Physics::ColliderConfiguration& colliderConfiguration);

    protected:
        AZStd::shared_ptr<Physics::Material> CreateDefaultMaterialInternal() override;
        AZStd::shared_ptr<Physics::Material> CreateMaterialInternal(
            const Physics::MaterialId& id, const AZ::Data::Asset<Physics::MaterialAsset>& materialAsset) override;
    };

} // namespace JoltPhysics
