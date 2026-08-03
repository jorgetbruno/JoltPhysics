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

        AZStd::shared_ptr<Physics::Material> Get() const
        {
            return m_shape ? m_shape->GetMaterial() : m_material;
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
