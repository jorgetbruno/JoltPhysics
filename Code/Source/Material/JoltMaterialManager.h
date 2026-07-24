#pragma once

#include <AzCore/Interface/Interface.h>
#include <AzCore/Memory/SystemAllocator.h>

#include <AzFramework/Physics/Material/PhysicsMaterialManager.h>

namespace Physics
{
    class ColliderConfiguration;
}

namespace JoltPhysics
{
    //! Physics::MaterialManager implementation for the Jolt backend.
    //! Self-registers on AZ::Interface<Physics::MaterialManager> (mirrors PhysX::MaterialManager).
    class JoltMaterialManager : public AZ::Interface<Physics::MaterialManager>::Registrar
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltMaterialManager, AZ::SystemAllocator);
        AZ_RTTI(JoltMaterialManager, "{B8C9D0E1-F2A3-4B5C-9D0E-1F2A3B4C5D6E}", Physics::MaterialManager);

        //! Resolves the material for a collider configuration: the material from the
        //! first material slot, or the default material when no asset is assigned.
        //! Returns nullptr when no material manager is registered.
        static AZStd::shared_ptr<Physics::Material> ResolveMaterial(
            const Physics::ColliderConfiguration& colliderConfiguration);

        //! Reads the current {friction, restitution} values from a material
        //! (defaults when the material is null or not a JoltMaterial).
        static AZStd::pair<float, float> GetFrictionRestitution(const Physics::Material* material);

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
