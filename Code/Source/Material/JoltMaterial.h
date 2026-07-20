#pragma once

#include <AzCore/Memory/SystemAllocator.h>

#include <AzFramework/Physics/Material/PhysicsMaterial.h>
#include <AzFramework/Physics/Material/PhysicsMaterialAsset.h>

namespace JoltPhysics
{
    //! Physics::Material implementation for the Jolt backend.
    //! Holds the PhysX-compatible property set (same property names as the PhysX gem's
    //! PhysX::Material so game code and assets interchange): DynamicFriction,
    //! StaticFriction, Restitution, Density, RestitutionCombineMode, FrictionCombineMode.
    class JoltMaterial : public Physics::Material
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltMaterial, AZ::SystemAllocator);
        AZ_RTTI(JoltMaterial, "{A7B8C9D0-E1F2-4A5B-8C9D-0E1F2A3B4C5D}", Physics::Material);

        JoltMaterial() = default;
        JoltMaterial(const Physics::MaterialId& id, const AZ::Data::Asset<Physics::MaterialAsset>& materialAsset);

        Physics::MaterialPropertyValue GetProperty(AZStd::string_view propertyName) const override;
        void SetProperty(AZStd::string_view propertyName, Physics::MaterialPropertyValue value) override;

        float GetDynamicFriction() const { return m_dynamicFriction; }
        float GetStaticFriction() const { return m_staticFriction; }
        float GetRestitution() const { return m_restitution; }
        float GetDensity() const { return m_density; }

        static constexpr float DefaultFriction = 0.5f;
        static constexpr float DefaultRestitution = 0.0f;
        static constexpr float DefaultDensity = 1000.0f;

    private:
        float m_dynamicFriction = DefaultFriction;
        float m_staticFriction = DefaultFriction;
        float m_restitution = DefaultRestitution;
        float m_density = DefaultDensity;
    };

} // namespace JoltPhysics
