#include <Material/JoltMaterialManager.h>
#include <Material/JoltMaterial.h>

#include <AzCore/Math/MathUtils.h>

namespace JoltPhysics
{
    namespace
    {
        // Property names, kept identical to the PhysX gem's PhysX::Material.
        constexpr AZStd::string_view DynamicFrictionName = "DynamicFriction";
        constexpr AZStd::string_view StaticFrictionName = "StaticFriction";
        constexpr AZStd::string_view RestitutionName = "Restitution";
        constexpr AZStd::string_view DensityName = "Density";

        float ReadFloatProperty(const Physics::MaterialAsset& asset, AZStd::string_view name, float defaultValue)
        {
            const auto& properties = asset.GetMaterialProperties();
            if (auto found = properties.find(AZStd::string(name)); found != properties.end() && found->second.Is<float>())
            {
                return found->second.GetValue<float>();
            }
            return defaultValue;
        }
    }

    JoltMaterial::JoltMaterial(const Physics::MaterialId& id, const AZ::Data::Asset<Physics::MaterialAsset>& materialAsset)
        : Physics::Material(id, materialAsset)
    {
        if (const Physics::MaterialAsset* asset = materialAsset.Get())
        {
            m_dynamicFriction = ReadFloatProperty(*asset, DynamicFrictionName, DefaultFriction);
            m_staticFriction = ReadFloatProperty(*asset, StaticFrictionName, m_dynamicFriction);
            // Jolt has one friction coefficient per body, not the static/dynamic pair
            // PhysX models, and the dynamic one is what it means. A material that asks for
            // a different static friction is silently getting the dynamic value, so say so
            // once rather than leaving the author to wonder why the slope still slips.
            AZ_Warning("JoltPhysics", AZ::IsClose(m_staticFriction, m_dynamicFriction, 1e-3f),
                "Physics material asks for a static friction of %.3f and a dynamic friction of %.3f. Jolt has a "
                "single friction coefficient, so the dynamic value is used and the static one is ignored "
                "(see DIVERGENCES.md, \"Static friction\").",
                m_staticFriction, m_dynamicFriction);
            m_restitution = ReadFloatProperty(*asset, RestitutionName, DefaultRestitution);
            m_density = ReadFloatProperty(*asset, DensityName, DefaultDensity);
        }
    }

    Physics::MaterialPropertyValue JoltMaterial::GetProperty(AZStd::string_view propertyName) const
    {
        if (propertyName == DynamicFrictionName)
        {
            return m_dynamicFriction;
        }
        if (propertyName == StaticFrictionName)
        {
            return m_staticFriction;
        }
        if (propertyName == RestitutionName)
        {
            return m_restitution;
        }
        if (propertyName == DensityName)
        {
            return m_density;
        }
        return Physics::MaterialPropertyValue();
    }

    void JoltMaterial::SetProperty(AZStd::string_view propertyName, Physics::MaterialPropertyValue value)
    {
        if (!value.Is<float>())
        {
            return;
        }

        // Bodies bake their friction and restitution at creation and refresh them when
        // this number moves, so a runtime edit still reaches bodies that already exist -
        // without every contact re-resolving a material that almost never changes.
        JoltMaterialManager::BumpMaterialGeneration();

        if (propertyName == DynamicFrictionName)
        {
            m_dynamicFriction = value.GetValue<float>();
        }
        else if (propertyName == StaticFrictionName)
        {
            m_staticFriction = value.GetValue<float>();
        }
        else if (propertyName == RestitutionName)
        {
            m_restitution = value.GetValue<float>();
        }
        else if (propertyName == DensityName)
        {
            m_density = value.GetValue<float>();
        }
    }

} // namespace JoltPhysics
