#pragma once

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <AzFramework/Physics/ShapeConfiguration.h>

namespace JoltPhysics
{
    //! Cylinder collider settings. AzFramework declares Physics::ShapeType::Cylinder but
    //! ships no configuration for it (PhysX has no native cylinder and approximates one
    //! with a cooked convex hull), so the Jolt backend supplies its own and maps it onto
    //! JPH::CylinderShape directly.
    //!
    //! The cylinder is Z-axis aligned, matching the O3DE capsule convention.
    class JoltCylinderShapeConfiguration
        : public Physics::ShapeConfiguration
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltCylinderShapeConfiguration, AZ::SystemAllocator);
        AZ_RTTI(JoltCylinderShapeConfiguration, "{C3D4E5F6-A7B8-4192-A5B6-C7D8E9F0A1B2}", Physics::ShapeConfiguration);
        static void Reflect(AZ::ReflectContext* context);

        JoltCylinderShapeConfiguration() = default;
        JoltCylinderShapeConfiguration(float height, float radius)
            : m_height(height)
            , m_radius(radius)
        {
        }

        Physics::ShapeType GetShapeType() const override
        {
            return Physics::ShapeType::Cylinder;
        }

        AZStd::shared_ptr<Physics::ShapeConfiguration> Clone() const override
        {
            return AZStd::make_shared<JoltCylinderShapeConfiguration>(*this);
        }

        float m_height = 1.0f; //!< Total height along the local Z axis.
        float m_radius = 0.5f;
    };
} // namespace JoltPhysics
