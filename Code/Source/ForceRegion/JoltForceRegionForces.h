#pragma once

#include <AzCore/Math/Aabb.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>

namespace AZ
{
    class ReflectContext;
}

namespace JoltPhysics
{
    //! What a force is being asked about: the body it would act on.
    struct JoltForceRegionEntityParams
    {
        AZ::Vector3 m_position = AZ::Vector3::CreateZero();
        AZ::Vector3 m_velocity = AZ::Vector3::CreateZero();
        AZ::Aabb m_aabb = AZ::Aabb::CreateNull();
        float m_mass = 1.0f;
    };

    //! Where the force is being applied from: the region entity itself.
    struct JoltForceRegionParams
    {
        AZ::Vector3 m_position = AZ::Vector3::CreateZero();
        AZ::Quaternion m_rotation = AZ::Quaternion::CreateIdentity();
        AZ::Aabb m_aabb = AZ::Aabb::CreateNull();
    };

    //! One force a region applies. A region sums whatever forces are attached to it, so a
    //! fan is a world-space force plus a drag, and a black hole is a point force.
    //!
    //! Mirrors PhysX's BaseForce hierarchy closely enough that a migrated author finds the
    //! same names and units; see DIVERGENCES for what is not wrapped.
    class JoltForceRegionBaseForce
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltForceRegionBaseForce, AZ::SystemAllocator);
        AZ_RTTI(JoltForceRegionBaseForce, "{4A0FF6E9-1C08-4C6B-9E7F-2B4A6E8A2E31}");

        static void Reflect(AZ::ReflectContext* context);

        virtual ~JoltForceRegionBaseForce() = default;

        //! Newtons to apply to this body this step, in world space.
        virtual AZ::Vector3 CalculateForce(
            const JoltForceRegionEntityParams& entity, const JoltForceRegionParams& region) const = 0;
    };

    //! A constant push along a world axis: wind down a corridor, a conveyor.
    class JoltForceWorldSpace final : public JoltForceRegionBaseForce
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltForceWorldSpace, AZ::SystemAllocator);
        AZ_RTTI(JoltForceWorldSpace, "{2C1B7F53-6E9D-4E5E-9E63-9E2E9B4D6E77}", JoltForceRegionBaseForce);

        static void Reflect(AZ::ReflectContext* context);

        AZ::Vector3 CalculateForce(
            const JoltForceRegionEntityParams& entity, const JoltForceRegionParams& region) const override;

        AZ::Vector3 m_direction = AZ::Vector3::CreateAxisZ();
        float m_magnitude = 10.0f;
    };

    //! The same, but in the region's own frame, so rotating the entity turns the force.
    class JoltForceLocalSpace final : public JoltForceRegionBaseForce
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltForceLocalSpace, AZ::SystemAllocator);
        AZ_RTTI(JoltForceLocalSpace, "{8B2E3A15-5F4C-4C9A-9C0E-16D8F9B7C4A2}", JoltForceRegionBaseForce);

        static void Reflect(AZ::ReflectContext* context);

        AZ::Vector3 CalculateForce(
            const JoltForceRegionEntityParams& entity, const JoltForceRegionParams& region) const override;

        AZ::Vector3 m_direction = AZ::Vector3::CreateAxisZ();
        float m_magnitude = 10.0f;
    };

    //! Radial from the region's centre: a repulsor at positive magnitude, a well at negative.
    class JoltForcePoint final : public JoltForceRegionBaseForce
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltForcePoint, AZ::SystemAllocator);
        AZ_RTTI(JoltForcePoint, "{D3F5A9C4-7B1E-4F8A-8C2D-5E6F1A9B3C7D}", JoltForceRegionBaseForce);

        static void Reflect(AZ::ReflectContext* context);

        AZ::Vector3 CalculateForce(
            const JoltForceRegionEntityParams& entity, const JoltForceRegionParams& region) const override;

        float m_magnitude = 10.0f;
    };

    //! Drag proportional to speed squared, opposing motion - water, thick air.
    class JoltForceSimpleDrag final : public JoltForceRegionBaseForce
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltForceSimpleDrag, AZ::SystemAllocator);
        AZ_RTTI(JoltForceSimpleDrag, "{6E9C2B84-1D3F-4A5E-B7C8-9F0A1B2C3D4E}", JoltForceRegionBaseForce);

        static void Reflect(AZ::ReflectContext* context);

        AZ::Vector3 CalculateForce(
            const JoltForceRegionEntityParams& entity, const JoltForceRegionParams& region) const override;

        float m_dragCoefficient = 0.47f; //!< A sphere, near enough, which is what PhysX defaults to.
        float m_volumeDensity = 1.0f;
    };

    //! Drag proportional to speed, which is what most gameplay actually wants.
    class JoltForceLinearDamping final : public JoltForceRegionBaseForce
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltForceLinearDamping, AZ::SystemAllocator);
        AZ_RTTI(JoltForceLinearDamping, "{1F7B4C6A-3E8D-4B2F-9A5C-7D8E0F1A2B3C}", JoltForceRegionBaseForce);

        static void Reflect(AZ::ReflectContext* context);

        AZ::Vector3 CalculateForce(
            const JoltForceRegionEntityParams& entity, const JoltForceRegionParams& region) const override;

        float m_damping = 1.0f;
    };

    //! The set of forces on one region, and the net force they produce.
    class JoltForceRegion
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltForceRegion, AZ::SystemAllocator);
        AZ_TYPE_INFO(JoltForceRegion, "{9D4E1A2B-8C7F-4E3A-B1D5-6F2A8C9E0B34}");

        static void Reflect(AZ::ReflectContext* context);

        AZ::Vector3 CalculateNetForce(
            const JoltForceRegionEntityParams& entity, const JoltForceRegionParams& region) const;

        //! The wind velocity this region represents, for the wind interface: a world-space
        //! force's own vector, since that is what a wind-tagged region is authored with.
        AZ::Vector3 GetWindVelocity() const;

        AZStd::vector<AZStd::shared_ptr<JoltForceRegionBaseForce>> m_forces;
    };
} // namespace JoltPhysics
