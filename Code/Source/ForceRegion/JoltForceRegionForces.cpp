#include <ForceRegion/JoltForceRegionForces.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace JoltPhysics
{
    void JoltForceRegionBaseForce::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltForceRegionBaseForce>()->Version(1);
        }

        JoltForceWorldSpace::Reflect(context);
        JoltForceLocalSpace::Reflect(context);
        JoltForcePoint::Reflect(context);
        JoltForceSimpleDrag::Reflect(context);
        JoltForceLinearDamping::Reflect(context);
    }

    void JoltForceWorldSpace::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltForceWorldSpace, JoltForceRegionBaseForce>()
                ->Version(1)
                ->Field("Direction", &JoltForceWorldSpace::m_direction)
                ->Field("Magnitude", &JoltForceWorldSpace::m_magnitude)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltForceWorldSpace>("World space force", "A constant push along a world axis.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltForceWorldSpace::m_direction,
                        "Direction", "Direction of the force in world space.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltForceWorldSpace::m_magnitude,
                        "Magnitude", "Strength of the force in newtons.")
                    ;
            }
        }
    }

    AZ::Vector3 JoltForceWorldSpace::CalculateForce(
        [[maybe_unused]] const JoltForceRegionEntityParams& entity,
        [[maybe_unused]] const JoltForceRegionParams& region) const
    {
        return m_direction.GetNormalizedSafe() * m_magnitude;
    }

    void JoltForceLocalSpace::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltForceLocalSpace, JoltForceRegionBaseForce>()
                ->Version(1)
                ->Field("Direction", &JoltForceLocalSpace::m_direction)
                ->Field("Magnitude", &JoltForceLocalSpace::m_magnitude)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltForceLocalSpace>(
                    "Local space force", "A constant push along an axis of the region's own frame.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltForceLocalSpace::m_direction,
                        "Direction", "Direction of the force in the region's local space.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltForceLocalSpace::m_magnitude,
                        "Magnitude", "Strength of the force in newtons.")
                    ;
            }
        }
    }

    AZ::Vector3 JoltForceLocalSpace::CalculateForce(
        [[maybe_unused]] const JoltForceRegionEntityParams& entity, const JoltForceRegionParams& region) const
    {
        return region.m_rotation.TransformVector(m_direction.GetNormalizedSafe()) * m_magnitude;
    }

    void JoltForcePoint::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltForcePoint, JoltForceRegionBaseForce>()
                ->Version(1)
                ->Field("Magnitude", &JoltForcePoint::m_magnitude)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltForcePoint>(
                    "Point force", "Pushes away from the region's centre, or pulls towards it when negative.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltForcePoint::m_magnitude,
                        "Magnitude", "Strength of the force in newtons. Negative attracts.")
                    ;
            }
        }
    }

    AZ::Vector3 JoltForcePoint::CalculateForce(
        const JoltForceRegionEntityParams& entity, const JoltForceRegionParams& region) const
    {
        // A body exactly on the centre has no direction to be pushed in, and normalizing
        // there would produce a NaN rather than nothing.
        const AZ::Vector3 offset = entity.m_position - region.m_position;
        return offset.GetNormalizedSafe() * m_magnitude;
    }

    void JoltForceSimpleDrag::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltForceSimpleDrag, JoltForceRegionBaseForce>()
                ->Version(1)
                ->Field("DragCoefficient", &JoltForceSimpleDrag::m_dragCoefficient)
                ->Field("VolumeDensity", &JoltForceSimpleDrag::m_volumeDensity)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltForceSimpleDrag>(
                    "Simple drag", "Resistance proportional to speed squared, as a fluid gives.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltForceSimpleDrag::m_dragCoefficient,
                        "Drag coefficient", "Shape-dependent drag factor; 0.47 is a sphere.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltForceSimpleDrag::m_volumeDensity,
                        "Fluid density", "Density of the medium filling the region.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ;
            }
        }
    }

    AZ::Vector3 JoltForceSimpleDrag::CalculateForce(
        const JoltForceRegionEntityParams& entity, [[maybe_unused]] const JoltForceRegionParams& region) const
    {
        const float speed = entity.m_velocity.GetLength();
        if (speed <= 0.0f)
        {
            return AZ::Vector3::CreateZero();
        }

        // Frontal area approximated from the body's bounds, which is the only measure of
        // its size available here.
        const AZ::Vector3 extents = entity.m_aabb.IsValid() ? entity.m_aabb.GetExtents() : AZ::Vector3::CreateOne();
        const float area = AZStd::max(extents.GetX() * extents.GetY(), 1e-4f);

        const float dragMagnitude = 0.5f * m_volumeDensity * m_dragCoefficient * area * speed * speed;
        return -entity.m_velocity.GetNormalizedSafe() * dragMagnitude;
    }

    void JoltForceLinearDamping::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltForceLinearDamping, JoltForceRegionBaseForce>()
                ->Version(1)
                ->Field("Damping", &JoltForceLinearDamping::m_damping)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltForceLinearDamping>(
                    "Linear damping", "Resistance proportional to speed - the predictable one.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltForceLinearDamping::m_damping,
                        "Damping", "How strongly motion is bled off.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ;
            }
        }
    }

    AZ::Vector3 JoltForceLinearDamping::CalculateForce(
        const JoltForceRegionEntityParams& entity, [[maybe_unused]] const JoltForceRegionParams& region) const
    {
        // Scaled by mass so the resulting acceleration is mass-independent, which is what
        // makes a damping region feel the same on a crate and a barrel.
        return -entity.m_velocity * m_damping * entity.m_mass;
    }

    void JoltForceRegion::Reflect(AZ::ReflectContext* context)
    {
        JoltForceRegionBaseForce::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltForceRegion>()
                ->Version(1)
                ->Field("Forces", &JoltForceRegion::m_forces)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltForceRegion>("Forces", "The forces this region applies.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltForceRegion::m_forces,
                        "Forces", "Forces applied to every body inside the region; their sum is what acts.")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    AZ::Vector3 JoltForceRegion::CalculateNetForce(
        const JoltForceRegionEntityParams& entity, const JoltForceRegionParams& region) const
    {
        AZ::Vector3 netForce = AZ::Vector3::CreateZero();
        for (const AZStd::shared_ptr<JoltForceRegionBaseForce>& force : m_forces)
        {
            if (force)
            {
                netForce += force->CalculateForce(entity, region);
            }
        }
        return netForce;
    }

    AZ::Vector3 JoltForceRegion::GetWindVelocity() const
    {
        // A wind-tagged region is authored as a world-space push, so that vector is what
        // consumers of the wind interface want. Other force types have no direction that
        // means "wind" on its own and are ignored here rather than guessed at.
        for (const AZStd::shared_ptr<JoltForceRegionBaseForce>& force : m_forces)
        {
            if (const auto* worldForce = azrtti_cast<const JoltForceWorldSpace*>(force.get()))
            {
                return worldForce->m_direction.GetNormalizedSafe() * worldForce->m_magnitude;
            }
        }
        return AZ::Vector3::CreateZero();
    }
} // namespace JoltPhysics
