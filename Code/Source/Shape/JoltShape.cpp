#include <Shape/JoltShape.h>
#include <Material/JoltMaterialManager.h>
#include <Utils/Conversions.h>

#include <AzCore/Math/Aabb.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <AzFramework/Physics/Material/PhysicsMaterial.h>

#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/Shape/SubShapeID.h>

namespace JoltPhysics
{
    JoltShape::JoltShape(
        const Physics::ColliderConfiguration& colliderConfiguration,
        const Physics::ShapeConfiguration& shapeConfiguration,
        JPH::RefConst<JPH::Shape> nativeShape)
        : m_nativeShape(AZStd::move(nativeShape))
        , m_colliderConfiguration(AZStd::make_shared<Physics::ColliderConfiguration>(colliderConfiguration))
        , m_shapeConfiguration(shapeConfiguration.Clone())
        , m_localPosition(colliderConfiguration.m_position)
        , m_localRotation(colliderConfiguration.m_rotation)
        , m_collisionLayer(colliderConfiguration.m_collisionLayer)
        , m_restOffset(colliderConfiguration.m_restOffset)
        , m_contactOffset(colliderConfiguration.m_contactOffset)
        , m_material(JoltMaterialManager::ResolveMaterial(colliderConfiguration))
    {
    }

    void JoltShape::SetMaterial(const AZStd::shared_ptr<Physics::Material>& material)
    {
        m_material = material;
    }

    AZStd::shared_ptr<Physics::Material> JoltShape::GetMaterial() const
    {
        return m_material;
    }

    Physics::MaterialId JoltShape::GetMaterialId() const
    {
        return m_material ? m_material->GetId() : Physics::MaterialId();
    }

    void JoltShape::SetCollisionLayer(const AzPhysics::CollisionLayer& layer)
    {
        m_collisionLayer = layer;
    }

    AzPhysics::CollisionLayer JoltShape::GetCollisionLayer() const
    {
        return m_collisionLayer;
    }

    void JoltShape::SetCollisionGroup(const AzPhysics::CollisionGroup& group)
    {
        m_collisionGroup = group;
    }

    AzPhysics::CollisionGroup JoltShape::GetCollisionGroup() const
    {
        return m_collisionGroup;
    }

    void JoltShape::SetName([[maybe_unused]] const char* name)
    {
        // Jolt shapes don't carry a runtime name; nothing to forward this to.
    }

    void JoltShape::SetLocalPose(const AZ::Vector3& offset, const AZ::Quaternion& rotation)
    {
        m_localPosition = offset;
        m_localRotation = rotation;
    }

    AZStd::pair<AZ::Vector3, AZ::Quaternion> JoltShape::GetLocalPose() const
    {
        return { m_localPosition, m_localRotation };
    }

    float JoltShape::GetRestOffset() const
    {
        return m_restOffset;
    }

    float JoltShape::GetContactOffset() const
    {
        return m_contactOffset;
    }

    void JoltShape::SetRestOffset(float restOffset)
    {
        m_restOffset = restOffset;
    }

    void JoltShape::SetContactOffset(float contactOffset)
    {
        m_contactOffset = contactOffset;
    }

    void* JoltShape::GetNativePointer()
    {
        return const_cast<JPH::Shape*>(m_nativeShape.GetPtr());
    }

    const void* JoltShape::GetNativePointer() const
    {
        return m_nativeShape.GetPtr();
    }

    AZ::Crc32 JoltShape::GetTag() const
    {
        return m_colliderConfiguration ? AZ::Crc32(m_colliderConfiguration->m_tag) : AZ::Crc32();
    }

    void JoltShape::AttachedToActor(void* actor)
    {
        // Jolt shapes are immutable geometry shared by reference; there's no live-actor
        // state to push into here. Track the pointer only for bookkeeping/debugging.
        m_attachedActor = actor;
    }

    void JoltShape::DetachedFromActor()
    {
        m_attachedActor = nullptr;
    }

    AzPhysics::SceneQueryHit JoltShape::RayCastLocal(const AzPhysics::RayCastRequest& localSpaceRequest)
    {
        AzPhysics::SceneQueryHit hit;
        if (!m_nativeShape)
        {
            return hit;
        }

        const JPH::RayCast ray(
            Conversions::ToJolt(localSpaceRequest.m_start),
            Conversions::ToJolt(localSpaceRequest.m_direction) * localSpaceRequest.m_distance);

        JPH::RayCastResult result;
        const bool hasHit = m_nativeShape->CastRay(ray, JPH::SubShapeIDCreator(), result);
        if (!hasHit)
        {
            return hit;
        }

        const JPH::Vec3 localHitPoint = ray.GetPointOnRay(result.mFraction);
        const JPH::Vec3 localNormal = m_nativeShape->GetSurfaceNormal(result.mSubShapeID2, localHitPoint);

        hit.m_resultFlags = AzPhysics::SceneQuery::ResultFlags::Distance | AzPhysics::SceneQuery::ResultFlags::Position |
            AzPhysics::SceneQuery::ResultFlags::Normal;
        hit.m_distance = result.mFraction * localSpaceRequest.m_distance;
        hit.m_position = Conversions::FromJolt(localHitPoint);
        hit.m_normal = Conversions::FromJolt(localNormal);
        hit.m_shape = this;
        return hit;
    }

    AzPhysics::SceneQueryHit JoltShape::RayCast(
        const AzPhysics::RayCastRequest& worldSpaceRequest, const AZ::Transform& worldTransform)
    {
        const AZ::Transform inverseWorld = worldTransform.GetInverse();
        AzPhysics::RayCastRequest localRequest = worldSpaceRequest;
        localRequest.m_start = inverseWorld.TransformPoint(worldSpaceRequest.m_start);
        localRequest.m_direction = inverseWorld.TransformVector(worldSpaceRequest.m_direction);

        AzPhysics::SceneQueryHit hit = RayCastLocal(localRequest);
        if (hit)
        {
            hit.m_position = worldTransform.TransformPoint(hit.m_position);
            hit.m_normal = worldTransform.TransformVector(hit.m_normal).GetNormalized();
        }
        return hit;
    }

    AZ::Aabb JoltShape::GetAabbLocal() const
    {
        if (!m_nativeShape)
        {
            return AZ::Aabb::CreateNull();
        }
        const JPH::AABox bounds = m_nativeShape->GetLocalBounds();
        return AZ::Aabb::CreateFromMinMax(Conversions::FromJolt(bounds.mMin), Conversions::FromJolt(bounds.mMax));
    }

    AZ::Aabb JoltShape::GetAabb(const AZ::Transform& worldTransform) const
    {
        if (!m_nativeShape)
        {
            return AZ::Aabb::CreateNull();
        }
        const JPH::AABox bounds = m_nativeShape->GetWorldSpaceBounds(Conversions::ToJolt(worldTransform), JPH::Vec3::sReplicate(1.0f));
        return AZ::Aabb::CreateFromMinMax(Conversions::FromJolt(bounds.mMin), Conversions::FromJolt(bounds.mMax));
    }

    AZStd::shared_ptr<Physics::ShapeConfiguration> JoltShape::GetShapeConfiguration() const
    {
        return m_shapeConfiguration;
    }

    void JoltShape::GetGeometry(
        AZStd::vector<AZ::Vector3>& vertices, AZStd::vector<AZ::u32>& indices, [[maybe_unused]] const AZ::Aabb* optionalBounds) const
    {
        vertices.clear();
        indices.clear();
        if (!m_nativeShape)
        {
            return;
        }

        // Generic triangle-soup extraction (works for any Jolt shape type, not just mesh).
        // Note: this does not currently respect optionalBounds and does not weld shared
        // vertices, so 'indices' here is really just [0, 1, 2, 3, ...] over the returned
        // triangle-soup 'vertices' (matching the "vertices only" contract Physics::Shape
        // documents for non-mesh shapes).
        JPH::Shape::GetTrianglesContext context;
        const JPH::AABox bounds = m_nativeShape->GetLocalBounds();
        // Same COM subtlety as BuildShapeWireframe: inPositionCOM positions the shape's
        // center of mass in the output frame, and convex-hull vertices are stored relative
        // to the hull's centroid, so the shape's own COM is what yields local-frame vertices.
        m_nativeShape->GetTrianglesStart(
            context, bounds, m_nativeShape->GetCenterOfMass(), JPH::Quat::sIdentity(), JPH::Vec3::sReplicate(1.0f));

        constexpr int batchSize = JPH::Shape::cGetTrianglesMinTrianglesRequested;
        JPH::Float3 buffer[batchSize * 3];
        int triangleCount = 0;
        while ((triangleCount = m_nativeShape->GetTrianglesNext(context, batchSize, buffer)) > 0)
        {
            for (int i = 0; i < triangleCount * 3; ++i)
            {
                vertices.emplace_back(buffer[i].x, buffer[i].y, buffer[i].z);
                indices.push_back(static_cast<AZ::u32>(indices.size()));
            }
        }
    }

} // namespace JoltPhysics
