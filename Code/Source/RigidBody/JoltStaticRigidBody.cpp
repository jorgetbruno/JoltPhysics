#include <RigidBody/JoltStaticRigidBody.h>
#include <Scene/JoltScene.h>
#include <Utils/Conversions.h>
#include <Shape/JoltHeightfieldUtils.h>
#include <Shape/JoltShape.h>
#include <Shape/JoltShapeUtils.h>
#include <Material/JoltMaterial.h>
#include <Material/JoltMaterialManager.h>
#include <System/CollisionLayerFilters.h>

#include <AzCore/Interface/Interface.h>
#include <AzCore/Memory/SystemAllocator.h>

#include <AzFramework/Physics/HeightfieldProviderBus.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>

namespace JoltPhysics
{
    AZ_CLASS_ALLOCATOR_IMPL(JoltStaticRigidBody, AZ::SystemAllocator);

    JoltStaticRigidBody::JoltStaticRigidBody(const AzPhysics::StaticRigidBodyConfiguration& configuration)
        : m_configuration(configuration)
    {
        m_entityId = configuration.m_entityId;
    }

    JoltStaticRigidBody::~JoltStaticRigidBody()
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                if (!m_removedFromWorld)
                {
                    bodyInterface->RemoveBody(m_bodyId);
                }
                bodyInterface->DestroyBody(m_bodyId);
            }
        }
    }

    void JoltStaticRigidBody::RemoveFromJoltWorld()
    {
        if (m_scene && !m_bodyId.IsInvalid() && !m_removedFromWorld)
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                bodyInterface->RemoveBody(m_bodyId);
                m_removedFromWorld = true;
            }
        }
    }

    void JoltStaticRigidBody::SetSimulationEnabled(bool enabled)
    {
        if (!m_scene || m_bodyId.IsInvalid())
        {
            return;
        }

        auto* bodyInterface = m_scene->GetBodyInterface();
        if (!bodyInterface)
        {
            return;
        }

        if (enabled && m_removedFromWorld)
        {
            bodyInterface->AddBody(m_bodyId, JPH::EActivation::DontActivate);
            m_removedFromWorld = false;
        }
        else if (!enabled && !m_removedFromWorld)
        {
            bodyInterface->RemoveBody(m_bodyId);
            m_removedFromWorld = true;
        }
    }

    void JoltStaticRigidBody::CreateInScene(JoltScene* scene)
    {
        m_scene = scene;

        if (!scene || !scene->GetBodyInterface())
        {
            return;
        }

        JPH::RefConst<JPH::Shape> shape = JoltShapeUtils::CreateJoltShapeFromStatic(m_configuration);
        if (!shape)
        {
            shape = new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f));
        }
        m_baseShape = shape;

        // The collider's layer and group decide the body's object layer, so they have to
        // be resolved before the creation settings are built.
        const AzPhysics::ShapeColliderPairList colliderPairs =
            JoltShapeUtils::GetColliderPairList(m_configuration.m_colliderAndShapeData);
        const AZStd::vector<AZStd::shared_ptr<Physics::Shape>> prebuiltShapes =
            JoltShapeUtils::GetPrebuiltShapes(m_configuration.m_colliderAndShapeData);

        const Physics::ColliderConfiguration* firstCollider = nullptr;
        if (!colliderPairs.empty())
        {
            firstCollider = colliderPairs.front().first.get();
        }
        else if (!prebuiltShapes.empty())
        {
            if (const auto* joltShape = azrtti_cast<const JoltShape*>(prebuiltShapes.front().get()))
            {
                firstCollider = joltShape->GetColliderConfiguration();
            }
        }

        JPH::BodyCreationSettings bodySettings(
            shape,
            Conversions::ToJolt(m_configuration.m_position),
            Conversions::ToJolt(m_configuration.m_orientation),
            JPH::EMotionType::Static,
            AcquireObjectLayer(firstCollider, /*isMoving*/ false)
        );

        if (firstCollider)
        {
            bodySettings.mIsSensor = firstCollider->m_isTrigger;
            m_isSensor = firstCollider->m_isTrigger;
        }
        JoltShapeUtils::WarnOnMixedTriggerFlags(colliderPairs, m_configuration.m_debugName);

        m_colliderMaterials.reserve(colliderPairs.size() + prebuiltShapes.size());
        for (const auto& [colliderConfig, shapeConfig] : colliderPairs)
        {
            // See JoltRigidBody: a Physics::Shape per collider so query hits can name the
            // collider they came from, sharing geometry rather than re-cooking it.
            m_colliderMaterials.push_back(
                { colliderConfig && shapeConfig ? JoltShapeUtils::CreateShape(*colliderConfig, *shapeConfig) : nullptr,
                  colliderConfig ? JoltMaterialManager::ResolveMaterial(*colliderConfig) : nullptr });
        }
        for (const AZStd::shared_ptr<Physics::Shape>& prebuiltShape : prebuiltShapes)
        {
            m_colliderMaterials.push_back({ prebuiltShape, nullptr });
        }

        const auto [initialFriction, initialRestitution] =
            JoltMaterialManager::GetFrictionRestitution(GetColliderMaterial(0).get());
        bodySettings.mFriction = initialFriction;
        bodySettings.mRestitution = initialRestitution;

        bodySettings.mUserData = static_cast<AZ::u64>(m_entityId);

        auto* bodyInterface = scene->GetBodyInterface();
        m_bodyId = bodyInterface->CreateAndAddBody(bodySettings, JPH::EActivation::DontActivate);

        // Heightfield bodies get per-triangle materials from the provider.
        if (const JPH::HeightFieldShape* heightFieldShape = JoltHeightfieldUtils::UnwrapHeightField(shape))
        {
            ResolveHeightfieldMaterialData(heightFieldShape);
        }
    }

    size_t JoltStaticRigidBody::GetColliderCount() const
    {
        return m_colliderMaterials.size();
    }

    AZStd::shared_ptr<Physics::Material> JoltStaticRigidBody::GetColliderMaterial(size_t colliderIndex) const
    {
        return colliderIndex < m_colliderMaterials.size() ? m_colliderMaterials[colliderIndex].Get() : nullptr;
    }

    const Physics::ColliderConfiguration* JoltStaticRigidBody::GetColliderConfiguration(size_t colliderIndex) const
    {
        if (colliderIndex >= m_colliderMaterials.size())
        {
            return nullptr;
        }
        const auto* joltShape = azrtti_cast<const JoltShape*>(m_colliderMaterials[colliderIndex].m_shape.get());
        return joltShape ? joltShape->GetColliderConfiguration() : nullptr;
    }

    AZ::u32 JoltStaticRigidBody::GetShapeCount() const
    {
        return static_cast<AZ::u32>(m_colliderMaterials.size());
    }

    AZStd::shared_ptr<Physics::Shape> JoltStaticRigidBody::GetShape(AZ::u32 index)
    {
        return index < m_colliderMaterials.size() ? m_colliderMaterials[index].m_shape : nullptr;
    }

    AZStd::shared_ptr<const Physics::Shape> JoltStaticRigidBody::GetShape(AZ::u32 index) const
    {
        return index < m_colliderMaterials.size() ? m_colliderMaterials[index].m_shape : nullptr;
    }

    AZStd::shared_ptr<Physics::Shape> JoltStaticRigidBody::GetShapeFromSubShapeId(const JPH::SubShapeID& subShapeId) const
    {
        const size_t colliderIndex = JoltShapeUtils::GetColliderIndexFromSubShapeId(m_baseShape, subShapeId);
        return colliderIndex < m_colliderMaterials.size() ? m_colliderMaterials[colliderIndex].m_shape : nullptr;
    }

    void JoltStaticRigidBody::ResolveHeightfieldMaterialData(const JPH::HeightFieldShape* heightFieldShape)
    {
        AZStd::vector<AZ::Data::Asset<Physics::MaterialAsset>> providerMaterials;
        Physics::HeightfieldProviderRequestsBus::EventResult(
            providerMaterials, m_entityId, &Physics::HeightfieldProviderRequests::GetMaterialList);

        m_colliderMaterials.clear();
        for (const auto& materialAsset : providerMaterials)
        {
            m_colliderMaterials.push_back({ nullptr,
                AZ::Interface<Physics::MaterialManager>::Get()->FindOrCreateMaterial(
                    Physics::MaterialId::CreateFromAssetId(materialAsset.GetId()), materialAsset) });
        }

        size_t numColumns = 0;
        size_t numRows = 0;
        Physics::HeightfieldProviderRequestsBus::EventResult(
            numColumns, m_entityId, &Physics::HeightfieldProviderRequests::GetHeightfieldGridColumns);
        Physics::HeightfieldProviderRequestsBus::EventResult(
            numRows, m_entityId, &Physics::HeightfieldProviderRequests::GetHeightfieldGridRows);

        AZStd::vector<Physics::HeightMaterialPoint> heightsAndMaterials;
        Physics::HeightfieldProviderRequestsBus::EventResult(
            heightsAndMaterials, m_entityId, &Physics::HeightfieldProviderRequests::GetHeightsAndMaterials);

        AZStd::vector<AZ::u8> perSampleIndices;
        if (!heightsAndMaterials.empty())
        {
            perSampleIndices.reserve(heightsAndMaterials.size());
            for (const auto& point : heightsAndMaterials)
            {
                perSampleIndices.push_back(point.m_materialIndex);
            }
        }

        m_heightfieldMaterialIndices = JoltHeightfieldUtils::PadMaterialIndices(
            static_cast<AZ::u32>(numColumns), static_cast<AZ::u32>(numRows), perSampleIndices, heightFieldShape->GetSampleCount());
    }

    AZ::Vector3 JoltStaticRigidBody::GetPosition() const
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                return Conversions::FromJolt(bodyInterface->GetPosition(m_bodyId));
            }
        }
        return m_configuration.m_position;
    }

    AZ::Quaternion JoltStaticRigidBody::GetOrientation() const
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                return Conversions::FromJolt(bodyInterface->GetRotation(m_bodyId));
            }
        }
        return m_configuration.m_orientation;
    }

    AZ::Aabb JoltStaticRigidBody::GetAabb() const
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* physicsSystem = m_scene->GetJoltPhysicsSystem())
            {
                JPH::BodyLockRead bodyLock(physicsSystem->GetBodyLockInterface(), m_bodyId);
                if (bodyLock.Succeeded())
                {
                    const JPH::Body& body = bodyLock.GetBody();
                    JPH::AABox aabb = body.GetWorldSpaceBounds();
                    return AZ::Aabb::CreateFromMinMax(
                        Conversions::FromJolt(aabb.mMin),
                        Conversions::FromJolt(aabb.mMax)
                    );
                }
            }
        }
        return AZ::Aabb::CreateNull();
    }

    AZ::EntityId JoltStaticRigidBody::GetEntityId() const
    {
        return m_entityId;
    }

    AZ::Transform JoltStaticRigidBody::GetTransform() const
    {
        return AZ::Transform::CreateFromQuaternionAndTranslation(GetOrientation(), GetPosition());
    }

    void JoltStaticRigidBody::SetTransform(const AZ::Transform& transform)
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                bodyInterface->SetPositionAndRotation(
                    m_bodyId,
                    Conversions::ToJolt(transform.GetTranslation()),
                    Conversions::ToJolt(transform.GetRotation()),
                    JPH::EActivation::DontActivate
                );
            }
        }
    }

    void JoltStaticRigidBody::AddShape(AZStd::shared_ptr<Physics::Shape> shape)
    {
        auto* bodyInterface = m_scene ? m_scene->GetBodyInterface() : nullptr;
        if (!shape || !bodyInterface || m_bodyId.IsInvalid())
        {
            return;
        }

        auto* nativeShape = static_cast<JPH::Shape*>(shape->GetNativePointer());
        if (!nativeShape)
        {
            return;
        }

        // Copy the geometry into a fresh mutable compound (sub-shape order is preserved,
        // so the per-collider material indices stay valid) and append to it. A fresh copy
        // each time: the live body still references the current shape, and mutating a
        // shape in use leaves the body's cached bounds stale.
        JPH::Ref<JPH::MutableCompoundShape> compound = JoltShapeUtils::MakeMutableCompound(m_baseShape);

        const auto [localPosition, localRotation] = shape->GetLocalPose();
        compound->AddShape(Conversions::ToJolt(localPosition), Conversions::ToJolt(localRotation), nativeShape);
        compound->AdjustCenterOfMass();

        m_baseShape = compound;
        m_attachedShapes.push_back(shape);
        m_colliderMaterials.push_back({ shape, nullptr });

        // Static bodies have no mass properties to recompute and must not be woken.
        bodyInterface->SetShape(m_bodyId, compound, /*updateMassProperties*/ false, JPH::EActivation::DontActivate);
    }

    AZ::Crc32 JoltStaticRigidBody::GetNativeType() const
    {
        return AZ_CRC_CE("JoltStaticRigidBody");
    }

    void* JoltStaticRigidBody::GetNativePointer() const
    {
        return const_cast<JPH::BodyID*>(&m_bodyId);
    }

    AzPhysics::SceneQueryHit JoltStaticRigidBody::RayCast(const AzPhysics::RayCastRequest& request)
    {
        AzPhysics::SceneQueryHit queryHit;

        if (!m_scene || m_bodyId.IsInvalid())
        {
            return queryHit;
        }

        auto* physicsSystem = m_scene->GetJoltPhysicsSystem();
        if (!physicsSystem)
        {
            return queryHit;
        }

        JPH::BodyLockRead bodyLock(physicsSystem->GetBodyLockInterface(), m_bodyId);
        if (!bodyLock.Succeeded())
        {
            return queryHit;
        }
        const JPH::Body& body = bodyLock.GetBody();

        // Cast in body-local space (the shape is expressed relative to the center of mass).
        const JPH::Mat44 worldToLocal = body.GetInverseCenterOfMassTransform();
        const JPH::Vec3 localStart = worldToLocal * Conversions::ToJolt(request.m_start);
        const JPH::Vec3 localDirection =
            worldToLocal.Multiply3x3(Conversions::ToJolt(request.m_direction * request.m_distance));

        JPH::RayCast ray(localStart, localDirection);
        JPH::RayCastResult hit;
        if (!body.GetShape()->CastRay(ray, JPH::SubShapeIDCreator(), hit))
        {
            return queryHit;
        }

        queryHit.m_distance = hit.mFraction * request.m_distance;
        queryHit.m_position = request.m_start + request.m_direction * (hit.mFraction * request.m_distance);
        queryHit.m_normal = Conversions::FromJolt(
            body.GetWorldSpaceSurfaceNormal(hit.mSubShapeID2, Conversions::ToJolt(queryHit.m_position)));
        queryHit.m_resultFlags = AzPhysics::SceneQuery::ResultFlags::Distance |
            AzPhysics::SceneQuery::ResultFlags::Position | AzPhysics::SceneQuery::ResultFlags::Normal |
            AzPhysics::SceneQuery::ResultFlags::BodyHandle | AzPhysics::SceneQuery::ResultFlags::EntityId;
        queryHit.m_bodyHandle = m_bodyHandle;
        queryHit.m_entityId = m_entityId;
        return queryHit;
    }

} // namespace JoltPhysics
