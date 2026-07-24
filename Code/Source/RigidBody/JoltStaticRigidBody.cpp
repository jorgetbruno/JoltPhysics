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

        JPH::BodyCreationSettings bodySettings(
            shape,
            Conversions::ToJolt(m_configuration.m_position),
            Conversions::ToJolt(m_configuration.m_orientation),
            JPH::EMotionType::Static,
            ObjectLayers::NonMoving
        );

        const AzPhysics::ShapeColliderPairList colliderPairs =
            JoltShapeUtils::GetColliderPairList(m_configuration.m_colliderAndShapeData);
        m_prebuiltShapes = JoltShapeUtils::GetPrebuiltShapes(m_configuration.m_colliderAndShapeData);

        const Physics::ColliderConfiguration* firstCollider = nullptr;
        if (!colliderPairs.empty())
        {
            firstCollider = colliderPairs.front().first.get();
        }
        else if (!m_prebuiltShapes.empty())
        {
            if (const auto* joltShape = azrtti_cast<const JoltShape*>(m_prebuiltShapes.front().get()))
            {
                firstCollider = joltShape->GetColliderConfiguration();
            }
        }

        if (firstCollider)
        {
            bodySettings.mCollisionGroup = CreateCollisionGroupFromConfig(*firstCollider);
            bodySettings.mIsSensor = firstCollider->m_isTrigger;
            m_isSensor = firstCollider->m_isTrigger;
        }

        m_colliderMaterials.reserve(colliderPairs.size());
        for (const auto& [colliderConfig, shapeConfig] : colliderPairs)
        {
            m_colliderMaterials.push_back(
                colliderConfig ? JoltMaterialManager::ResolveMaterial(*colliderConfig) : nullptr);
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
        return AZStd::max(m_colliderMaterials.size(), m_prebuiltShapes.size());
    }

    AZStd::shared_ptr<Physics::Material> JoltStaticRigidBody::GetColliderMaterial(size_t colliderIndex) const
    {
        // Prebuilt shapes own their material and may be given a new one at any time
        // (Physics::Shape::SetMaterial), so read through the shape.
        if (colliderIndex < m_prebuiltShapes.size())
        {
            return m_prebuiltShapes[colliderIndex]->GetMaterial();
        }
        if (colliderIndex < m_colliderMaterials.size())
        {
            return m_colliderMaterials[colliderIndex];
        }
        return nullptr;
    }

    void JoltStaticRigidBody::ResolveHeightfieldMaterialData(const JPH::HeightFieldShape* heightFieldShape)
    {
        AZStd::vector<AZ::Data::Asset<Physics::MaterialAsset>> providerMaterials;
        Physics::HeightfieldProviderRequestsBus::EventResult(
            providerMaterials, m_entityId, &Physics::HeightfieldProviderRequests::GetMaterialList);

        m_colliderMaterials.clear();
        for (const auto& materialAsset : providerMaterials)
        {
            m_colliderMaterials.push_back(AZ::Interface<Physics::MaterialManager>::Get()->FindOrCreateMaterial(
                Physics::MaterialId::CreateFromAssetId(materialAsset.GetId()), materialAsset));
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

    void JoltStaticRigidBody::AddShape([[maybe_unused]] AZStd::shared_ptr<Physics::Shape> shape)
    {
        // TODO: Implement adding shapes to a created body
    }

    AZ::Crc32 JoltStaticRigidBody::GetNativeType() const
    {
        return AZ_CRC_CE("JoltStaticRigidBody");
    }

    void* JoltStaticRigidBody::GetNativePointer() const
    {
        return const_cast<JPH::BodyID*>(&m_bodyId);
    }

    AzPhysics::SceneQueryHit JoltStaticRigidBody::RayCast([[maybe_unused]] const AzPhysics::RayCastRequest& request)
    {
        // TODO: Implement per-body raycast
        return AzPhysics::SceneQueryHit();
    }

} // namespace JoltPhysics
