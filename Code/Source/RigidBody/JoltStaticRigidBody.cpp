#include <RigidBody/JoltStaticRigidBody.h>
#include <Scene/JoltScene.h>
#include <Utils/Conversions.h>
#include <Shape/JoltShapeUtils.h>
#include <Material/JoltMaterialManager.h>
#include <System/CollisionLayerFilters.h>

#include <AzCore/Memory/SystemAllocator.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

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
                bodyInterface->RemoveBody(m_bodyId);
                bodyInterface->DestroyBody(m_bodyId);
            }
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

        if (const Physics::ColliderConfiguration* colliderConfiguration =
                JoltShapeUtils::GetFirstColliderConfiguration(m_configuration.m_colliderAndShapeData))
        {
            bodySettings.mCollisionGroup = CreateCollisionGroupFromConfig(*colliderConfiguration);
            bodySettings.mIsSensor = colliderConfiguration->m_isTrigger;
            m_isSensor = colliderConfiguration->m_isTrigger;

            const auto [friction, restitution] = JoltMaterialManager::ResolveFrictionRestitution(*colliderConfiguration);
            bodySettings.mFriction = friction;
            bodySettings.mRestitution = restitution;
        }

        bodySettings.mUserData = static_cast<AZ::u64>(m_entityId);

        auto* bodyInterface = scene->GetBodyInterface();
        m_bodyId = bodyInterface->CreateAndAddBody(bodySettings, JPH::EActivation::DontActivate);
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
