#include <RigidBody/JoltRigidBody.h>
#include <Scene/JoltScene.h>
#include <Utils/Conversions.h>
#include <Shape/JoltShapeUtils.h>
#include <System/CollisionLayerFilters.h>

#include <AzCore/Memory/SystemAllocator.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

namespace JoltPhysics
{
    AZ_CLASS_ALLOCATOR_IMPL(JoltRigidBody, AZ::SystemAllocator);

    JoltRigidBody::JoltRigidBody(const AzPhysics::RigidBodyConfiguration& configuration)
        : m_configuration(configuration)
        , m_isKinematic(configuration.m_kinematic)
    {
        m_entityId = configuration.m_entityId;
    }

    JoltRigidBody::~JoltRigidBody()
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

    void JoltRigidBody::CreateInScene(JoltScene* scene)
    {
        m_scene = scene;

        if (!scene || !scene->GetBodyInterface())
        {
            return;
        }

        JPH::RefConst<JPH::Shape> shape = JoltShapeUtils::CreateJoltShape(m_configuration);
        if (!shape)
        {
            shape = new JPH::SphereShape(0.5f);
        }

        JPH::EMotionType motionType = m_isKinematic
            ? JPH::EMotionType::Kinematic
            : JPH::EMotionType::Dynamic;

        JPH::ObjectLayer objectLayer = m_isKinematic
            ? ObjectLayers::Moving
            : ObjectLayers::Moving;

        JPH::BodyCreationSettings bodySettings(
            shape,
            Conversions::ToJolt(m_configuration.m_position),
            Conversions::ToJolt(m_configuration.m_orientation),
            motionType,
            objectLayer
        );

        bodySettings.mLinearVelocity = Conversions::ToJolt(m_configuration.m_initialLinearVelocity);
        bodySettings.mAngularVelocity = Conversions::ToJolt(m_configuration.m_initialAngularVelocity);
        bodySettings.mLinearDamping = m_configuration.m_linearDamping;
        bodySettings.mAngularDamping = m_configuration.m_angularDamping;
        bodySettings.mMaxAngularVelocity = m_configuration.m_maxAngularVelocity;
        bodySettings.mGravityFactor = m_configuration.m_gravityEnabled ? 1.0f : 0.0f;

        if (m_configuration.m_mass > 0.0f)
        {
            bodySettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            bodySettings.mMassPropertiesOverride.mMass = m_configuration.m_mass;
        }

        bodySettings.mAllowSleeping = m_configuration.m_sleepMinEnergy > 0.0f;

        if (const Physics::ColliderConfiguration* colliderConfiguration =
                JoltShapeUtils::GetFirstColliderConfiguration(m_configuration.m_colliderAndShapeData))
        {
            bodySettings.mCollisionGroup = CreateCollisionGroupFromConfig(*colliderConfiguration);
            bodySettings.mIsSensor = colliderConfiguration->m_isTrigger;
        }

        bodySettings.mUserData = static_cast<AZ::u64>(m_entityId);

        auto* bodyInterface = scene->GetBodyInterface();
        m_bodyId = bodyInterface->CreateAndAddBody(bodySettings, JPH::EActivation::Activate);
    }

    void JoltRigidBody::SyncTransform()
    {
        // Transform sync is handled by Jolt's simulation
    }

    AZ::Vector3 JoltRigidBody::GetPosition() const
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

    AZ::Quaternion JoltRigidBody::GetOrientation() const
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

    AZ::Aabb JoltRigidBody::GetAabb() const
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

    AZ::EntityId JoltRigidBody::GetEntityId() const
    {
        return m_entityId;
    }

    AZ::Transform JoltRigidBody::GetTransform() const
    {
        return AZ::Transform::CreateFromQuaternionAndTranslation(GetOrientation(), GetPosition());
    }

    void JoltRigidBody::SetTransform(const AZ::Transform& transform)
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                bodyInterface->SetPositionAndRotation(
                    m_bodyId,
                    Conversions::ToJolt(transform.GetTranslation()),
                    Conversions::ToJolt(transform.GetRotation()),
                    JPH::EActivation::Activate
                );
            }
        }
    }

    void JoltRigidBody::SetKinematic(bool isKinematic)
    {
        m_isKinematic = isKinematic;
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                bodyInterface->SetMotionType(
                    m_bodyId,
                    isKinematic ? JPH::EMotionType::Kinematic : JPH::EMotionType::Dynamic,
                    JPH::EActivation::Activate
                );
            }
        }
    }

    bool JoltRigidBody::IsKinematic() const
    {
        return m_isKinematic;
    }

    void JoltRigidBody::SetKinematicTarget([[maybe_unused]] const AZ::Transform& targetPosition)
    {
        // TODO: Implement kinematic target
    }

    bool JoltRigidBody::IsGravityEnabled() const
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                return bodyInterface->GetGravityFactor(m_bodyId) > 0.0f;
            }
        }
        return m_configuration.m_gravityEnabled;
    }

    void JoltRigidBody::SetGravityEnabled(bool enabled)
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                bodyInterface->SetGravityFactor(m_bodyId, enabled ? 1.0f : 0.0f);
            }
        }
    }

    void JoltRigidBody::SetSimulationEnabled([[maybe_unused]] bool enabled)
    {
        // TODO: Implement simulation enable/disable
    }

    void JoltRigidBody::SetCCDEnabled([[maybe_unused]] bool enabled)
    {
        // TODO: Implement CCD toggle
    }

    AZ::Vector3 JoltRigidBody::GetLinearVelocity() const
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                return Conversions::FromJolt(bodyInterface->GetLinearVelocity(m_bodyId));
            }
        }
        return AZ::Vector3::CreateZero();
    }

    void JoltRigidBody::SetLinearVelocity(const AZ::Vector3& velocity)
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                bodyInterface->SetLinearVelocity(m_bodyId, Conversions::ToJolt(velocity));
            }
        }
    }

    AZ::Vector3 JoltRigidBody::GetAngularVelocity() const
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                return Conversions::FromJolt(bodyInterface->GetAngularVelocity(m_bodyId));
            }
        }
        return AZ::Vector3::CreateZero();
    }

    void JoltRigidBody::SetAngularVelocity(const AZ::Vector3& angularVelocity)
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                bodyInterface->SetAngularVelocity(m_bodyId, Conversions::ToJolt(angularVelocity));
            }
        }
    }

    AZ::Vector3 JoltRigidBody::GetLinearVelocityAtWorldPoint(const AZ::Vector3& worldPoint) const
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* physicsSystem = m_scene->GetJoltPhysicsSystem())
            {
                JPH::BodyLockRead bodyLock(physicsSystem->GetBodyLockInterface(), m_bodyId);
                if (bodyLock.Succeeded())
                {
                    const JPH::Body& body = bodyLock.GetBody();
                    return Conversions::FromJolt(body.GetPointVelocity(Conversions::ToJolt(worldPoint)));
                }
            }
        }
        return AZ::Vector3::CreateZero();
    }

    void JoltRigidBody::ApplyLinearImpulse(const AZ::Vector3& impulse)
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                bodyInterface->AddImpulse(m_bodyId, Conversions::ToJolt(impulse));
            }
        }
    }

    void JoltRigidBody::ApplyLinearImpulseAtWorldPoint(const AZ::Vector3& impulse, const AZ::Vector3& worldPoint)
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                bodyInterface->AddImpulse(m_bodyId, Conversions::ToJolt(impulse), Conversions::ToJolt(worldPoint));
            }
        }
    }

    void JoltRigidBody::ApplyAngularImpulse(const AZ::Vector3& angularImpulse)
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                bodyInterface->AddAngularImpulse(m_bodyId, Conversions::ToJolt(angularImpulse));
            }
        }
    }

    float JoltRigidBody::GetMass() const
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* physicsSystem = m_scene->GetJoltPhysicsSystem())
            {
                JPH::BodyLockRead bodyLock(physicsSystem->GetBodyLockInterface(), m_bodyId);
                if (bodyLock.Succeeded())
                {
                    const JPH::Body& body = bodyLock.GetBody();
                    if (body.GetMotionProperties())
                    {
                        return 1.0f / body.GetMotionProperties()->GetInverseMass();
                    }
                }
            }
        }
        return m_configuration.m_mass;
    }

    float JoltRigidBody::GetInverseMass() const
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* physicsSystem = m_scene->GetJoltPhysicsSystem())
            {
                JPH::BodyLockRead bodyLock(physicsSystem->GetBodyLockInterface(), m_bodyId);
                if (bodyLock.Succeeded())
                {
                    const JPH::Body& body = bodyLock.GetBody();
                    if (body.GetMotionProperties())
                    {
                        return body.GetMotionProperties()->GetInverseMass();
                    }
                }
            }
        }
        return m_configuration.m_mass > 0.0f ? 1.0f / m_configuration.m_mass : 0.0f;
    }

    void JoltRigidBody::SetMass([[maybe_unused]] float mass)
    {
        // TODO: Implement mass update
    }

    void JoltRigidBody::SetCenterOfMassOffset([[maybe_unused]] const AZ::Vector3& comOffset)
    {
        // TODO: Implement center of mass offset
    }

    AZ::Matrix3x3 JoltRigidBody::GetInertiaLocal() const
    {
        // TODO: Return actual inertia tensor
        return AZ::Matrix3x3::CreateIdentity();
    }

    AZ::Matrix3x3 JoltRigidBody::GetInertiaWorld() const
    {
        // TODO: Return actual world-space inertia tensor
        return AZ::Matrix3x3::CreateIdentity();
    }

    AZ::Matrix3x3 JoltRigidBody::GetInverseInertiaLocal() const
    {
        // TODO: Return actual inertia tensor
        return AZ::Matrix3x3::CreateIdentity();
    }

    AZ::Matrix3x3 JoltRigidBody::GetInverseInertiaWorld() const
    {
        // TODO: Return actual world-space inertia tensor
        return AZ::Matrix3x3::CreateIdentity();
    }

    void JoltRigidBody::SetLinearDamping(float damping)
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* physicsSystem = m_scene->GetJoltPhysicsSystem())
            {
                JPH::BodyLockWrite bodyLock(physicsSystem->GetBodyLockInterface(), m_bodyId);
                if (bodyLock.Succeeded())
                {
                    JPH::Body& body = bodyLock.GetBody();
                    if (body.GetMotionProperties())
                    {
                        body.GetMotionProperties()->SetLinearDamping(damping);
                    }
                }
            }
        }
    }

    float JoltRigidBody::GetLinearDamping() const
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* physicsSystem = m_scene->GetJoltPhysicsSystem())
            {
                JPH::BodyLockRead bodyLock(physicsSystem->GetBodyLockInterface(), m_bodyId);
                if (bodyLock.Succeeded())
                {
                    const JPH::Body& body = bodyLock.GetBody();
                    if (body.GetMotionProperties())
                    {
                        return body.GetMotionProperties()->GetLinearDamping();
                    }
                }
            }
        }
        return m_configuration.m_linearDamping;
    }

    void JoltRigidBody::SetAngularDamping(float damping)
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* physicsSystem = m_scene->GetJoltPhysicsSystem())
            {
                JPH::BodyLockWrite bodyLock(physicsSystem->GetBodyLockInterface(), m_bodyId);
                if (bodyLock.Succeeded())
                {
                    JPH::Body& body = bodyLock.GetBody();
                    if (body.GetMotionProperties())
                    {
                        body.GetMotionProperties()->SetAngularDamping(damping);
                    }
                }
            }
        }
    }

    float JoltRigidBody::GetAngularDamping() const
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* physicsSystem = m_scene->GetJoltPhysicsSystem())
            {
                JPH::BodyLockRead bodyLock(physicsSystem->GetBodyLockInterface(), m_bodyId);
                if (bodyLock.Succeeded())
                {
                    const JPH::Body& body = bodyLock.GetBody();
                    if (body.GetMotionProperties())
                    {
                        return body.GetMotionProperties()->GetAngularDamping();
                    }
                }
            }
        }
        return m_configuration.m_angularDamping;
    }

    bool JoltRigidBody::IsAwake() const
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                return bodyInterface->IsActive(m_bodyId);
            }
        }
        return true;
    }

    void JoltRigidBody::ForceAsleep()
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                bodyInterface->DeactivateBody(m_bodyId);
            }
        }
    }

    void JoltRigidBody::ForceAwake()
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                bodyInterface->ActivateBody(m_bodyId);
            }
        }
    }

    float JoltRigidBody::GetSleepThreshold() const
    {
        return m_configuration.m_sleepMinEnergy;
    }

    void JoltRigidBody::SetSleepThreshold([[maybe_unused]] float threshold)
    {
        // TODO: Implement sleep threshold
    }

    AZ::Vector3 JoltRigidBody::GetCenterOfMassWorld() const
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                return Conversions::FromJolt(bodyInterface->GetCenterOfMassPosition(m_bodyId));
            }
        }
        return GetPosition();
    }

    AZ::Vector3 JoltRigidBody::GetCenterOfMassLocal() const
    {
        return m_configuration.m_centerOfMassOffset;
    }

    void JoltRigidBody::AddShape([[maybe_unused]] AZStd::shared_ptr<Physics::Shape> shape)
    {
        // TODO: Implement adding shapes to a created body
    }

    void JoltRigidBody::RemoveShape([[maybe_unused]] AZStd::shared_ptr<Physics::Shape> shape)
    {
        // TODO: Implement removing shapes from a created body
    }

    void JoltRigidBody::UpdateMassProperties(
        [[maybe_unused]] AzPhysics::MassComputeFlags flags,
        [[maybe_unused]] const AZ::Vector3& centerOfMassOffsetOverride,
        [[maybe_unused]] const AZ::Matrix3x3& inertiaTensorOverride,
        [[maybe_unused]] const float massOverride)
    {
        // TODO: Implement mass properties update
    }

    AZ::Crc32 JoltRigidBody::GetNativeType() const
    {
        return AZ_CRC_CE("JoltRigidBody");
    }

    void* JoltRigidBody::GetNativePointer() const
    {
        return const_cast<JPH::BodyID*>(&m_bodyId);
    }

    AzPhysics::SceneQueryHit JoltRigidBody::RayCast([[maybe_unused]] const AzPhysics::RayCastRequest& request)
    {
        // TODO: Implement per-body raycast
        return AzPhysics::SceneQueryHit();
    }

} // namespace JoltPhysics
