#include <RigidBody/JoltRigidBody.h>
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
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>

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
                if (!m_removedFromWorld)
                {
                    bodyInterface->RemoveBody(m_bodyId);
                }
                bodyInterface->DestroyBody(m_bodyId);
            }
        }
    }

    void JoltRigidBody::RemoveFromJoltWorld()
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
        m_baseShape = shape;

        if (!m_configuration.m_centerOfMassOffset.IsZero())
        {
            shape = new JPH::RotatedTranslatedShape(
                Conversions::ToJolt(-m_configuration.m_centerOfMassOffset), JPH::Quat::sIdentity(), m_baseShape);
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

        const AzPhysics::ShapeColliderPairList colliderPairs =
            JoltShapeUtils::GetColliderPairList(m_configuration.m_colliderAndShapeData);

        if (!colliderPairs.empty() && colliderPairs.front().first)
        {
            const Physics::ColliderConfiguration& firstCollider = *colliderPairs.front().first;
            bodySettings.mCollisionGroup = CreateCollisionGroupFromConfig(firstCollider);
            bodySettings.mIsSensor = firstCollider.m_isTrigger;
            m_isSensor = firstCollider.m_isTrigger;
        }

        m_colliderMaterials.reserve(colliderPairs.size());
        for (const auto& [colliderConfig, shapeConfig] : colliderPairs)
        {
            if (colliderConfig)
            {
                m_colliderMaterials.push_back(JoltMaterialManager::ResolveFrictionRestitution(*colliderConfig));
            }
            else
            {
                m_colliderMaterials.emplace_back(JoltMaterial::DefaultFriction, JoltMaterial::DefaultRestitution);
            }
        }

        if (!m_colliderMaterials.empty())
        {
            bodySettings.mFriction = m_colliderMaterials.front().first;
            bodySettings.mRestitution = m_colliderMaterials.front().second;
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

    void JoltRigidBody::SetKinematicTarget(const AZ::Transform& targetPosition)
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                bodyInterface->MoveKinematic(
                    m_bodyId,
                    Conversions::ToJolt(targetPosition.GetTranslation()),
                    Conversions::ToJolt(targetPosition.GetRotation()),
                    m_scene->GetCurrentDeltaTime() > 0.0f ? m_scene->GetCurrentDeltaTime() : 1.0f / 60.0f
                );
            }
        }
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

    void JoltRigidBody::SetSimulationEnabled(bool enabled)
    {
        if (!m_scene || m_bodyId.IsInvalid() || enabled == m_simulationEnabled)
        {
            return;
        }

        if (auto* bodyInterface = m_scene->GetBodyInterface())
        {
            if (enabled)
            {
                bodyInterface->AddBody(m_bodyId, JPH::EActivation::Activate);
            }
            else
            {
                bodyInterface->RemoveBody(m_bodyId);
            }
            m_simulationEnabled = enabled;
        }
    }

    void JoltRigidBody::SetCCDEnabled(bool enabled)
    {
        if (m_scene && !m_bodyId.IsInvalid())
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                bodyInterface->SetMotionQuality(
                    m_bodyId,
                    enabled ? JPH::EMotionQuality::LinearCast : JPH::EMotionQuality::Discrete
                );
            }
        }
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

    void JoltRigidBody::SetMass(float mass)
    {
        m_configuration.m_mass = mass;
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
                        body.GetMotionProperties()->SetInverseMass(1.0f / AZStd::max(mass, 0.001f));
                    }
                }
            }
        }
    }

    void JoltRigidBody::SetCenterOfMassOffset(const AZ::Vector3& comOffset)
    {
        m_configuration.m_centerOfMassOffset = comOffset;

        if (!m_scene || m_bodyId.IsInvalid() || !m_baseShape)
        {
            return;
        }

        // Jolt-native semantics: the collision geometry is shifted by -offset around
        // the body pivot (which is also the center of mass Jolt integrates around).
        // NOTE: Jolt cannot express PhysX's "geometry fixed, mass frame moved" model;
        // see DIVERGENCES.md.
        JPH::RefConst<JPH::Shape> shiftedShape = m_baseShape;
        if (!comOffset.IsZero())
        {
            shiftedShape = new JPH::RotatedTranslatedShape(
                Conversions::ToJolt(-comOffset), JPH::Quat::sIdentity(), m_baseShape);
        }

        if (auto* bodyInterface = m_scene->GetBodyInterface())
        {
            bodyInterface->SetShape(m_bodyId, shiftedShape, true /* update mass properties */, JPH::EActivation::Activate);
        }
    }

    AZ::Matrix3x3 JoltRigidBody::GetInertiaLocal() const
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
                        const JPH::Vec3 inverseDiagonal = body.GetMotionProperties()->GetInverseInertiaDiagonal();
                        return AZ::Matrix3x3::CreateDiagonal(AZ::Vector3(
                            inverseDiagonal.GetX() > 0.0f ? 1.0f / inverseDiagonal.GetX() : 0.0f,
                            inverseDiagonal.GetY() > 0.0f ? 1.0f / inverseDiagonal.GetY() : 0.0f,
                            inverseDiagonal.GetZ() > 0.0f ? 1.0f / inverseDiagonal.GetZ() : 0.0f));
                    }
                }
            }
        }
        return m_configuration.m_inertiaTensor;
    }

    AZ::Matrix3x3 JoltRigidBody::GetInertiaWorld() const
    {
        return GetInverseInertiaWorld().GetInverseFull();
    }

    AZ::Matrix3x3 JoltRigidBody::GetInverseInertiaLocal() const
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
                        const JPH::Vec3 inverseDiagonal = body.GetMotionProperties()->GetInverseInertiaDiagonal();
                        return AZ::Matrix3x3::CreateDiagonal(Conversions::FromJolt(inverseDiagonal));
                    }
                }
            }
        }
        return m_configuration.m_inertiaTensor.GetInverseFull();
    }

    AZ::Matrix3x3 JoltRigidBody::GetInverseInertiaWorld() const
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
                        const JPH::Mat44 inverseInertia = body.GetMotionProperties()->GetInverseInertiaForRotation(
                            JPH::Mat44::sRotation(body.GetRotation()));
                        return AZ::Matrix3x3::CreateFromRows(
                            AZ::Vector3(inverseInertia(0, 0), inverseInertia(0, 1), inverseInertia(0, 2)),
                            AZ::Vector3(inverseInertia(1, 0), inverseInertia(1, 1), inverseInertia(1, 2)),
                            AZ::Vector3(inverseInertia(2, 0), inverseInertia(2, 1), inverseInertia(2, 2)));
                    }
                }
            }
        }
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
        AzPhysics::MassComputeFlags flags,
        const AZ::Vector3& centerOfMassOffsetOverride,
        const AZ::Matrix3x3& inertiaTensorOverride,
        const float massOverride)
    {
        if ((flags & AzPhysics::MassComputeFlags::COMPUTE_COM) == AzPhysics::MassComputeFlags::COMPUTE_COM)
        {
            SetCenterOfMassOffset(centerOfMassOffsetOverride);
        }

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
                        if ((flags & AzPhysics::MassComputeFlags::COMPUTE_MASS) == AzPhysics::MassComputeFlags::COMPUTE_MASS)
                        {
                            m_configuration.m_mass = massOverride;
                            body.GetMotionProperties()->SetInverseMass(1.0f / AZStd::max(massOverride, 0.001f));
                        }
                        if ((flags & AzPhysics::MassComputeFlags::COMPUTE_INERTIA) == AzPhysics::MassComputeFlags::COMPUTE_INERTIA)
                        {
                            // Jolt stores a diagonal local inertia; off-diagonal elements are not supported.
                            body.GetMotionProperties()->SetInverseInertia(JPH::Vec3(
                                inertiaTensorOverride(0, 0) > 0.0f ? 1.0f / inertiaTensorOverride(0, 0) : 0.0f,
                                inertiaTensorOverride(1, 1) > 0.0f ? 1.0f / inertiaTensorOverride(1, 1) : 0.0f,
                                inertiaTensorOverride(2, 2) > 0.0f ? 1.0f / inertiaTensorOverride(2, 2) : 0.0f),
                                body.GetMotionProperties()->GetInertiaRotation());
                        }
                    }
                }
            }
        }
    }

    AZ::Crc32 JoltRigidBody::GetNativeType() const
    {
        return AZ_CRC_CE("JoltRigidBody");
    }

    void* JoltRigidBody::GetNativePointer() const
    {
        return const_cast<JPH::BodyID*>(&m_bodyId);
    }

    AzPhysics::SceneQueryHit JoltRigidBody::RayCast(const AzPhysics::RayCastRequest& request)
    {
        AzPhysics::SceneQueryHit queryHit;

        if (!m_scene || m_bodyId.IsInvalid())
        {
            return queryHit;
        }

        if (auto* physicsSystem = m_scene->GetJoltPhysicsSystem())
        {
            JPH::BodyLockRead bodyLock(physicsSystem->GetBodyLockInterface(), m_bodyId);
            if (bodyLock.Succeeded())
            {
                const JPH::Body& body = bodyLock.GetBody();

                // Cast in body-local space.
                const JPH::Mat44 worldToLocal = body.GetInverseCenterOfMassTransform();
                const JPH::Vec3 localStart = worldToLocal * Conversions::ToJolt(request.m_start);
                const JPH::Vec3 localDirection = worldToLocal.Multiply3x3(Conversions::ToJolt(request.m_direction * request.m_distance));

                JPH::RayCast ray(localStart, localDirection);
                JPH::RayCastResult hit;
                if (body.GetShape()->CastRay(ray, JPH::SubShapeIDCreator(), hit))
                {
                    queryHit.m_distance = hit.mFraction * request.m_distance;
                    queryHit.m_position = request.m_start + request.m_direction * (hit.mFraction * request.m_distance);
                    queryHit.m_normal = Conversions::FromJolt(
                        body.GetWorldSpaceSurfaceNormal(hit.mSubShapeID2, Conversions::ToJolt(queryHit.m_position)));
                    queryHit.m_resultFlags = AzPhysics::SceneQuery::ResultFlags::Distance |
                                            AzPhysics::SceneQuery::ResultFlags::Position |
                                            AzPhysics::SceneQuery::ResultFlags::Normal |
                                            AzPhysics::SceneQuery::ResultFlags::BodyHandle |
                                            AzPhysics::SceneQuery::ResultFlags::EntityId;
                    queryHit.m_bodyHandle = m_bodyHandle;
                    queryHit.m_entityId = m_entityId;
                }
            }
        }

        return queryHit;
    }

} // namespace JoltPhysics
