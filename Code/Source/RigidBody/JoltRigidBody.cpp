#include <RigidBody/JoltRigidBody.h>
#include <Scene/JoltScene.h>
#include <Utils/Conversions.h>
#include <Shape/JoltShape.h>
#include <Shape/JoltShapeUtils.h>
#include <Material/JoltMaterial.h>
#include <Material/JoltMaterialManager.h>
#include <System/CollisionLayerFilters.h>
#include <System/JoltSystem.h>

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/std/algorithm.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>

#include <Utils/JoltDiagnostics.h>

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
        // An adopted body is owned by something else (e.g. a JPH::Ragdoll); never remove
        // or destroy it here.
        if (!m_adopted && m_scene && !m_bodyId.IsInvalid())
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

    void JoltRigidBody::AdoptBody(JoltScene* scene, const JPH::BodyID& bodyId, AZ::EntityId entityId)
    {
        m_scene = scene;
        m_bodyId = bodyId;
        m_entityId = entityId;
        m_adopted = true;
    }

    void JoltRigidBody::RemoveFromJoltWorld()
    {
        if (!m_adopted && m_scene && !m_bodyId.IsInvalid() && !m_removedFromWorld)
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
            motionType,
            AcquireObjectLayer(firstCollider, /*isMoving*/ true)
        );

        bodySettings.mLinearVelocity = Conversions::ToJolt(m_configuration.m_initialLinearVelocity);
        bodySettings.mAngularVelocity = Conversions::ToJolt(m_configuration.m_initialAngularVelocity);
        bodySettings.mLinearDamping = m_configuration.m_linearDamping;
        bodySettings.mAngularDamping = m_configuration.m_angularDamping;
        bodySettings.mMaxAngularVelocity = m_configuration.m_maxAngularVelocity;
        bodySettings.mGravityFactor = m_configuration.m_gravityEnabled ? 1.0f : 0.0f;
        // Only the moving body needs the flag: Jolt ORs it across the contact pair, so
        // this covers a dynamic body sliding over a static mesh or heightfield.
        bodySettings.mEnhancedInternalEdgeRemoval = UseEnhancedInternalEdgeRemoval();

        // Mass: computed from the geometry, or taken from the configuration. The engine
        // defaults m_computeMass to true and PhysX honours it, so a body ported from PhysX
        // carries a meaningless m_mass of 1 kg that used to be applied verbatim here -
        // every crate in a migrated level weighing the same as a loaf of bread.
        //
        // CalculateInertia either way: Jolt derives the inertia tensor from the shape and
        // scales it to whichever mass it is given, which is what both branches want.
        const float configuredMass = m_configuration.m_computeMass ? 0.0f : m_configuration.m_mass;
        const float resolvedMass = configuredMass > 0.0f ? configuredMass : ComputeMassFromGeometry();
        if (resolvedMass > 0.0f)
        {
            bodySettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            bodySettings.mMassPropertiesOverride.mMass = resolvedMass;
            // Keep the configuration honest, so GetMass and a later save agree with the body.
            m_configuration.m_mass = resolvedMass;
        }

        bodySettings.mAllowSleeping = m_configuration.m_sleepMinEnergy > 0.0f;

        if (firstCollider)
        {
            bodySettings.mIsSensor = firstCollider->m_isTrigger;
            m_isSensor = firstCollider->m_isTrigger;
        }
        JoltShapeUtils::WarnOnMixedTriggerFlags(colliderPairs, m_configuration.m_debugName);

        m_colliderMaterials.reserve(colliderPairs.size() + prebuiltShapes.size());
        for (const auto& [colliderConfig, shapeConfig] : colliderPairs)
        {
            // A Physics::Shape per collider, so scene-query hits can name the collider they
            // came from. JoltShape resolves its material the same way this used to directly,
            // so GetColliderMaterial is unaffected. Cooked meshes cache their native shape on
            // the configuration, so this shares geometry with the body's compound rather than
            // cooking it a second time.
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
        m_bodyId = bodyInterface->CreateAndAddBody(bodySettings, JPH::EActivation::Activate);
    }

    void JoltRigidBody::SyncTransform()
    {
        // Transform sync is handled by Jolt's simulation
    }

    void JoltRigidBody::SetObjectLayerFrom(
        const AzPhysics::CollisionLayer& collisionLayer, const AzPhysics::CollisionGroups::Id& collisionGroupId)
    {
        auto* bodyInterface = m_scene ? m_scene->GetBodyInterface() : nullptr;
        if (!bodyInterface || m_bodyId.IsInvalid())
        {
            return;
        }
        bodyInterface->SetObjectLayer(
            m_bodyId, AcquireObjectLayer(collisionLayer, collisionGroupId, /*isMoving*/ true));
    }

    size_t JoltRigidBody::GetColliderCount() const
    {
        return m_colliderMaterials.size();
    }

    AZStd::shared_ptr<Physics::Material> JoltRigidBody::GetColliderMaterial(size_t colliderIndex) const
    {
        return colliderIndex < m_colliderMaterials.size() ? m_colliderMaterials[colliderIndex].Get() : nullptr;
    }

    const Physics::ColliderConfiguration* JoltRigidBody::GetColliderConfiguration(size_t colliderIndex) const
    {
        if (colliderIndex >= m_colliderMaterials.size())
        {
            return nullptr;
        }
        const auto* joltShape = azrtti_cast<const JoltShape*>(m_colliderMaterials[colliderIndex].m_shape.get());
        return joltShape ? joltShape->GetColliderConfiguration() : nullptr;
    }

    AZ::u32 JoltRigidBody::GetShapeCount() const
    {
        return static_cast<AZ::u32>(m_colliderMaterials.size());
    }

    AZStd::shared_ptr<Physics::Shape> JoltRigidBody::GetShape(AZ::u32 index)
    {
        return index < m_colliderMaterials.size() ? m_colliderMaterials[index].m_shape : nullptr;
    }

    AZStd::shared_ptr<const Physics::Shape> JoltRigidBody::GetShape(AZ::u32 index) const
    {
        return index < m_colliderMaterials.size() ? m_colliderMaterials[index].m_shape : nullptr;
    }

    AZStd::shared_ptr<Physics::Shape> JoltRigidBody::GetShapeFromSubShapeId(const JPH::SubShapeID& subShapeId) const
    {
        const size_t colliderIndex =
            JoltShapeUtils::GetColliderIndexFromSubShapeId(m_baseShape, subShapeId, m_colliderMaterials.size());
        return colliderIndex < m_colliderMaterials.size() ? m_colliderMaterials[colliderIndex].m_shape : nullptr;
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
            // Keep the world-membership flag in sync so the destructor and
            // RemoveFromJoltWorld don't remove a body that is not in the world.
            m_removedFromWorld = !enabled;
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
        // A kinematic body does not answer to impulses, and Jolt asserts rather than
        // ignores - PhysX ignores, and so does this. Non-finite impulses are refused for
        // the same reason: Jolt would assert, and a NaN says the *caller* is broken.
        if (IsKinematic() || !impulse.IsFinite())
        {
            AZ_Warning("JoltPhysics", !IsKinematic() || impulse.IsFinite(),
                "Ignoring a non-finite impulse on entity %s.", GetEntityId().ToString().c_str());
            return;
        }

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
                        // Unchecked on purpose: the checked accessor asserts on any
                        // non-dynamic body, and a kinematic body's mass is a fair
                        // question with a sensible answer - its configured mass, since
                        // its inverse is zero by definition rather than by data.
                        const float inverseMass = body.GetMotionProperties()->GetInverseMassUnchecked();
                        if (inverseMass > 0.0f)
                        {
                            return 1.0f / inverseMass;
                        }
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
        ApplyBaseShapeToBody();
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

    void JoltRigidBody::SetSleepThreshold(float threshold)
    {
        m_configuration.m_sleepMinEnergy = threshold;

        // Jolt has no per-body sleep threshold: the velocity below which a body may sleep
        // is a scene-wide setting (PhysicsSettings::mPointVelocitySleepThreshold). What is
        // per-body is whether sleeping is allowed at all, so a threshold of zero (or less)
        // is honoured as "never sleep" and any positive value as "may sleep". The magnitude
        // itself is not applied; see DIVERGENCES.md.
        AZ_WarningOnce("JoltPhysics", threshold <= 0.0f,
            "JoltRigidBody::SetSleepThreshold%s: Jolt's sleep velocity threshold is scene-wide, so the value "
            "%.3f only toggles whether this body may sleep. Configure the threshold on the scene instead.",
            Internal::NameClause(m_configuration.m_debugName).c_str(), threshold);

        if (!m_scene || m_bodyId.IsInvalid())
        {
            return;
        }

        if (auto* physicsSystem = m_scene->GetJoltPhysicsSystem())
        {
            JPH::BodyLockWrite bodyLock(physicsSystem->GetBodyLockInterface(), m_bodyId);
            if (!bodyLock.Succeeded())
            {
                return;
            }
            bodyLock.GetBody().SetAllowSleeping(threshold > 0.0f);
        }

        // A body that may no longer sleep should not be left asleep either. Activating
        // takes body locks of its own, so it has to happen after the lock above is released.
        if (threshold <= 0.0f)
        {
            if (auto* bodyInterface = m_scene->GetBodyInterface())
            {
                bodyInterface->ActivateBody(m_bodyId);
            }
        }
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

    JPH::Ref<JPH::MutableCompoundShape> JoltRigidBody::CloneBaseShapeAsMutableCompound() const
    {
        // A fresh copy every time: the live body still references the current shape, and
        // mutating a shape in use leaves the body's cached bounds and mass properties stale.
        //
        // The collider count tells MakeMutableCompound whether a compound base is several
        // colliders or one collider's own hull group; see its comment for why that matters.
        const size_t createdColliderCount = m_colliderMaterials.size() - m_attachedShapes.size();
        return JoltShapeUtils::MakeMutableCompound(m_baseShape, createdColliderCount);
    }

    float JoltRigidBody::ComputeMassFromGeometry() const
    {
        // Jolt builds shapes at its own default density, so a shape's computed mass is
        // volume * JPH default density - divide it back out to recover the volume, then
        // apply the collider's real material density. The two defaults are both
        // 1000 kg/m^3, so an unauthored material scales by exactly one.
        constexpr float JoltDefaultShapeDensity = 1000.0f;

        float totalMass = 0.0f;
        for (const JoltColliderMaterial& entry : m_colliderMaterials)
        {
            const auto* joltShape = azrtti_cast<const JoltShape*>(entry.m_shape.get());
            if (!joltShape)
            {
                continue;
            }
            const auto* nativeShape = static_cast<const JPH::Shape*>(joltShape->GetNativePointer());
            if (!nativeShape)
            {
                continue;
            }

            // Query-only geometry has no business adding weight, matching the engine's
            // INCLUDE_ALL_SHAPES flag being off by default.
            if (const Physics::ColliderConfiguration* colliderConfig = joltShape->GetColliderConfiguration();
                colliderConfig != nullptr && !colliderConfig->m_isSimulated)
            {
                continue;
            }

            // A triangle mesh encloses no volume and reports no mass; it cannot contribute
            // to a dynamic body's mass and Jolt will not simulate one dynamically anyway.
            const float volume = nativeShape->GetMassProperties().mMass / JoltDefaultShapeDensity;
            if (volume <= 0.0f)
            {
                continue;
            }

            float density = JoltMaterial::DefaultDensity;
            if (const auto* material = azdynamic_cast<const JoltMaterial*>(entry.Get().get()))
            {
                density = material->GetDensity();
            }
            totalMass += volume * density;
        }
        return totalMass;
    }

    void JoltRigidBody::ApplyBaseShapeToBody()
    {
        auto* bodyInterface = m_scene ? m_scene->GetBodyInterface() : nullptr;
        if (!bodyInterface || m_bodyId.IsInvalid() || !m_baseShape)
        {
            return;
        }

        JPH::RefConst<JPH::Shape> shape = m_baseShape;
        if (!m_configuration.m_centerOfMassOffset.IsZero())
        {
            shape = new JPH::RotatedTranslatedShape(
                Conversions::ToJolt(-m_configuration.m_centerOfMassOffset), JPH::Quat::sIdentity(), m_baseShape);
        }

        // Recompute the inertia for the new geometry, then set the mass the body should
        // have: recomputed from the new geometry when the body computes its own mass,
        // otherwise the configured one (which the recompute would have overwritten).
        bodyInterface->SetShape(m_bodyId, shape, /*updateMassProperties*/ true, JPH::EActivation::Activate);
        const float mass = m_configuration.m_computeMass ? ComputeMassFromGeometry() : m_configuration.m_mass;
        if (mass > 0.0f)
        {
            SetMass(mass);
        }
    }

    void JoltRigidBody::AddShape(AZStd::shared_ptr<Physics::Shape> shape)
    {
        if (!shape || !m_scene || m_bodyId.IsInvalid())
        {
            return;
        }

        auto* nativeShape = static_cast<JPH::Shape*>(shape->GetNativePointer());
        if (!nativeShape)
        {
            return;
        }

        JPH::Ref<JPH::MutableCompoundShape> compound = CloneBaseShapeAsMutableCompound();
        const auto [localPosition, localRotation] = shape->GetLocalPose();
        compound->AddShape(Conversions::ToJolt(localPosition), Conversions::ToJolt(localRotation), nativeShape);
        compound->AdjustCenterOfMass();
        m_baseShape = compound;

        // The new sub-shape lands at the end, so the material entry does too.
        m_attachedShapes.push_back(shape);
        m_colliderMaterials.push_back({ shape, nullptr });

        ApplyBaseShapeToBody();
    }

    void JoltRigidBody::RemoveShape(AZStd::shared_ptr<Physics::Shape> shape)
    {
        if (!shape || !m_scene || m_bodyId.IsInvalid())
        {
            return;
        }

        auto attachedIt = AZStd::find(m_attachedShapes.begin(), m_attachedShapes.end(), shape);
        if (attachedIt == m_attachedShapes.end())
        {
            AZ_Warning("JoltPhysics", false,
                "JoltRigidBody::RemoveShape%s: the shape is not attached to this body. Only shapes added "
                "with AddShape can be removed; colliders the body was created with are part of its geometry.",
                Internal::NameClause(m_configuration.m_debugName).c_str());
            return;
        }

        // Attached shapes occupy the sub-shapes after the colliders the body was created
        // with, in attachment order.
        const size_t attachedIndex = static_cast<size_t>(AZStd::distance(m_attachedShapes.begin(), attachedIt));
        const size_t createdColliderCount = m_colliderMaterials.size() - m_attachedShapes.size();
        const size_t subShapeIndex = createdColliderCount + attachedIndex;

        JPH::Ref<JPH::MutableCompoundShape> compound = CloneBaseShapeAsMutableCompound();
        if (subShapeIndex >= compound->GetNumSubShapes())
        {
            return;
        }

        compound->RemoveShape(static_cast<JPH::uint>(subShapeIndex));
        compound->AdjustCenterOfMass();
        m_baseShape = compound;

        m_attachedShapes.erase(attachedIt);
        m_colliderMaterials.erase(m_colliderMaterials.begin() + subShapeIndex);

        ApplyBaseShapeToBody();
    }

    void JoltRigidBody::UpdateMassProperties(
        AzPhysics::MassComputeFlags flags,
        const AZ::Vector3& centerOfMassOffsetOverride,
        const AZ::Matrix3x3& inertiaTensorOverride,
        const float massOverride)
    {
        // Each flag means "compute this from the geometry", and the engine documents the
        // matching override parameter as *ignored* when its flag is set. This read used to
        // be inverted - applying an override precisely when asked to compute - so the
        // default call, UpdateMassProperties() with DEFAULT flags, set mass to 1 kg, the
        // centre of mass to zero and the inertia to identity instead of recomputing any
        // of them.
        const auto isSet = [flags](AzPhysics::MassComputeFlags flag)
        {
            return (flags & flag) == flag;
        };
        const bool computeCenterOfMass = isSet(AzPhysics::MassComputeFlags::COMPUTE_COM);
        const bool computeInertia = isSet(AzPhysics::MassComputeFlags::COMPUTE_INERTIA);
        const bool computeMass = isSet(AzPhysics::MassComputeFlags::COMPUTE_MASS);

        // Computing the centre of mass means letting the geometry decide it, which in this
        // backend is the absence of an offset (see the centre-of-mass entry in DIVERGENCES).
        SetCenterOfMassOffset(computeCenterOfMass ? AZ::Vector3::CreateZero() : centerOfMassOffsetOverride);

        const float geometryMass = ComputeMassFromGeometry();
        const float mass = computeMass ? (geometryMass > 0.0f ? geometryMass : m_configuration.m_mass)
                                       : massOverride;
        m_configuration.m_computeMass = computeMass;
        m_configuration.m_mass = mass;

        if (!m_scene || m_bodyId.IsInvalid())
        {
            return;
        }
        auto* physicsSystem = m_scene->GetJoltPhysicsSystem();
        if (!physicsSystem)
        {
            return;
        }

        JPH::BodyLockWrite bodyLock(physicsSystem->GetBodyLockInterface(), m_bodyId);
        if (!bodyLock.Succeeded())
        {
            return;
        }
        JPH::Body& body = bodyLock.GetBody();
        JPH::MotionProperties* motionProperties = body.GetMotionProperties();
        if (!motionProperties)
        {
            return;
        }

        if (computeInertia)
        {
            // Take the shape's own inertia tensor and scale it to the mass in force, which
            // is what Jolt does when it builds a body with CalculateInertia.
            JPH::MassProperties massProperties = body.GetShape()->GetMassProperties();
            massProperties.ScaleToMass(AZStd::max(mass, 0.001f));
            motionProperties->SetMassProperties(motionProperties->GetAllowedDOFs(), massProperties);
        }
        else
        {
            motionProperties->SetInverseMass(1.0f / AZStd::max(mass, 0.001f));
            // Jolt stores a diagonal local inertia; off-diagonal elements are not supported.
            motionProperties->SetInverseInertia(JPH::Vec3(
                inertiaTensorOverride(0, 0) > 0.0f ? 1.0f / inertiaTensorOverride(0, 0) : 0.0f,
                inertiaTensorOverride(1, 1) > 0.0f ? 1.0f / inertiaTensorOverride(1, 1) : 0.0f,
                inertiaTensorOverride(2, 2) > 0.0f ? 1.0f / inertiaTensorOverride(2, 2) : 0.0f),
                motionProperties->GetInertiaRotation());
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
