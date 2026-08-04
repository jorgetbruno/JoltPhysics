#include <Character/JoltCharacter.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>

#include <AzFramework/Physics/Collision/CollisionGroups.h>
#include <AzFramework/Physics/Collision/CollisionLayers.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

#include <Scene/JoltScene.h>
#include <Shape/JoltShapeUtils.h>
#include <System/CollisionLayerFilters.h>
#include <System/JoltSystem.h>
#include <Utils/Conversions.h>

#include <Jolt/Jolt.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <Utils/JoltDiagnostics.h>

namespace JoltPhysics
{
    namespace
    {
        // Capsule fallback when the configuration carries no shape (1.8 m tall, 0.3 m radius).
        constexpr float DefaultCharacterHeight = 1.8f;
        constexpr float DefaultCharacterRadius = 0.3f;

    }

    JoltCharacter::JoltCharacter(const Physics::CharacterConfiguration& configuration)
        : m_configuration(configuration)
        , m_collisionLayer(configuration.m_collisionLayer)
        , m_collisionGroup(AzPhysics::CollisionGroup::All)
    {
        m_stepHeight = configuration.m_stepHeight;
        m_slopeLimitDegrees = configuration.m_maximumSlopeAngle;
        m_maximumSpeed = configuration.m_maximumSpeed;
        if (const auto* joltConfig = azrtti_cast<const JoltCharacterConfiguration*>(&configuration))
        {
            m_rigidBodyCharacter = joltConfig->m_rigidBodyCharacter;
        }
    }

    JoltCharacter::~JoltCharacter()
    {
        RemoveFromScene();
    }

    void JoltCharacter::CreateInScene(JoltScene* scene)
    {
        m_scene = scene;
        if (!m_scene || !m_scene->GetJoltPhysicsSystem())
        {
            return;
        }

        if (m_configuration.m_shapeConfig)
        {
            m_shape = JoltShapeUtils::CreateJoltShapeFromConfig(*m_configuration.m_shapeConfig, m_configuration.m_debugName);
        }
        if (!m_shape)
        {
            Physics::CapsuleShapeConfiguration defaultShape(DefaultCharacterHeight, DefaultCharacterRadius);
            m_shape = JoltShapeUtils::CreateJoltShapeFromConfig(defaultShape, m_configuration.m_debugName);
        }
        if (!m_shape)
        {
            return;
        }

        if (m_rigidBodyCharacter)
        {
            JPH::CharacterSettings settings;
            settings.mUp = Conversions::ToJolt(m_configuration.m_upDirection);
            settings.mMaxSlopeAngle = AZ::DegToRad(m_slopeLimitDegrees);
            settings.mShape = m_shape;
            m_objectLayer =
                AcquireObjectLayer(
                m_configuration.m_collisionLayer, m_configuration.m_collisionGroupId, /*isMoving*/ true,
                JoltBodyClass::Character);
            settings.mLayer = m_objectLayer;
            // The character is driven entirely by the requested velocity (same contract as
            // the virtual backend), so Jolt's own gravity is disabled. The supporting
            // volume accepts contacts below the centre; steep contacts are still rejected
            // as ground by the slope check.
            settings.mGravityFactor = 0.0f;
            settings.mSupportingVolume = JPH::Plane(settings.mUp, -1.0e10f);
            // Characters walk over meshes and heightfields constantly, so ghost contacts
            // on triangle seams are felt here more than anywhere else.
            settings.mEnhancedInternalEdgeRemoval = UseEnhancedInternalEdgeRemoval();

            m_rigidBody = new JPH::Character(
                &settings,
                Conversions::ToJoltR(BaseToCenter(m_configuration.m_position)),
                Conversions::ToJolt(m_configuration.m_orientation),
                /*userData*/ 0,
                m_scene->GetJoltPhysicsSystem());
            m_rigidBody->AddToPhysicsSystem();
            m_orientation = m_configuration.m_orientation;
            AttachConfiguredColliders();
            return;
        }

        JPH::CharacterVirtualSettings settings;
        settings.mUp = Conversions::ToJolt(m_configuration.m_upDirection);
        settings.mMaxSlopeAngle = AZ::DegToRad(m_slopeLimitDegrees);
        settings.mShape = m_shape;
        // The inner body makes the character visible to the simulation: dynamic bodies
        // are blocked/pushed by it and sensors fire trigger events for it.
        settings.mInnerBodyShape = m_shape;
        m_objectLayer =
            AcquireObjectLayer(
                m_configuration.m_collisionLayer, m_configuration.m_collisionGroupId, /*isMoving*/ true,
                JoltBodyClass::Character);
        settings.mInnerBodyLayer = m_objectLayer;
        settings.mEnhancedInternalEdgeRemoval = UseEnhancedInternalEdgeRemoval();

        m_character = AZStd::make_unique<JPH::CharacterVirtual>(
            &settings,
            Conversions::ToJoltR(BaseToCenter(m_configuration.m_position)),
            Conversions::ToJolt(m_configuration.m_orientation),
            m_scene->GetJoltPhysicsSystem());

        m_orientation = m_configuration.m_orientation;

        AttachConfiguredColliders();
    }

    void JoltCharacter::AttachConfiguredColliders()
    {
        // The colliders the character was configured with - in a component setup these are
        // the collider components sitting on the same entity, which the character
        // controller component gathers for exactly this. Appended rather than assigned, so
        // a shape attached before the character reached a scene is not lost.
        if (m_configuration.m_colliders.empty())
        {
            RefreshAttachmentBody();
            return;
        }

        m_attachedShapes.insert(
            m_attachedShapes.end(), m_configuration.m_colliders.begin(), m_configuration.m_colliders.end());
        RefreshAttachmentBody();
    }

    void JoltCharacter::RemoveFromScene()
    {
        DestroyAttachmentBody();

        if (m_rigidBody)
        {
            m_rigidBody->RemoveFromPhysicsSystem();
            m_rigidBody = nullptr;
        }
    }

    void JoltCharacter::PostSimulation()
    {
        if (m_rigidBody)
        {
            // Snap to and detect ground within a small separation distance, then read back
            // the velocity the solver actually produced this step.
            m_rigidBody->PostSimulation(0.05f);
            m_observedVelocity = Conversions::FromJolt(m_rigidBody->GetLinearVelocity());

            // The rigid-body character moves during the step rather than in Move(), so
            // this is where its attachments catch up. Teleported: the body has already
            // arrived, and driving them at a pose it is standing on would fight it.
            SyncAttachmentBody(0.0f);
        }
    }

    JPH::BodyID JoltCharacter::GetInnerBodyId() const
    {
        if (m_rigidBody)
        {
            return m_rigidBody->GetBodyID();
        }
        return m_character ? m_character->GetInnerBodyID() : JPH::BodyID();
    }

    void JoltCharacter::SaveNativeState(JPH::StateRecorder& recorder) const
    {
        // The virtual character's position, rotation and velocity live on the
        // CharacterVirtual itself; the rigid character's live on its body (saved with
        // the system), so its CharacterBase::SaveState only adds the cached ground.
        // Either way the backend knows what it needs.
        if (m_character)
        {
            m_character->SaveState(recorder);
        }
        else if (m_rigidBody)
        {
            m_rigidBody->SaveState(recorder);
        }
    }

    void JoltCharacter::RestoreNativeState(JPH::StateRecorder& recorder)
    {
        if (m_character)
        {
            m_character->RestoreState(recorder);
        }
        else if (m_rigidBody)
        {
            m_rigidBody->RestoreState(recorder);
        }
    }

    float JoltCharacter::GetBottomOffset() const
    {
        // Local z of the shape's bottom (negative for shapes centered at the origin).
        return m_shape ? m_shape->GetLocalBounds().mMin.GetZ() : 0.0f;
    }

    AZ::Vector3 JoltCharacter::BaseToCenter(const AZ::Vector3& basePosition) const
    {
        // GetBottomOffset is negative for a shape centred on its origin, so this lifts
        // the centre above the feet.
        return basePosition - m_configuration.m_upDirection * GetBottomOffset();
    }

    AZ::Vector3 JoltCharacter::GetBasePosition() const
    {
        return GetCenterPosition() + m_configuration.m_upDirection * GetBottomOffset();
    }

    void JoltCharacter::SetBasePosition(const AZ::Vector3& position)
    {
        const AZ::Vector3 center = BaseToCenter(position);
        if (m_rigidBody)
        {
            m_rigidBody->SetPosition(Conversions::ToJoltR(center));
            SyncAttachmentBody(0.0f);
            return;
        }
        if (m_character)
        {
            m_character->SetPosition(Conversions::ToJoltR(center));
        }
        SyncAttachmentBody(0.0f);
    }

    void JoltCharacter::SetRotation(const AZ::Quaternion& rotation)
    {
        m_orientation = rotation;
        if (m_rigidBody)
        {
            m_rigidBody->SetPositionAndRotation(m_rigidBody->GetPosition(), Conversions::ToJolt(rotation));
            SyncAttachmentBody(0.0f);
            return;
        }
        if (m_character)
        {
            m_character->SetRotation(Conversions::ToJolt(rotation));
        }
        SyncAttachmentBody(0.0f);
    }

    AZ::Vector3 JoltCharacter::GetCenterPosition() const
    {
        if (m_rigidBody)
        {
            return Conversions::FromJolt(m_rigidBody->GetPosition());
        }
        if (!m_character)
        {
            // Nothing created yet, so answer from the configuration - which holds a base
            // position. With no shape the offset is zero and this degrades to the base.
            return BaseToCenter(m_configuration.m_position);
        }
        const JPH::RVec3 position = m_character->GetPosition();
        return AZ::Vector3(
            static_cast<float>(position.GetX()),
            static_cast<float>(position.GetY()),
            static_cast<float>(position.GetZ()));
    }

    float JoltCharacter::GetStepHeight() const
    {
        return m_stepHeight;
    }

    void JoltCharacter::SetStepHeight(float stepHeight)
    {
        m_stepHeight = stepHeight;
    }

    AZ::Vector3 JoltCharacter::GetUpDirection() const
    {
        return m_configuration.m_upDirection;
    }

    void JoltCharacter::SetUpDirection(const AZ::Vector3& upDirection)
    {
        m_configuration.m_upDirection = upDirection;
        if (m_character)
        {
            m_character->SetUp(Conversions::ToJolt(upDirection));
        }
        if (m_rigidBody)
        {
            m_rigidBody->SetUp(Conversions::ToJolt(upDirection));
        }
    }

    float JoltCharacter::GetSlopeLimitDegrees() const
    {
        return m_slopeLimitDegrees;
    }

    void JoltCharacter::SetSlopeLimitDegrees(float slopeLimitDegrees)
    {
        m_slopeLimitDegrees = slopeLimitDegrees;
        if (m_character)
        {
            m_character->SetMaxSlopeAngle(AZ::DegToRad(slopeLimitDegrees));
        }
        if (m_rigidBody)
        {
            m_rigidBody->SetMaxSlopeAngle(AZ::DegToRad(slopeLimitDegrees));
        }
    }

    float JoltCharacter::GetMaximumSpeed() const
    {
        return m_maximumSpeed;
    }

    void JoltCharacter::SetMaximumSpeed(float maximumSpeed)
    {
        m_maximumSpeed = AZStd::max(0.0f, maximumSpeed);
    }

    AZ::Vector3 JoltCharacter::GetVelocity() const
    {
        return m_observedVelocity;
    }

    void JoltCharacter::SetCollisionLayer(const AzPhysics::CollisionLayer& layer)
    {
        m_collisionLayer = layer;
    }

    void JoltCharacter::SetCollisionGroup(const AzPhysics::CollisionGroup& group)
    {
        m_collisionGroup = group;
    }

    AzPhysics::CollisionLayer JoltCharacter::GetCollisionLayer() const
    {
        return m_collisionLayer;
    }

    AzPhysics::CollisionGroup JoltCharacter::GetCollisionGroup() const
    {
        return m_collisionGroup;
    }

    AZ::Crc32 JoltCharacter::GetColliderTag() const
    {
        return AZ::Crc32(m_configuration.m_colliderTag.c_str());
    }

    void JoltCharacter::AddVelocityForTick(const AZ::Vector3& velocity)
    {
        m_requestedVelocityForTick += velocity;
    }

    void JoltCharacter::AddVelocityForPhysicsTimestep(const AZ::Vector3& velocity)
    {
        m_requestedVelocityForPhysicsTimestep += velocity;
    }

    void JoltCharacter::ApplyRequestedVelocity(float deltaTime)
    {
        AZ::Vector3 velocity = m_requestedVelocityForTick + m_requestedVelocityForPhysicsTimestep;
        const float speed = velocity.GetLength();
        if (speed > m_maximumSpeed && speed > 0.0f)
        {
            velocity *= m_maximumSpeed / speed;
        }
        if (m_rigidBodyCharacter)
        {
            // The physics step integrates the motion; PostSimulation (after the step)
            // refreshes ground state and the observed velocity.
            if (m_rigidBody)
            {
                m_rigidBody->SetLinearVelocity(Conversions::ToJolt(velocity));
            }
        }
        else
        {
            Move(velocity * deltaTime, deltaTime);
        }
        ResetRequestedVelocityForPhysicsTimestep();
    }

    void JoltCharacter::ResetRequestedVelocityForTick()
    {
        m_requestedVelocityForTick = AZ::Vector3::CreateZero();
    }

    void JoltCharacter::ResetRequestedVelocityForPhysicsTimestep()
    {
        m_requestedVelocityForPhysicsTimestep = AZ::Vector3::CreateZero();
    }

    void JoltCharacter::Move(const AZ::Vector3& requestedMovement, float deltaTime)
    {
        if (!m_character || !m_scene || deltaTime <= 0.0f)
        {
            return;
        }
        if (requestedMovement.GetLengthSq() < m_configuration.m_minimumMovementDistance * m_configuration.m_minimumMovementDistance)
        {
            m_observedVelocity = AZ::Vector3::CreateZero();
            return;
        }

        m_character->SetLinearVelocity(Conversions::ToJolt(requestedMovement / deltaTime));

        JPH::CharacterVirtual::ExtendedUpdateSettings updateSettings;
        const JPH::Vec3 up = Conversions::ToJolt(m_configuration.m_upDirection);
        // Stick to the ground when walking down slopes/steps, and climb steps of up to
        // the configured step height.
        updateSettings.mStickToFloorStepDown = -up * m_stepHeight;
        updateSettings.mWalkStairsStepUp = up * m_stepHeight;

        // Filtered by the character's own object layer. A CharacterVirtual sweeps its
        // movement itself instead of going through the simulation, so a default-
        // constructed filter here would accept every layer and the configured collision
        // layer / "collides with" group would only ever affect the inner body - the
        // character would still walk into everything.
        JPH::PhysicsSystem* physicsSystem = m_scene->GetJoltPhysicsSystem();
        m_character->ExtendedUpdate(
            deltaTime,
            Conversions::ToJolt(m_scene->GetGravity()),
            updateSettings,
            physicsSystem->GetDefaultBroadPhaseLayerFilter(m_objectLayer),
            // Soft bodies are excluded here, without a body filter: the character's object
            // layer is registered as JoltBodyClass::Character, and the object layer pair
            // filter refuses character-versus-cloth. That covers the paths a filter here
            // could not - Jolt's rigid Character runs its own ground query with a hardcoded
            // body filter, and the inner body's contacts are resolved by the simulation.
            physicsSystem->GetDefaultLayerFilter(m_objectLayer),
            JPH::BodyFilter(),
            JPH::ShapeFilter(),
            *GetJoltSystem()->GetJoltAllocator());

        m_observedVelocity = Conversions::FromJolt(m_character->GetLinearVelocity());

        SyncAttachmentBody(deltaTime);
    }

    void JoltCharacter::AttachShape(AZStd::shared_ptr<Physics::Shape> shape)
    {
        if (!shape)
        {
            return;
        }

        m_attachedShapes.push_back(AZStd::move(shape));
        RefreshAttachmentBody();
    }

    JPH::RefConst<JPH::Shape> JoltCharacter::BuildAttachmentShape() const
    {
        JPH::StaticCompoundShapeSettings compound;
        AZ::u32 addedShapes = 0;

        for (const AZStd::shared_ptr<Physics::Shape>& attached : m_attachedShapes)
        {
            const auto* nativeShape = attached ? static_cast<const JPH::Shape*>(attached->GetNativePointer()) : nullptr;
            if (nativeShape == nullptr)
            {
                continue;
            }

            const auto [offset, rotation] = attached->GetLocalPose();
            compound.AddShape(Conversions::ToJolt(offset), Conversions::ToJolt(rotation), nativeShape);
            ++addedShapes;
        }

        if (addedShapes == 0)
        {
            return nullptr;
        }

        // Jolt rejects a compound with a single child at the origin, and it would be
        // wasted indirection anyway.
        if (addedShapes == 1)
        {
            const AZStd::shared_ptr<Physics::Shape>& only = m_attachedShapes.front();
            const auto [offset, rotation] = only->GetLocalPose();
            if (offset.IsZero() && rotation.IsIdentity())
            {
                return static_cast<const JPH::Shape*>(only->GetNativePointer());
            }
        }

        const JPH::ShapeSettings::ShapeResult result = compound.Create();
        if (result.HasError())
        {
            AZ_Warning("JoltPhysics", false,
                "Could not build the attached colliders for character%s into one shape: %s",
                Internal::NameClause(m_configuration.m_debugName).c_str(), result.GetError().c_str());
            return nullptr;
        }
        return result.Get();
    }

    void JoltCharacter::RefreshAttachmentBody()
    {
        if (m_scene == nullptr)
        {
            // Attached before the character reached a scene; CreateInScene builds the body.
            return;
        }

        JPH::BodyInterface* bodyInterface = m_scene->GetBodyInterface();
        if (bodyInterface == nullptr)
        {
            return;
        }

        const JPH::RefConst<JPH::Shape> attachmentShape = BuildAttachmentShape();
        if (attachmentShape == nullptr)
        {
            DestroyAttachmentBody();
            return;
        }

        if (!m_attachmentBodyId.IsInvalid())
        {
            // Already there: re-shape it in place rather than churning a body id that
            // other things may be holding on to.
            bodyInterface->SetShape(m_attachmentBodyId, attachmentShape, /*updateMassProperties*/ false,
                JPH::EActivation::Activate);
            return;
        }

        const AZ::Transform transform = GetTransform();

        // Kinematic, and on the character's own object layer: the attachments are there to
        // be found by queries, to fire sensors and to be hit, not to hold the character up.
        // Kinematic also keeps it from colliding with the character's own inner body,
        // which is kinematic too.
        JPH::BodyCreationSettings settings(
            attachmentShape,
            Conversions::ToJoltR(transform.GetTranslation()),
            Conversions::ToJolt(transform.GetRotation()),
            JPH::EMotionType::Kinematic,
            m_objectLayer);
        settings.mIsSensor = false;

        JPH::Body* body = bodyInterface->CreateBody(settings);
        if (body == nullptr)
        {
            AZ_Warning("JoltPhysics", false,
                "Ran out of bodies attaching colliders to character%s.",
                Internal::NameClause(m_configuration.m_debugName).c_str());
            return;
        }

        m_attachmentBodyId = body->GetID();
        bodyInterface->AddBody(m_attachmentBodyId, JPH::EActivation::Activate);
    }

    void JoltCharacter::DestroyAttachmentBody()
    {
        if (m_attachmentBodyId.IsInvalid())
        {
            return;
        }

        if (m_scene != nullptr)
        {
            if (JPH::BodyInterface* bodyInterface = m_scene->GetBodyInterface())
            {
                bodyInterface->RemoveBody(m_attachmentBodyId);
                bodyInterface->DestroyBody(m_attachmentBodyId);
            }
        }
        m_attachmentBodyId = JPH::BodyID();
    }

    void JoltCharacter::SyncAttachmentBody(float deltaTime)
    {
        if (m_attachmentBodyId.IsInvalid() || m_scene == nullptr)
        {
            return;
        }

        JPH::BodyInterface* bodyInterface = m_scene->GetBodyInterface();
        if (bodyInterface == nullptr)
        {
            return;
        }

        const AZ::Transform transform = GetTransform();
        const JPH::RVec3 position = Conversions::ToJoltR(transform.GetTranslation());
        const JPH::Quat rotation = Conversions::ToJolt(transform.GetRotation());

        if (deltaTime > 0.0f)
        {
            // Driven rather than teleported, so the body carries a velocity and can push
            // what it touches - a shield or a swung weapon would otherwise pass through
            // dynamic bodies without moving them.
            bodyInterface->MoveKinematic(m_attachmentBodyId, position, rotation, deltaTime);
        }
        else
        {
            // An explicit position set is a teleport, and the attachments go with it.
            bodyInterface->SetPositionAndRotation(m_attachmentBodyId, position, rotation, JPH::EActivation::Activate);
        }
    }

    AZ::EntityId JoltCharacter::GetEntityId() const
    {
        return m_configuration.m_entityId;
    }

    AZ::Transform JoltCharacter::GetTransform() const
    {
        // The entity transform tracks the base, not the shape centre - see BaseToCenter.
        return AZ::Transform::CreateFromQuaternionAndTranslation(m_orientation, GetBasePosition());
    }

    void JoltCharacter::SetTransform(const AZ::Transform& transform)
    {
        SetRotation(transform.GetRotation());
        SetBasePosition(transform.GetTranslation());
    }

    AZ::Vector3 JoltCharacter::GetPosition() const
    {
        // Must agree with GetTransform, which reports the base - they are the same
        // concept on SimulatedBody, and the entity transform tracks both.
        return GetBasePosition();
    }

    AZ::Quaternion JoltCharacter::GetOrientation() const
    {
        return m_orientation;
    }

    AZ::Aabb JoltCharacter::GetAabb() const
    {
        if (m_rigidBody)
        {
            // Approximate: the shape's local bounds translated to the character centre.
            const AZ::Vector3 center = GetCenterPosition();
            const JPH::AABox local = m_shape->GetLocalBounds();
            return AZ::Aabb::CreateFromMinMax(
                center + Conversions::FromJolt(local.mMin), center + Conversions::FromJolt(local.mMax));
        }
        if (!m_character)
        {
            return AZ::Aabb::CreateNull();
        }
        const JPH::AABox bounds = m_character->GetTransformedShape().GetWorldSpaceBounds();
        return AZ::Aabb::CreateFromMinMax(
            AZ::Vector3(bounds.mMin.GetX(), bounds.mMin.GetY(), bounds.mMin.GetZ()),
            AZ::Vector3(bounds.mMax.GetX(), bounds.mMax.GetY(), bounds.mMax.GetZ()));
    }

    AzPhysics::SceneQueryHit JoltCharacter::RayCast(const AzPhysics::RayCastRequest& request)
    {
        AzPhysics::SceneQueryHit queryHit;
        if (!m_shape)
        {
            return queryHit;
        }

        // Cast against the character's shape placed at its current pose. The virtual
        // character exposes its transformed shape directly; the rigid-body backend's
        // shape sits at the character centre, so the equivalent is built here (Jolt
        // positions a transformed shape by its centre of mass).
        const JPH::TransformedShape transformedShape = (!m_rigidBodyCharacter && m_character)
            ? m_character->GetTransformedShape()
            : JPH::TransformedShape(
                  Conversions::ToJoltR(GetCenterPosition()) +
                      Conversions::ToJolt(m_orientation) * m_shape->GetCenterOfMass(),
                  Conversions::ToJolt(m_orientation), m_shape, JPH::BodyID());

        const JPH::RRayCast ray(
            Conversions::ToJoltR(request.m_start), Conversions::ToJolt(request.m_direction * request.m_distance));

        JPH::RayCastResult result;
        if (!transformedShape.CastRay(ray, result))
        {
            return queryHit;
        }

        queryHit.m_distance = result.mFraction * request.m_distance;
        queryHit.m_position = request.m_start + request.m_direction * queryHit.m_distance;
        queryHit.m_normal = Conversions::FromJolt(
            transformedShape.GetWorldSpaceSurfaceNormal(result.mSubShapeID2, Conversions::ToJoltR(queryHit.m_position)));
        queryHit.m_resultFlags = AzPhysics::SceneQuery::ResultFlags::Distance |
            AzPhysics::SceneQuery::ResultFlags::Position | AzPhysics::SceneQuery::ResultFlags::Normal |
            AzPhysics::SceneQuery::ResultFlags::BodyHandle | AzPhysics::SceneQuery::ResultFlags::EntityId;
        queryHit.m_bodyHandle = m_bodyHandle;
        queryHit.m_entityId = GetEntityId();
        return queryHit;
    }

    AZ::Crc32 JoltCharacter::GetNativeType() const
    {
        return AZ_CRC_CE("JoltCharacter");
    }

    void* JoltCharacter::GetNativePointer() const
    {
        return m_rigidBody ? static_cast<void*>(m_rigidBody.GetPtr()) : static_cast<void*>(m_character.get());
    }

    bool JoltCharacter::IsOnGround() const
    {
        if (m_rigidBody)
        {
            return m_rigidBody->GetGroundState() == JPH::CharacterBase::EGroundState::OnGround;
        }
        return m_character && m_character->GetGroundState() == JPH::CharacterBase::EGroundState::OnGround;
    }

    AZ::Vector3 JoltCharacter::GetGroundNormal() const
    {
        if (m_rigidBody)
        {
            return Conversions::FromJolt(m_rigidBody->GetGroundNormal());
        }
        if (!m_character)
        {
            return m_configuration.m_upDirection;
        }
        return Conversions::FromJolt(m_character->GetGroundNormal());
    }

} // namespace JoltPhysics
