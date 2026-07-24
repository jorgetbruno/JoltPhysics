#include <Character/JoltCharacter.h>

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
            m_shape = JoltShapeUtils::CreateJoltShapeFromConfig(*m_configuration.m_shapeConfig);
        }
        if (!m_shape)
        {
            Physics::CapsuleShapeConfiguration defaultShape(DefaultCharacterHeight, DefaultCharacterRadius);
            m_shape = JoltShapeUtils::CreateJoltShapeFromConfig(defaultShape);
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
            settings.mLayer = ObjectLayers::Moving;
            // The character is driven entirely by the requested velocity (same contract as
            // the virtual backend), so Jolt's own gravity is disabled. The supporting
            // volume accepts contacts below the centre; steep contacts are still rejected
            // as ground by the slope check.
            settings.mGravityFactor = 0.0f;
            settings.mSupportingVolume = JPH::Plane(settings.mUp, -1.0e10f);

            m_rigidBody = new JPH::Character(
                &settings,
                Conversions::ToJoltR(m_configuration.m_position),
                Conversions::ToJolt(m_configuration.m_orientation),
                /*userData*/ 0,
                m_scene->GetJoltPhysicsSystem());
            m_rigidBody->AddToPhysicsSystem();
            m_orientation = m_configuration.m_orientation;
            return;
        }

        JPH::CharacterVirtualSettings settings;
        settings.mUp = Conversions::ToJolt(m_configuration.m_upDirection);
        settings.mMaxSlopeAngle = AZ::DegToRad(m_slopeLimitDegrees);
        settings.mShape = m_shape;
        // The inner body makes the character visible to the simulation: dynamic bodies
        // are blocked/pushed by it and sensors fire trigger events for it.
        settings.mInnerBodyShape = m_shape;
        settings.mInnerBodyLayer = ObjectLayers::Moving;

        m_character = AZStd::make_unique<JPH::CharacterVirtual>(
            &settings,
            Conversions::ToJoltR(m_configuration.m_position),
            Conversions::ToJolt(m_configuration.m_orientation),
            m_scene->GetJoltPhysicsSystem());

        m_orientation = m_configuration.m_orientation;
    }

    void JoltCharacter::RemoveFromScene()
    {
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

    float JoltCharacter::GetBottomOffset() const
    {
        // Local z of the shape's bottom (negative for shapes centered at the origin).
        return m_shape ? m_shape->GetLocalBounds().mMin.GetZ() : 0.0f;
    }

    AZ::Vector3 JoltCharacter::GetBasePosition() const
    {
        return GetCenterPosition() + m_configuration.m_upDirection * GetBottomOffset();
    }

    void JoltCharacter::SetBasePosition(const AZ::Vector3& position)
    {
        const AZ::Vector3 center = position - m_configuration.m_upDirection * GetBottomOffset();
        if (m_rigidBody)
        {
            m_rigidBody->SetPosition(Conversions::ToJoltR(center));
            return;
        }
        if (m_character)
        {
            m_character->SetPosition(Conversions::ToJoltR(center));
        }
    }

    void JoltCharacter::SetRotation(const AZ::Quaternion& rotation)
    {
        m_orientation = rotation;
        if (m_rigidBody)
        {
            m_rigidBody->SetPositionAndRotation(m_rigidBody->GetPosition(), Conversions::ToJolt(rotation));
            return;
        }
        if (m_character)
        {
            m_character->SetRotation(Conversions::ToJolt(rotation));
        }
    }

    AZ::Vector3 JoltCharacter::GetCenterPosition() const
    {
        if (m_rigidBody)
        {
            return Conversions::FromJolt(m_rigidBody->GetPosition());
        }
        if (!m_character)
        {
            return m_configuration.m_position;
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

        m_character->ExtendedUpdate(
            deltaTime,
            Conversions::ToJolt(m_scene->GetGravity()),
            updateSettings,
            JPH::BroadPhaseLayerFilter(),
            JPH::ObjectLayerFilter(),
            JPH::BodyFilter(),
            JPH::ShapeFilter(),
            *GetJoltSystem()->GetJoltAllocator());

        m_observedVelocity = Conversions::FromJolt(m_character->GetLinearVelocity());
    }

    void JoltCharacter::AttachShape([[maybe_unused]] AZStd::shared_ptr<Physics::Shape> shape)
    {
        AZ_WarningOnce("JoltPhysics", false,
            "JoltCharacter::AttachShape is not supported (no Physics::Shape wrapper in the Jolt backend yet)");
    }

    AZ::EntityId JoltCharacter::GetEntityId() const
    {
        return m_configuration.m_entityId;
    }

    AZ::Transform JoltCharacter::GetTransform() const
    {
        return AZ::Transform::CreateFromQuaternionAndTranslation(m_orientation, GetCenterPosition());
    }

    void JoltCharacter::SetTransform(const AZ::Transform& transform)
    {
        SetRotation(transform.GetRotation());
        if (m_rigidBody)
        {
            m_rigidBody->SetPosition(Conversions::ToJoltR(transform.GetTranslation()));
            return;
        }
        if (m_character)
        {
            m_character->SetPosition(Conversions::ToJoltR(transform.GetTranslation()));
        }
    }

    AZ::Vector3 JoltCharacter::GetPosition() const
    {
        return GetCenterPosition();
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
