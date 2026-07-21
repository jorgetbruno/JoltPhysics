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
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
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

    JPH::BodyID JoltCharacter::GetInnerBodyId() const
    {
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
        if (!m_character)
        {
            return;
        }
        const AZ::Vector3 center = position - m_configuration.m_upDirection * GetBottomOffset();
        m_character->SetPosition(Conversions::ToJoltR(center));
    }

    void JoltCharacter::SetRotation(const AZ::Quaternion& rotation)
    {
        m_orientation = rotation;
        if (m_character)
        {
            m_character->SetRotation(Conversions::ToJolt(rotation));
        }
    }

    AZ::Vector3 JoltCharacter::GetCenterPosition() const
    {
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
        Move(velocity * deltaTime, deltaTime);
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
        if (!m_character)
        {
            return AZ::Aabb::CreateNull();
        }
        const JPH::AABox bounds = m_character->GetTransformedShape().GetWorldSpaceBounds();
        return AZ::Aabb::CreateFromMinMax(
            AZ::Vector3(bounds.mMin.GetX(), bounds.mMin.GetY(), bounds.mMin.GetZ()),
            AZ::Vector3(bounds.mMax.GetX(), bounds.mMax.GetY(), bounds.mMax.GetZ()));
    }

    AzPhysics::SceneQueryHit JoltCharacter::RayCast([[maybe_unused]] const AzPhysics::RayCastRequest& request)
    {
        // Body-level raycast against the character itself is not supported; use scene queries.
        return AzPhysics::SceneQueryHit();
    }

    AZ::Crc32 JoltCharacter::GetNativeType() const
    {
        return AZ_CRC_CE("JoltCharacter");
    }

    void* JoltCharacter::GetNativePointer() const
    {
        return m_character.get();
    }

    bool JoltCharacter::IsOnGround() const
    {
        return m_character && m_character->GetGroundState() == JPH::CharacterBase::EGroundState::OnGround;
    }

    AZ::Vector3 JoltCharacter::GetGroundNormal() const
    {
        if (!m_character)
        {
            return m_configuration.m_upDirection;
        }
        return Conversions::FromJolt(m_character->GetGroundNormal());
    }

} // namespace JoltPhysics
