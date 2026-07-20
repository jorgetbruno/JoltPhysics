#include <Shape/JoltShapeUtils.h>
#include <Utils/Conversions.h>

#include <AzFramework/Physics/Collision/CollisionGroups.h>
#include <AzFramework/Physics/Collision/CollisionLayers.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>

namespace JoltPhysics
{
    namespace
    {
        JPH::RefConst<JPH::Shape> CreateJoltShapeFromVariant(const AzPhysics::ShapeVariantData& colliderAndShapeData)
        {
            const AzPhysics::ShapeColliderPair* firstCollider = AZStd::get_if<AzPhysics::ShapeColliderPair>(&colliderAndShapeData);
            if (!firstCollider)
            {
                if (const auto* colliderList = AZStd::get_if<AzPhysics::ShapeColliderPairList>(&colliderAndShapeData);
                    colliderList && !colliderList->empty())
                {
                    firstCollider = &colliderList->front();
                }
            }

            if (firstCollider && firstCollider->second)
            {
                return JoltShapeUtils::CreateJoltShapeFromConfig(*firstCollider->second);
            }
            return nullptr;
        }
    }

    AZStd::shared_ptr<Physics::Shape> JoltShapeUtils::CreateShape(
        [[maybe_unused]] const Physics::ColliderConfiguration& colliderConfiguration,
        [[maybe_unused]] const Physics::ShapeConfiguration& shapeConfiguration)
    {
        // TODO: Create O3DE Shape wrapper around Jolt shape
        return nullptr;
    }

    JPH::RefConst<JPH::Shape> JoltShapeUtils::CreateJoltShape(
        const AzPhysics::RigidBodyConfiguration& configuration)
    {
        return CreateJoltShapeFromVariant(configuration.m_colliderAndShapeData);
    }

    JPH::RefConst<JPH::Shape> JoltShapeUtils::CreateJoltShapeFromStatic(
        const AzPhysics::StaticRigidBodyConfiguration& configuration)
    {
        return CreateJoltShapeFromVariant(configuration.m_colliderAndShapeData);
    }

    JPH::RefConst<JPH::Shape> JoltShapeUtils::CreateJoltShapeFromConfig(
        const Physics::ShapeConfiguration& shapeConfiguration)
    {
        switch (shapeConfiguration.GetShapeType())
        {
        case Physics::ShapeType::Box:
            return CreateBoxShape(static_cast<const Physics::BoxShapeConfiguration&>(shapeConfiguration));

        case Physics::ShapeType::Sphere:
            return CreateSphereShape(static_cast<const Physics::SphereShapeConfiguration&>(shapeConfiguration));

        case Physics::ShapeType::Capsule:
            return CreateCapsuleShape(static_cast<const Physics::CapsuleShapeConfiguration&>(shapeConfiguration));

        default:
            return nullptr;
        }
    }

    JPH::RefConst<JPH::Shape> JoltShapeUtils::CreateBoxShape(const Physics::BoxShapeConfiguration& config)
    {
        const AZ::Vector3 halfExtents = config.m_dimensions * 0.5f;
        return new JPH::BoxShape(Conversions::ToJolt(halfExtents));
    }

    JPH::RefConst<JPH::Shape> JoltShapeUtils::CreateSphereShape(const Physics::SphereShapeConfiguration& config)
    {
        return new JPH::SphereShape(config.m_radius);
    }

    JPH::RefConst<JPH::Shape> JoltShapeUtils::CreateCapsuleShape(const Physics::CapsuleShapeConfiguration& config)
    {
        const float halfHeight = config.m_height * 0.5f - config.m_radius;
        return new JPH::CapsuleShape(halfHeight, config.m_radius);
    }

} // namespace JoltPhysics
