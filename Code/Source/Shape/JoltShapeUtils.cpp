#include <Shape/JoltHeightfieldUtils.h>
#include <Shape/JoltShapeUtils.h>
#include <Utils/Conversions.h>

#include <AzFramework/Physics/Collision/CollisionGroups.h>
#include <AzFramework/Physics/Collision/CollisionLayers.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>

namespace JoltPhysics
{
    namespace
    {
        // Creates the shape for a single collider/shape pair, wrapping it in a
        // RotatedTranslatedShape when the collider configuration has a non-identity
        // offset or rotation. Note: trigger shapes (m_isTrigger) are not supported yet.
        JPH::RefConst<JPH::Shape> CreateOffsetJoltShape(const AzPhysics::ShapeColliderPair& colliderAndShape)
        {
            if (!colliderAndShape.second)
            {
                return nullptr;
            }

            JPH::RefConst<JPH::Shape> shape = JoltShapeUtils::CreateJoltShapeFromConfig(*colliderAndShape.second);
            if (!shape || !colliderAndShape.first)
            {
                return shape;
            }

            const Physics::ColliderConfiguration& colliderConfiguration = *colliderAndShape.first;
            if (colliderConfiguration.m_position.IsZero() && colliderConfiguration.m_rotation.IsIdentity())
            {
                return shape;
            }

            return new JPH::RotatedTranslatedShape(
                Conversions::ToJolt(colliderConfiguration.m_position),
                Conversions::ToJolt(colliderConfiguration.m_rotation),
                shape);
        }

        JPH::RefConst<JPH::Shape> CreateJoltShapeFromVariant(const AzPhysics::ShapeVariantData& colliderAndShapeData)
        {
            if (const AzPhysics::ShapeColliderPair* singleCollider = AZStd::get_if<AzPhysics::ShapeColliderPair>(&colliderAndShapeData))
            {
                return CreateOffsetJoltShape(*singleCollider);
            }

            if (const auto* colliderList = AZStd::get_if<AzPhysics::ShapeColliderPairList>(&colliderAndShapeData))
            {
                if (colliderList->empty())
                {
                    return nullptr;
                }

                if (colliderList->size() == 1)
                {
                    return CreateOffsetJoltShape(colliderList->front());
                }

                JPH::StaticCompoundShapeSettings compoundSettings;
                for (const AzPhysics::ShapeColliderPair& colliderAndShape : *colliderList)
                {
                    if (!colliderAndShape.second)
                    {
                        continue;
                    }

                    JPH::RefConst<JPH::Shape> subShape = JoltShapeUtils::CreateJoltShapeFromConfig(*colliderAndShape.second);
                    if (!subShape)
                    {
                        continue;
                    }

                    JPH::Vec3 position = JPH::Vec3::sZero();
                    JPH::Quat rotation = JPH::Quat::sIdentity();
                    if (colliderAndShape.first)
                    {
                        position = Conversions::ToJolt(colliderAndShape.first->m_position);
                        rotation = Conversions::ToJolt(colliderAndShape.first->m_rotation);
                    }
                    compoundSettings.AddShape(position, rotation, subShape);
                }

                if (compoundSettings.mSubShapes.empty())
                {
                    return nullptr;
                }

                JPH::ShapeSettings::ShapeResult result = compoundSettings.Create();
                if (result.HasError())
                {
                    AZ_Error("JoltPhysics", false, "Failed to create compound shape: %s", result.GetError().c_str());
                    return nullptr;
                }
                return result.Get();
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

        case Physics::ShapeType::Heightfield:
        {
            // The native heightfield is produced by the heightfield collider component
            // (from a HeightfieldProviderBus implementation) and cached on the configuration.
            const auto& heightfieldConfiguration =
                static_cast<const Physics::HeightfieldShapeConfiguration&>(shapeConfiguration);
            if (auto* nativeHeightfield =
                    static_cast<JPH::Shape*>(const_cast<void*>(heightfieldConfiguration.GetCachedNativeHeightfield())))
            {
                return JoltHeightfieldUtils::WrapZUp(nativeHeightfield);
            }
            return nullptr;
        }

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

        // Jolt capsules are Y-axis aligned, O3DE (like PhysX) capsules are Z-axis
        // aligned: wrap the capsule rotated +90 degrees around X.
        const JPH::Quat yToZRotation(0.70710678f, 0.0f, 0.0f, 0.70710678f);
        return new JPH::RotatedTranslatedShape(
            JPH::Vec3::sZero(),
            yToZRotation,
            new JPH::CapsuleShape(halfHeight, config.m_radius));
    }

    const Physics::ColliderConfiguration* JoltShapeUtils::GetFirstColliderConfiguration(
        const AzPhysics::ShapeVariantData& colliderAndShapeData)
    {
        if (const auto* singleCollider = AZStd::get_if<AzPhysics::ShapeColliderPair>(&colliderAndShapeData))
        {
            return singleCollider->first.get();
        }

        if (const auto* colliderList = AZStd::get_if<AzPhysics::ShapeColliderPairList>(&colliderAndShapeData);
            colliderList && !colliderList->empty())
        {
            return colliderList->front().first.get();
        }

        return nullptr;
    }

    AzPhysics::ShapeColliderPairList JoltShapeUtils::GetColliderPairList(
        const AzPhysics::ShapeVariantData& colliderAndShapeData)
    {
        if (const auto* singleCollider = AZStd::get_if<AzPhysics::ShapeColliderPair>(&colliderAndShapeData))
        {
            if (singleCollider->first || singleCollider->second)
            {
                return { *singleCollider };
            }
            return {};
        }

        if (const auto* colliderList = AZStd::get_if<AzPhysics::ShapeColliderPairList>(&colliderAndShapeData))
        {
            return *colliderList;
        }

        return {};
    }

} // namespace JoltPhysics
