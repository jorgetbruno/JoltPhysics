#include <Shape/JoltHeightfieldUtils.h>
#include <Shape/JoltMeshUtils.h>
#include <Shape/JoltShapeUtils.h>
#include <Shape/JoltShape.h>
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
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>

namespace JoltPhysics
{
    namespace
    {
        // Creates the shape for a single collider/shape pair, wrapping it in a
        // RotatedTranslatedShape when the collider configuration has a non-identity
        // offset or rotation. Note: the trigger flag (m_isTrigger) is applied per body
        // (Jolt sensors are body-level), not per shape - a compound mixing trigger and
        // solid shapes is not supported.
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

            // A pre-built Physics::Shape (e.g. from Physics::SystemRequests::CreateShape,
            // as used by the WhiteBox gem). Unwrap its native pointer directly.
            if (const auto* prebuiltShape = AZStd::get_if<AZStd::shared_ptr<Physics::Shape>>(&colliderAndShapeData))
            {
                if (*prebuiltShape)
                {
                    return static_cast<JPH::Shape*>((*prebuiltShape)->GetNativePointer());
                }
                return nullptr;
            }

            if (const auto* prebuiltShapeList = AZStd::get_if<AZStd::vector<AZStd::shared_ptr<Physics::Shape>>>(&colliderAndShapeData))
            {
                if (prebuiltShapeList->empty())
                {
                    return nullptr;
                }

                if (prebuiltShapeList->size() == 1)
                {
                    const AZStd::shared_ptr<Physics::Shape>& single = prebuiltShapeList->front();
                    return single ? static_cast<JPH::Shape*>(single->GetNativePointer()) : nullptr;
                }

                JPH::StaticCompoundShapeSettings compoundSettings;
                for (const AZStd::shared_ptr<Physics::Shape>& shapePtr : *prebuiltShapeList)
                {
                    if (!shapePtr)
                    {
                        continue;
                    }
                    if (auto* nativeShape = static_cast<JPH::Shape*>(shapePtr->GetNativePointer()))
                    {
                        const auto [localPosition, localRotation] = shapePtr->GetLocalPose();
                        compoundSettings.AddShape(
                            Conversions::ToJolt(localPosition), Conversions::ToJolt(localRotation), nativeShape);
                    }
                }

                if (compoundSettings.mSubShapes.empty())
                {
                    return nullptr;
                }

                JPH::ShapeSettings::ShapeResult result = compoundSettings.Create();
                if (result.HasError())
                {
                    AZ_Error("JoltPhysics", false, "Failed to create compound shape from prebuilt shapes: %s", result.GetError().c_str());
                    return nullptr;
                }
                return result.Get();
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
        const Physics::ColliderConfiguration& colliderConfiguration,
        const Physics::ShapeConfiguration& shapeConfiguration)
    {
        JPH::RefConst<JPH::Shape> nativeShape = CreateJoltShapeFromConfig(shapeConfiguration);
        if (!nativeShape)
        {
            return nullptr;
        }
        return AZStd::make_shared<JoltShape>(colliderConfiguration, shapeConfiguration, nativeShape);
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

        case Physics::ShapeType::CookedMesh:
        {
            // "Cooked" here just means our own packed vertex/index blob (see JoltMeshUtils);
            // Jolt needs no offline cooking pass. Cache the built native shape on the
            // configuration so repeated calls (and ReleaseNativeMeshObject) don't rebuild it.
            auto& meshConfiguration = const_cast<Physics::CookedMeshShapeConfiguration&>(
                static_cast<const Physics::CookedMeshShapeConfiguration&>(shapeConfiguration));

            if (auto* cachedMesh = static_cast<JPH::Shape*>(meshConfiguration.GetCachedNativeMesh()))
            {
                return cachedMesh;
            }

            // The same cooked config carries either a triangle mesh or a convex hull;
            // decode the blob according to the type the cooker recorded on it.
            JPH::RefConst<JPH::Shape> meshShape =
                (meshConfiguration.GetMeshType() == Physics::CookedMeshShapeConfiguration::MeshType::Convex)
                ? JoltMeshUtils::CreateConvexShapeFromCookedData(meshConfiguration.GetCookedMeshData())
                : JoltMeshUtils::CreateMeshShapeFromCookedData(meshConfiguration.GetCookedMeshData());
            if (!meshShape)
            {
                return nullptr;
            }

            // The configuration stores a raw void*; take an extra ref so the shape
            // stays alive independent of this RefConst going out of scope, matched by
            // a Release() in JoltPhysicsSystemComponent::ReleaseNativeMeshObject.
            meshShape->AddRef();
            meshConfiguration.SetCachedNativeMesh(const_cast<JPH::Shape*>(meshShape.GetPtr()));
            return meshShape;
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
        // A capsule is two hemispheres joined by a cylinder, so its total height must be at
        // least 2*radius. Below that the cylinder half-height would be negative (an invalid
        // Jolt capsule); degrade to a sphere sized to the requested height instead.
        const float halfHeight = config.m_height * 0.5f - config.m_radius;
        if (halfHeight <= 0.0f)
        {
            const float sphereRadius = AZStd::max(config.m_height * 0.5f, 0.001f);
            AZ_WarningOnce("JoltPhysics", false,
                "Capsule height (%.3f) is less than twice its radius (%.3f); using a sphere of "
                "radius %.3f instead. Increase the height or reduce the radius for a capsule.",
                config.m_height, 2.0f * config.m_radius, sphereRadius);
            return new JPH::SphereShape(sphereRadius);
        }

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

    AZStd::vector<AZStd::shared_ptr<Physics::Shape>> JoltShapeUtils::GetPrebuiltShapes(
        const AzPhysics::ShapeVariantData& colliderAndShapeData)
    {
        if (const auto* prebuiltShape = AZStd::get_if<AZStd::shared_ptr<Physics::Shape>>(&colliderAndShapeData))
        {
            if (*prebuiltShape)
            {
                return { *prebuiltShape };
            }
            return {};
        }

        if (const auto* prebuiltShapeList = AZStd::get_if<AZStd::vector<AZStd::shared_ptr<Physics::Shape>>>(&colliderAndShapeData))
        {
            AZStd::vector<AZStd::shared_ptr<Physics::Shape>> shapes;
            shapes.reserve(prebuiltShapeList->size());
            for (const AZStd::shared_ptr<Physics::Shape>& shape : *prebuiltShapeList)
            {
                // Match the sub-shape order of CreateJoltShapeFromVariant, which skips
                // null entries when building the compound.
                if (shape)
                {
                    shapes.push_back(shape);
                }
            }
            return shapes;
        }

        return {};
    }

    JPH::Ref<JPH::MutableCompoundShape> JoltShapeUtils::MakeMutableCompound(const JPH::Shape* shape)
    {
        JPH::Ref<JPH::MutableCompoundShape> mutableCompound = new JPH::MutableCompoundShape();
        if (!shape)
        {
            return mutableCompound;
        }

        const JPH::EShapeSubType subType = shape->GetSubType();
        if (subType == JPH::EShapeSubType::StaticCompound || subType == JPH::EShapeSubType::MutableCompound)
        {
            const auto* compound = static_cast<const JPH::CompoundShape*>(shape);
            // Sub-shape positions are stored relative to the compound's center of mass,
            // while AddShape takes them in the compound's local space; convert back by
            // adding the source compound's center of mass.
            const JPH::Vec3 sourceCenterOfMass = compound->GetCenterOfMass();
            for (const JPH::CompoundShape::SubShape& subShape : compound->GetSubShapes())
            {
                mutableCompound->AddShape(
                    subShape.GetPositionCOM() + sourceCenterOfMass, subShape.GetRotation(), subShape.mShape,
                    subShape.mUserData);
            }
        }
        else
        {
            mutableCompound->AddShape(JPH::Vec3::sZero(), JPH::Quat::sIdentity(), shape);
        }

        mutableCompound->AdjustCenterOfMass();
        return mutableCompound;
    }

} // namespace JoltPhysics
