#include <Scene/JoltSceneQueryHelpers.h>
#include <Scene/JoltScene.h>
#include <RigidBody/JoltRigidBody.h>
#include <RigidBody/JoltStaticRigidBody.h>
#include <Shape/JoltShapeUtils.h>
#include <System/CollisionLayerFilters.h>
#include <Utils/Conversions.h>

#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>

namespace JoltPhysics
{
    namespace
    {
        // Broadphase filter mapping the query's static/dynamic object type selection.
        class QueryBroadPhaseLayerFilter final : public JPH::BroadPhaseLayerFilter
        {
        public:
            explicit QueryBroadPhaseLayerFilter(AzPhysics::SceneQuery::QueryType queryType)
                : m_queryType(queryType)
            {
            }

            bool ShouldCollide(JPH::BroadPhaseLayer inLayer) const override
            {
                switch (m_queryType)
                {
                case AzPhysics::SceneQuery::QueryType::Static:
                    return inLayer == BroadPhaseLayers::NonMoving;
                case AzPhysics::SceneQuery::QueryType::Dynamic:
                    return inLayer == BroadPhaseLayers::Moving;
                case AzPhysics::SceneQuery::QueryType::StaticAndDynamic:
                default:
                    return true;
                }
            }

        private:
            AzPhysics::SceneQuery::QueryType m_queryType;
        };

        //! Accepts bodies whose collision layer is contained in the query's collision
        //! group mask. Filtering by object layer rather than per body lets Jolt reject
        //! whole layers before it touches a body, and it applies to every body in the
        //! scene - including ones Jolt builds itself, such as ragdoll parts.
        class SceneQueryObjectLayerFilter final : public JPH::ObjectLayerFilter
        {
        public:
            explicit SceneQueryObjectLayerFilter(AZ::u64 collisionGroupMask)
                : m_collisionGroupMask(collisionGroupMask)
            {
            }

            bool ShouldCollide(JPH::ObjectLayer inLayer) const override
            {
                return ObjectLayerMatchesQueryMask(inLayer, m_collisionGroupMask);
            }

        private:
            AZ::u64 m_collisionGroupMask;
        };

        // The collider a hit came from. Bodies keep one Physics::Shape per collider in
        // sub-shape order, so the sub-shape id the hit carries names the collider directly.
        // Null for body types that hold no shape objects (characters, ragdoll parts).
        Physics::Shape* ResolveHitShape(const AzPhysics::SimulatedBody* body, const JPH::SubShapeID& subShapeId)
        {
            if (const auto* rigidBody = azrtti_cast<const JoltRigidBody*>(body))
            {
                return rigidBody->GetShapeFromSubShapeId(subShapeId).get();
            }
            if (const auto* staticBody = azrtti_cast<const JoltStaticRigidBody*>(body))
            {
                return staticBody->GetShapeFromSubShapeId(subShapeId).get();
            }
            return nullptr;
        }

        void FillCommonHitData(
            AzPhysics::SceneQueryHit& queryHit,
            const JPH::BodyID& bodyId,
            const JPH::SubShapeID& subShapeId,
            JoltScene* scene)
        {
            queryHit.m_bodyHandle = scene->GetBodyHandleFromJoltId(bodyId);
            if (queryHit.m_bodyHandle != AzPhysics::InvalidSimulatedBodyHandle)
            {
                if (AzPhysics::SimulatedBody* body = scene->GetSimulatedBodyFromHandle(queryHit.m_bodyHandle))
                {
                    queryHit.m_entityId = body->GetEntityId();

                    queryHit.m_shape = ResolveHitShape(body, subShapeId);
                    if (queryHit.m_shape)
                    {
                        queryHit.m_resultFlags |= AzPhysics::SceneQuery::ResultFlags::Shape;
                    }

                    // Footstep audio, impact decals and surface VFX all key off the
                    // material a ray struck; the flag and field existed on the hit and
                    // were never filled, so every surface resolved to the default - which
                    // is especially galling now that per-face mesh materials are a
                    // headline feature queries could not report at all.
                    if (AZStd::shared_ptr<Physics::Material> material =
                            scene->GetMaterialInstanceForSubShape(bodyId, subShapeId))
                    {
                        queryHit.m_physicsMaterialId = material->GetId();
                        queryHit.m_resultFlags |= AzPhysics::SceneQuery::ResultFlags::Material;
                    }
                }
                queryHit.m_resultFlags |= AzPhysics::SceneQuery::ResultFlags::BodyHandle |
                                          AzPhysics::SceneQuery::ResultFlags::EntityId;
            }
        }

        AZ::Vector3 GetSurfaceNormal(
            JPH::PhysicsSystem* physicsSystem,
            const JPH::BodyID& bodyId,
            const JPH::SubShapeID& subShapeId,
            const JPH::Vec3& point)
        {
            JPH::BodyLockRead bodyLock(physicsSystem->GetBodyLockInterface(), bodyId);
            if (bodyLock.Succeeded())
            {
                return Conversions::FromJolt(bodyLock.GetBody().GetWorldSpaceSurfaceNormal(subShapeId, point));
            }
            return AZ::Vector3::CreateZero();
        }

        // Applies the request's filter callback (if any) to a candidate hit.
        // Returns true when the hit should be included in the results.
        bool PassesFilterCallback(
            const AzPhysics::SceneQueryRequest& request,
            const AzPhysics::SceneQueryHit& queryHit,
            JoltScene* scene)
        {
            const AzPhysics::SimulatedBody* body = nullptr;
            if (queryHit.m_bodyHandle != AzPhysics::InvalidSimulatedBodyHandle)
            {
                body = scene->GetSimulatedBodyFromHandle(queryHit.m_bodyHandle);
            }

            // Resolved when the hit was built, so the callback sees the same collider the
            // caller will find on the hit itself.
            Physics::Shape* shape = queryHit.m_shape;

            if (const auto* raycastRequest = azdynamic_cast<const AzPhysics::RayCastRequest*>(&request))
            {
                if (raycastRequest->m_filterCallback)
                {
                    return raycastRequest->m_filterCallback(body, shape) != AzPhysics::SceneQuery::QueryHitType::None;
                }
            }
            else if (const auto* shapecastRequest = azdynamic_cast<const AzPhysics::ShapeCastRequest*>(&request))
            {
                if (shapecastRequest->m_filterCallback)
                {
                    return shapecastRequest->m_filterCallback(body, shape) != AzPhysics::SceneQuery::QueryHitType::None;
                }
            }
            else if (const auto* overlapRequest = azdynamic_cast<const AzPhysics::OverlapRequest*>(&request))
            {
                if (overlapRequest->m_filterCallback)
                {
                    return overlapRequest->m_filterCallback(body, shape);
                }
            }

            return true;
        }

        void AppendHitIfAccepted(
            AzPhysics::SceneQueryHits& result,
            const AzPhysics::SceneQueryRequest& request,
            AzPhysics::SceneQueryHit& queryHit,
            JoltScene* scene)
        {
            if (result.m_hits.size() >= request.m_maxResults)
            {
                return;
            }
            if (PassesFilterCallback(request, queryHit, scene))
            {
                result.m_hits.push_back(queryHit);
            }
        }
    }

    //! Drops sub-shapes whose collider has Scene Queries unticked, so a collider can be
    //! solid to the simulation while staying invisible to raycasts and overlaps.
    //!
    //! A shape-level filter rather than a check on the hits: with a closest-hit collector,
    //! discarding a hit after the fact would report nothing where a farther, perfectly
    //! valid collider should have been found.
    class SceneQueryColliderFilter final : public JPH::ShapeFilter
    {
    public:
        explicit SceneQueryColliderFilter(const JoltScene* scene)
            : m_scene(scene)
        {
        }

        bool ShouldCollide(
            [[maybe_unused]] const JPH::Shape* inShape2, const JPH::SubShapeID& inSubShapeIDOfShape2) const override
        {
            return m_scene == nullptr || m_scene->IsColliderInSceneQueries(mBodyID2, inSubShapeIDOfShape2);
        }

        bool ShouldCollide(
            [[maybe_unused]] const JPH::Shape* inShape1,
            [[maybe_unused]] const JPH::SubShapeID& inSubShapeIDOfShape1,
            [[maybe_unused]] const JPH::Shape* inShape2,
            const JPH::SubShapeID& inSubShapeIDOfShape2) const override
        {
            return m_scene == nullptr || m_scene->IsColliderInSceneQueries(mBodyID2, inSubShapeIDOfShape2);
        }

    private:
        const JoltScene* m_scene = nullptr;
    };

    bool JoltSceneQueryHelpers::QueryScene(
        JoltScene* scene,
        const AzPhysics::SceneQueryRequest* request,
        AzPhysics::SceneQueryHits& result)
    {
        if (!scene || !request)
        {
            return false;
        }

        if (const auto* raycastRequest = azdynamic_cast<const AzPhysics::RayCastRequest*>(request))
        {
            return Raycast(scene, *raycastRequest, result);
        }
        else if (const auto* shapecastRequest = azdynamic_cast<const AzPhysics::ShapeCastRequest*>(request))
        {
            return ShapeCast(scene, *shapecastRequest, result);
        }
        else if (const auto* overlapRequest = azdynamic_cast<const AzPhysics::OverlapRequest*>(request))
        {
            return Overlap(scene, *overlapRequest, result);
        }

        return false;
    }

    bool JoltSceneQueryHelpers::Raycast(
        JoltScene* scene,
        const AzPhysics::RayCastRequest& request,
        AzPhysics::SceneQueryHits& result)
    {
        JPH::PhysicsSystem* physicsSystem = scene->GetJoltPhysicsSystem();
        if (!physicsSystem)
        {
            return false;
        }

        const JPH::RVec3 origin = Conversions::ToJolt(request.m_start);
        const JPH::Vec3 direction = Conversions::ToJolt(request.m_direction * request.m_distance);

        JPH::RRayCast ray(origin, direction);

        const JPH::NarrowPhaseQuery& query = physicsSystem->GetNarrowPhaseQuery();
        const QueryBroadPhaseLayerFilter broadPhaseLayerFilter(request.m_queryType);
        const SceneQueryObjectLayerFilter objectLayerFilter(request.m_collisionGroup.GetMask());
        const SceneQueryColliderFilter shapeFilter(scene);

        if (request.m_reportMultipleHits)
        {
            JPH::AllHitCollisionCollector<JPH::CastRayCollector> collector;
            query.CastRay(ray, JPH::RayCastSettings(), collector, broadPhaseLayerFilter, objectLayerFilter, JPH::BodyFilter(), shapeFilter);

            if (collector.HadHit())
            {
                collector.Sort();

                for (const auto& hit : collector.mHits)
                {
                    AzPhysics::SceneQueryHit queryHit;
                    queryHit.m_distance = hit.mFraction * request.m_distance;
                    queryHit.m_position = Conversions::FromJolt(ray.GetPointOnRay(hit.mFraction));
                    queryHit.m_normal = GetSurfaceNormal(physicsSystem, hit.mBodyID, hit.mSubShapeID2, ray.GetPointOnRay(hit.mFraction));
                    queryHit.m_resultFlags = AzPhysics::SceneQuery::ResultFlags::Distance |
                                            AzPhysics::SceneQuery::ResultFlags::Position |
                                            AzPhysics::SceneQuery::ResultFlags::Normal;
                    FillCommonHitData(queryHit, hit.mBodyID, hit.mSubShapeID2, scene);
                    AppendHitIfAccepted(result, request, queryHit, scene);
                }

                return !result.m_hits.empty();
            }
        }
        else
        {
            // The collector form rather than the single-result CastRay overload, because
            // only this one takes a shape filter - and a collider opted out of scene
            // queries has to be skipped during traversal, not discarded afterwards, or the
            // closest valid collider behind it would never be reported.
            JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> closestCollector;
            query.CastRay(ray, JPH::RayCastSettings(), closestCollector, broadPhaseLayerFilter, objectLayerFilter,
                JPH::BodyFilter(), shapeFilter);
            if (closestCollector.HadHit())
            {
                const JPH::RayCastResult& hit = closestCollector.mHit;
                AzPhysics::SceneQueryHit queryHit;
                queryHit.m_distance = hit.mFraction * request.m_distance;
                queryHit.m_position = Conversions::FromJolt(ray.GetPointOnRay(hit.mFraction));
                queryHit.m_normal = GetSurfaceNormal(physicsSystem, hit.mBodyID, hit.mSubShapeID2, ray.GetPointOnRay(hit.mFraction));
                queryHit.m_resultFlags = AzPhysics::SceneQuery::ResultFlags::Distance |
                                        AzPhysics::SceneQuery::ResultFlags::Position |
                                        AzPhysics::SceneQuery::ResultFlags::Normal;
                FillCommonHitData(queryHit, hit.mBodyID, hit.mSubShapeID2, scene);
                AppendHitIfAccepted(result, request, queryHit, scene);

                return !result.m_hits.empty();
            }
        }

        return false;
    }

    bool JoltSceneQueryHelpers::ShapeCast(
        JoltScene* scene,
        const AzPhysics::ShapeCastRequest& request,
        AzPhysics::SceneQueryHits& result)
    {
        JPH::PhysicsSystem* physicsSystem = scene->GetJoltPhysicsSystem();
        if (!physicsSystem || !request.m_shapeConfiguration)
        {
            return false;
        }

        JPH::RefConst<JPH::Shape> shape = JoltShapeUtils::CreateJoltShapeFromConfig(*request.m_shapeConfiguration);
        if (!shape)
        {
            return false;
        }

        const JPH::Vec3 direction = Conversions::ToJolt(request.m_direction * request.m_distance);
        const JPH::RMat44 startTransform = JPH::RMat44::sRotationTranslation(
            Conversions::ToJolt(request.m_start.GetRotation()),
            Conversions::ToJolt(request.m_start.GetTranslation()));
        // Jolt poses a cast shape by its centre of mass, not its origin, and a cooked
        // convex hull's centre of mass is its centroid - so handing it the world pose
        // directly would run the whole query displaced by that centroid. sFromWorldTransform
        // applies the correction; it is a no-op for the primitives, whose centre of mass
        // is already the origin.
        const JPH::RShapeCast shapeCast =
            JPH::RShapeCast::sFromWorldTransform(shape, JPH::Vec3::sReplicate(1.0f), startTransform, direction);
        // CollideShape below takes the same centre-of-mass convention, but as a plain
        // transform with no helper of its own.
        const JPH::RMat44 startTransformCOM = startTransform.PreTranslated(shape->GetCenterOfMass());

        const JPH::NarrowPhaseQuery& query = physicsSystem->GetNarrowPhaseQuery();
        const QueryBroadPhaseLayerFilter broadPhaseLayerFilter(request.m_queryType);
        const SceneQueryObjectLayerFilter objectLayerFilter(request.m_collisionGroup.GetMask());
        const SceneQueryColliderFilter shapeFilter(scene);

        JPH::ShapeCastSettings settings;

        const bool reportMultiple = request.m_reportMultipleHits;
        if (reportMultiple)
        {
            JPH::AllHitCollisionCollector<JPH::CastShapeCollector> collector;
            query.CastShape(shapeCast, settings, JPH::RVec3::sZero(), collector, broadPhaseLayerFilter, objectLayerFilter, JPH::BodyFilter(), shapeFilter);
            collector.Sort();

            for (const auto& hit : collector.mHits)
            {
                AzPhysics::SceneQueryHit queryHit;
                queryHit.m_distance = hit.mFraction * request.m_distance;
                queryHit.m_position = Conversions::FromJolt(hit.mContactPointOn2);
                queryHit.m_normal = GetSurfaceNormal(physicsSystem, hit.mBodyID2, hit.mSubShapeID2, hit.mContactPointOn2);
                queryHit.m_resultFlags = AzPhysics::SceneQuery::ResultFlags::Distance |
                                        AzPhysics::SceneQuery::ResultFlags::Position |
                                        AzPhysics::SceneQuery::ResultFlags::Normal;
                FillCommonHitData(queryHit, hit.mBodyID2, hit.mSubShapeID2, scene);
                AppendHitIfAccepted(result, request, queryHit, scene);
            }
        }
        else
        {
            JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> collector;
            query.CastShape(shapeCast, settings, JPH::RVec3::sZero(), collector, broadPhaseLayerFilter, objectLayerFilter, JPH::BodyFilter(), shapeFilter);

            if (collector.HadHit())
            {
                const JPH::ShapeCastResult& hit = collector.mHit;

                AzPhysics::SceneQueryHit queryHit;
                queryHit.m_distance = hit.mFraction * request.m_distance;
                queryHit.m_position = Conversions::FromJolt(hit.mContactPointOn2);
                queryHit.m_normal = GetSurfaceNormal(physicsSystem, hit.mBodyID2, hit.mSubShapeID2, hit.mContactPointOn2);
                queryHit.m_resultFlags = AzPhysics::SceneQuery::ResultFlags::Distance |
                                        AzPhysics::SceneQuery::ResultFlags::Position |
                                        AzPhysics::SceneQuery::ResultFlags::Normal;
                FillCommonHitData(queryHit, hit.mBodyID2, hit.mSubShapeID2, scene);
                AppendHitIfAccepted(result, request, queryHit, scene);
            }
        }

        // MTD (minimum translational distance) recovery: when the shape starts in
        // contact the cast reports no hit; report the deepest penetration instead.
        if (result.m_hits.empty() &&
            (request.m_hitFlags & AzPhysics::SceneQuery::HitFlags::MTD) == AzPhysics::SceneQuery::HitFlags::MTD)
        {
            JPH::CollideShapeSettings collideSettings;
            JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collideCollector;
            query.CollideShape(shape, JPH::Vec3::sReplicate(1.0f), startTransformCOM, collideSettings, JPH::RVec3::sZero(), collideCollector,
                broadPhaseLayerFilter, objectLayerFilter, JPH::BodyFilter(), shapeFilter);

            if (collideCollector.HadHit())
            {
                const JPH::CollideShapeResult* deepest = &collideCollector.mHits.front();
                for (const auto& hit : collideCollector.mHits)
                {
                    if (hit.mPenetrationDepth > deepest->mPenetrationDepth)
                    {
                        deepest = &hit;
                    }
                }

                AzPhysics::SceneQueryHit queryHit;
                queryHit.m_distance = 0.0f;
                queryHit.m_position = Conversions::FromJolt(deepest->mContactPointOn2);
                queryHit.m_normal = Conversions::FromJolt(-deepest->mPenetrationAxis);
                queryHit.m_resultFlags = AzPhysics::SceneQuery::ResultFlags::Distance |
                                        AzPhysics::SceneQuery::ResultFlags::Position |
                                        AzPhysics::SceneQuery::ResultFlags::Normal;
                FillCommonHitData(queryHit, deepest->mBodyID2, deepest->mSubShapeID2, scene);
                AppendHitIfAccepted(result, request, queryHit, scene);
            }
        }

        return !result.m_hits.empty();
    }

    bool JoltSceneQueryHelpers::Overlap(
        JoltScene* scene,
        const AzPhysics::OverlapRequest& request,
        AzPhysics::SceneQueryHits& result)
    {
        JPH::PhysicsSystem* physicsSystem = scene->GetJoltPhysicsSystem();
        if (!physicsSystem || !request.m_shapeConfiguration)
        {
            return false;
        }

        JPH::RefConst<JPH::Shape> shape = JoltShapeUtils::CreateJoltShapeFromConfig(*request.m_shapeConfiguration);
        if (!shape)
        {
            return false;
        }

        // PreTranslated by the shape's centre of mass: CollideShape's transform parameter
        // is a centre-of-mass transform, so a cooked convex hull handed its raw world pose
        // would overlap-test from its centroid instead of its origin. No-op for primitives.
        const JPH::RMat44 pose = JPH::RMat44::sRotationTranslation(
            Conversions::ToJolt(request.m_pose.GetRotation()),
            Conversions::ToJolt(request.m_pose.GetTranslation()))
            .PreTranslated(shape->GetCenterOfMass());

        const JPH::NarrowPhaseQuery& query = physicsSystem->GetNarrowPhaseQuery();
        const QueryBroadPhaseLayerFilter broadPhaseLayerFilter(request.m_queryType);
        const SceneQueryObjectLayerFilter objectLayerFilter(request.m_collisionGroup.GetMask());
        const SceneQueryColliderFilter shapeFilter(scene);

        JPH::CollideShapeSettings settings;
        JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
        query.CollideShape(shape, JPH::Vec3::sReplicate(1.0f), pose, settings, JPH::RVec3::sZero(), collector,
            broadPhaseLayerFilter, objectLayerFilter, JPH::BodyFilter(), shapeFilter);

        // CollideShape reports every contact point; an overlap query reports one hit per body.
        AZStd::unordered_set<AZ::u32> reportedBodies;
        for (const auto& hit : collector.mHits)
        {
            if (!reportedBodies.insert(hit.mBodyID2.GetIndexAndSequenceNumber()).second)
            {
                continue;
            }

            AzPhysics::SceneQueryHit queryHit;
            queryHit.m_resultFlags = AzPhysics::SceneQuery::ResultFlags(0);
            FillCommonHitData(queryHit, hit.mBodyID2, hit.mSubShapeID2, scene);
            AppendHitIfAccepted(result, request, queryHit, scene);
        }

        return !result.m_hits.empty();
    }

} // namespace JoltPhysics
