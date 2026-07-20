#include <Scene/JoltSceneQueryHelpers.h>
#include <Utils/Conversions.h>

#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace JoltPhysics
{
    bool JoltSceneQueryHelpers::QueryScene(
        JPH::PhysicsSystem* physicsSystem,
        const AzPhysics::SceneQueryRequest* request,
        AzPhysics::SceneQueryHits& result)
    {
        if (!physicsSystem || !request)
        {
            return false;
        }

        if (const auto* raycastRequest = azdynamic_cast<const AzPhysics::RayCastRequest*>(request))
        {
            return Raycast(physicsSystem, *raycastRequest, result);
        }
        else if (const auto* shapecastRequest = azdynamic_cast<const AzPhysics::ShapeCastRequest*>(request))
        {
            return ShapeCast(physicsSystem, *shapecastRequest, result);
        }
        else if (const auto* overlapRequest = azdynamic_cast<const AzPhysics::OverlapRequest*>(request))
        {
            return Overlap(physicsSystem, *overlapRequest, result);
        }

        return false;
    }

    bool JoltSceneQueryHelpers::Raycast(
        JPH::PhysicsSystem* physicsSystem,
        const AzPhysics::RayCastRequest& request,
        AzPhysics::SceneQueryHits& result)
    {
        if (!physicsSystem)
        {
            return false;
        }

        const JPH::RVec3 origin = Conversions::ToJolt(request.m_start);
        const JPH::Vec3 direction = Conversions::ToJolt(request.m_direction * request.m_distance);

        JPH::RRayCast ray(origin, direction);

        const JPH::NarrowPhaseQuery& query = physicsSystem->GetNarrowPhaseQuery();

        if (request.m_reportMultipleHits)
        {
            JPH::AllHitCollisionCollector<JPH::CastRayCollector> collector;
            query.CastRay(ray, JPH::RayCastSettings(), collector);

            if (collector.HadHit())
            {
                collector.Sort();

                result.m_hits.reserve(collector.mHits.size());

                for (const auto& hit : collector.mHits)
                {
                    AzPhysics::SceneQueryHit queryHit;
                    queryHit.m_distance = hit.mFraction * request.m_distance;
                    queryHit.m_position = Conversions::FromJolt(ray.GetPointOnRay(hit.mFraction));
                    queryHit.m_resultFlags = AzPhysics::SceneQuery::ResultFlags::Distance |
                                            AzPhysics::SceneQuery::ResultFlags::Position;

                    JPH::BodyLockRead bodyLock(physicsSystem->GetBodyLockInterface(), hit.mBodyID);
                    if (bodyLock.Succeeded())
                    {
                        const JPH::Body& body = bodyLock.GetBody();
                        queryHit.m_normal = Conversions::FromJolt(body.GetWorldSpaceSurfaceNormal(
                            hit.mSubShapeID2, ray.GetPointOnRay(hit.mFraction)));
                        queryHit.m_resultFlags |= AzPhysics::SceneQuery::ResultFlags::Normal;
                    }

                    result.m_hits.push_back(queryHit);
                }

                return true;
            }
        }
        else
        {
            JPH::RayCastResult hit;
            if (query.CastRay(ray, hit))
            {
                AzPhysics::SceneQueryHit queryHit;
                queryHit.m_distance = hit.mFraction * request.m_distance;
                queryHit.m_position = Conversions::FromJolt(ray.GetPointOnRay(hit.mFraction));
                queryHit.m_resultFlags = AzPhysics::SceneQuery::ResultFlags::Distance |
                                        AzPhysics::SceneQuery::ResultFlags::Position;

                JPH::BodyLockRead bodyLock(physicsSystem->GetBodyLockInterface(), hit.mBodyID);
                if (bodyLock.Succeeded())
                {
                    const JPH::Body& body = bodyLock.GetBody();
                    queryHit.m_normal = Conversions::FromJolt(body.GetWorldSpaceSurfaceNormal(
                        hit.mSubShapeID2, ray.GetPointOnRay(hit.mFraction)));
                    queryHit.m_resultFlags |= AzPhysics::SceneQuery::ResultFlags::Normal;
                }

                result.m_hits.push_back(queryHit);
                return true;
            }
        }

        return false;
    }

    bool JoltSceneQueryHelpers::ShapeCast(
        [[maybe_unused]] JPH::PhysicsSystem* physicsSystem,
        [[maybe_unused]] const AzPhysics::ShapeCastRequest& request,
        [[maybe_unused]] AzPhysics::SceneQueryHits& result)
    {
        // TODO: Implement shape cast
        return false;
    }

    bool JoltSceneQueryHelpers::Overlap(
        [[maybe_unused]] JPH::PhysicsSystem* physicsSystem,
        [[maybe_unused]] const AzPhysics::OverlapRequest& request,
        [[maybe_unused]] AzPhysics::SceneQueryHits& result)
    {
        // TODO: Implement overlap query
        return false;
    }

} // namespace JoltPhysics
