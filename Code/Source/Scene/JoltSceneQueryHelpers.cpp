#include <Scene/JoltSceneQueryHelpers.h>
#include <Scene/JoltScene.h>
#include <RigidBody/JoltRigidBody.h>
#include <RigidBody/JoltStaticRigidBody.h>
#include <SoftBody/JoltSoftBody.h>
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

        //! Keeps the nearest candidate per collider, discarding the rest.
        //!
        //! Jolt's collectors report one candidate per SUB-SHAPE, which for a triangle mesh
        //! is one per triangle. The engine's contract is one hit per collider: HitFlags
        //! documents MeshMultiple as "report all hits for meshes rather than just the
        //! first", says a single closest hit is reported when it is absent, leaves it out
        //! of HitFlags::Default, and calls it "not applicable to ShapeCast queries" at all.
        //!
        //! Measured before this existed: a 0.06 m sphere cast at a house wall returned two
        //! hits on the same collider 0.01 m apart - two triangles of one flat face. So
        //! m_maxResults was spent on duplicates, a caller counting distinct obstacles
        //! over-counted, and the answer changed depending on whether a filter callback was
        //! present, because a callback returning Block ends the loop and hides the rest.
        //!
        //! Keyed on the resolved collider rather than the body, so a compound body still
        //! reports each of its colliders and only the sub-shapes WITHIN one collapse. A
        //! body that owns no shape objects - a character, a ragdoll part - resolves to
        //! null and collapses to a single hit, which is what it has.
        class ColliderHitCoalescer
        {
        public:
            //! True the first time this (body, collider) pair is offered; false after.
            bool ShouldReport(const JPH::BodyID& bodyId, const Physics::Shape* shape)
            {
                const Entry entry{ bodyId.GetIndexAndSequenceNumber(), shape };
                // Linear, deliberately: what this scans is the number of DISTINCT colliders
                // the cast crossed, which stays tiny even when the candidate list is
                // thousands of triangles of one terrain mesh.
                for (const Entry& seen : m_seen)
                {
                    if (seen.m_bodyId == entry.m_bodyId && seen.m_shape == entry.m_shape)
                    {
                        return false;
                    }
                }
                m_seen.push_back(entry);
                return true;
            }

        private:
            struct Entry
            {
                AZ::u32 m_bodyId;
                const Physics::Shape* m_shape;
            };
            AZStd::vector<Entry> m_seen;
        };

        //! Whether a cast should collapse a mesh's triangles into one hit per collider.
        //! MeshMultiple opts out, and it is not in HitFlags::Default. AnyHit needs no
        //! handling here: it asks for *any* hit rather than the closest, and one hit per
        //! collider satisfies that.
        bool ShouldCoalescePerCollider(AzPhysics::SceneQuery::HitFlags hitFlags)
        {
            return (hitFlags & AzPhysics::SceneQuery::HitFlags::MeshMultiple) !=
                AzPhysics::SceneQuery::HitFlags::MeshMultiple;
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

        //! Runs the request's filter callback before narrow phase, for the queries where
        //! that is the difference between filtering a hit and never generating it.
        //!
        //! Colliding a shape against a soft body walks its faces; a face crushed to no area
        //! has no normal to seed GJK with, and Jolt reports an assertion. The callback that
        //! would have skipped that body was only ever applied to hits already returned - so
        //! a caller who knew the problem, read the note in KNOWN_ISSUES and wrote a filter
        //! to avoid cloth still walked those faces. Consulted here, the body is rejected in
        //! the broad phase and the faces are never touched.
        //!
        //! Only soft bodies are put to the callback here, and only because of that
        //! assertion. Everything else is left to the per-hit call, which can hand the
        //! callback the collider it actually struck. Asking about every body at this point
        //! meant the callback was invoked twice for each one - once now with a null shape,
        //! once later with the real one - so a callback that dereferenced the shape
        //! crashed on the first call and one that counted what it saw counted double.
        class SceneQueryPreNarrowPhaseBodyFilter final : public JPH::BodyFilter
        {
        public:
            SceneQueryPreNarrowPhaseBodyFilter(const AzPhysics::OverlapRequest& request, JoltScene* scene)
                : m_request(request)
                , m_scene(scene)
            {
            }

            bool ShouldCollideLocked(const JPH::Body& body) const override
            {
                if (!m_request.m_filterCallback || !m_scene)
                {
                    return true;
                }

                const AzPhysics::SimulatedBody* simulatedBody =
                    m_scene->GetSimulatedBodyFromHandle(m_scene->GetBodyHandleFromJoltId(body.GetID()));
                if (!simulatedBody || azrtti_cast<const JoltSoftBody*>(simulatedBody) == nullptr)
                {
                    return true;
                }
                // OverlapFilterCallback returns bool, unlike the raycast and shapecast
                // callbacks which return a QueryHitType. A soft body has no Physics::Shape
                // to name, so the null here is the same answer the per-hit call would give.
                return m_request.m_filterCallback(simulatedBody, nullptr);
            }

        private:
            const AzPhysics::OverlapRequest& m_request;
            JoltScene* m_scene;
        };

        //! Fills a shape-cast hit from a Jolt result, using the penetration data when the
        //! cast started already overlapping.
        //!
        //! Jolt reports such a body as a hit at fraction 0 carrying a penetration depth
        //! and axis. Read as an ordinary hit that gives a distance of zero and the surface
        //! normal at the contact point - neither of which tells a caller which way to move
        //! to get out, which is the whole point of asking for MTD. The separate
        //! CollideShape pass below was meant to cover this and almost never ran, because
        //! it only runs when there were no hits at all and an initial overlap is a hit.
        void FillShapeCastHit(
            AzPhysics::SceneQueryHit& queryHit,
            const JPH::ShapeCastResult& hit,
            const AzPhysics::ShapeCastRequest& request,
            JPH::PhysicsSystem* physicsSystem)
        {
            const bool startedInContact = hit.mFraction <= 0.0f && hit.mPenetrationDepth > 0.0f;
            const bool wantsMtd =
                (request.m_hitFlags & AzPhysics::SceneQuery::HitFlags::MTD) == AzPhysics::SceneQuery::HitFlags::MTD;

            queryHit.m_position = Conversions::FromJolt(hit.mContactPointOn2);
            if (startedInContact && wantsMtd)
            {
                // Negative distance is the depth, and the normal is the direction out,
                // which is the convention the CollideShape recovery below already used.
                queryHit.m_distance = -hit.mPenetrationDepth;
                queryHit.m_normal = Conversions::FromJolt(-hit.mPenetrationAxis.Normalized());
            }
            else
            {
                queryHit.m_distance = hit.mFraction * request.m_distance;
                queryHit.m_normal =
                    GetSurfaceNormal(physicsSystem, hit.mBodyID2, hit.mSubShapeID2, hit.mContactPointOn2);
            }
            queryHit.m_resultFlags = AzPhysics::SceneQuery::ResultFlags::Distance |
                                    AzPhysics::SceneQuery::ResultFlags::Position |
                                    AzPhysics::SceneQuery::ResultFlags::Normal;
        }

        //! What the request's filter callback makes of a candidate hit. Touch when there
        //! is no callback. Block means "report this one and stop looking": the engine
        //! defines Touch as reported but not blocking and Block as reported and blocking,
        //! and collapsing the two to a yes/no meant a multi-hit cast kept collecting past
        //! a body the caller had said should stop it.
        AzPhysics::SceneQuery::QueryHitType ClassifyHit(
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
                    return raycastRequest->m_filterCallback(body, shape);
                }
            }
            else if (const auto* shapecastRequest = azdynamic_cast<const AzPhysics::ShapeCastRequest*>(&request))
            {
                if (shapecastRequest->m_filterCallback)
                {
                    return shapecastRequest->m_filterCallback(body, shape);
                }
            }
            else if (const auto* overlapRequest = azdynamic_cast<const AzPhysics::OverlapRequest*>(&request))
            {
                if (overlapRequest->m_filterCallback)
                {
                    // An overlap's callback answers yes or no; there is nothing behind a
                    // hit for a Block to stop.
                    return overlapRequest->m_filterCallback(body, shape)
                        ? AzPhysics::SceneQuery::QueryHitType::Touch
                        : AzPhysics::SceneQuery::QueryHitType::None;
                }
            }

            return AzPhysics::SceneQuery::QueryHitType::Touch;
        }

        bool PassesFilterCallback(
            const AzPhysics::SceneQueryRequest& request,
            const AzPhysics::SceneQueryHit& queryHit,
            JoltScene* scene)
        {
            return ClassifyHit(request, queryHit, scene) != AzPhysics::SceneQuery::QueryHitType::None;
        }

        //! How many hits this request may return: what it asked for, bounded by the
        //! system configuration's cap for its kind. The engine's own comment on
        //! m_maxResults says as much - "this is limited by the value set in the
        //! SceneConfiguration" - and those caps were read by nothing here, so a project
        //! that raised or lowered one saw no difference.
        AZ::u32 ResultCapFor(const AzPhysics::SceneQueryRequest& request, JoltScene* scene)
        {
            if (scene == nullptr)
            {
                return request.m_maxResults;
            }
            AZ::u32 bufferSize = scene->GetRaycastBufferSize();
            if (azrtti_istypeof<const AzPhysics::ShapeCastRequest*>(&request))
            {
                bufferSize = scene->GetShapecastBufferSize();
            }
            else if (azrtti_istypeof<const AzPhysics::OverlapRequest*>(&request))
            {
                bufferSize = scene->GetOverlapBufferSize();
            }
            return AZStd::min(request.m_maxResults, bufferSize);
        }

        //! Appends the hit if the filter accepts it, and says what the filter made of it
        //! so a caller walking hits in order knows whether to stop. None means the hit was
        //! rejected or the result cap was already reached.
        AzPhysics::SceneQuery::QueryHitType AppendHitIfAccepted(
            AzPhysics::SceneQueryHits& result,
            const AzPhysics::SceneQueryRequest& request,
            AzPhysics::SceneQueryHit& queryHit,
            JoltScene* scene)
        {
            if (result.m_hits.size() >= ResultCapFor(request, scene))
            {
                return AzPhysics::SceneQuery::QueryHitType::None;
            }
            const AzPhysics::SceneQuery::QueryHitType hitType = ClassifyHit(request, queryHit, scene);
            if (hitType != AzPhysics::SceneQuery::QueryHitType::None)
            {
                result.m_hits.push_back(queryHit);
            }
            return hitType;
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

        // A filter callback has to be consulted during the search, not after it. The
        // closest-hit collector keeps one hit chosen by distance alone, so a callback that
        // rejects it - "not the body I am standing on", "not my own car" - left the caller
        // with no hit at all, when the whole point of the callback was to be told what is
        // behind the thing it rejected. Walking every candidate in order costs more, so it
        // is only done when there is a callback to make it necessary. The same reasoning
        // is already written down for SceneQueryColliderFilter.
        const bool collectEveryCandidate = request.m_reportMultipleHits || request.m_filterCallback != nullptr;

        if (collectEveryCandidate)
        {
            JPH::AllHitCollisionCollector<JPH::CastRayCollector> collector;
            query.CastRay(ray, JPH::RayCastSettings(), collector, broadPhaseLayerFilter, objectLayerFilter, JPH::BodyFilter(), shapeFilter);

            if (collector.HadHit())
            {
                collector.Sort();

                // A raycast is the one query MeshMultiple applies to; a caller that wants
                // every triangle asks for it explicitly.
                const bool coalescePerCollider = ShouldCoalescePerCollider(request.m_hitFlags);
                ColliderHitCoalescer coalescer;

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

                    // Checked after FillCommonHitData, which is what resolves the collider,
                    // and before the filter callback, so the callback is asked once per
                    // collider rather than once per triangle.
                    if (coalescePerCollider && !coalescer.ShouldReport(hit.mBodyID, queryHit.m_shape))
                    {
                        continue;
                    }

                    const AzPhysics::SceneQuery::QueryHitType hitType =
                        AppendHitIfAccepted(result, request, queryHit, scene);

                    if (!request.m_reportMultipleHits && !result.m_hits.empty())
                    {
                        // Single-hit mode: the nearest candidate the callback accepted is
                        // the answer, and the rest of the sorted list is behind it.
                        break;
                    }
                    if (hitType == AzPhysics::SceneQuery::QueryHitType::Block)
                    {
                        // "Reported, and it should block the query": everything left in
                        // the sorted list is further away and behind this one.
                        break;
                    }
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

        // See the note in Raycast: a filter callback that rejects the nearest hit must
        // leave the caller with the next one, not with nothing, so every candidate is
        // collected whenever there is a callback to reject one.
        const bool collectEveryCandidate = request.m_reportMultipleHits || request.m_filterCallback != nullptr;
        if (collectEveryCandidate)
        {
            JPH::AllHitCollisionCollector<JPH::CastShapeCollector> collector;
            query.CastShape(shapeCast, settings, JPH::RVec3::sZero(), collector, broadPhaseLayerFilter, objectLayerFilter, JPH::BodyFilter(), shapeFilter);
            collector.Sort();

            // Unconditional for a shape cast: HitFlags calls MeshMultiple "not applicable
            // to ShapeCast queries", so there is no way to ask for the triangles.
            ColliderHitCoalescer coalescer;

            for (const auto& hit : collector.mHits)
            {
                AzPhysics::SceneQueryHit queryHit;
                FillShapeCastHit(queryHit, hit, request, physicsSystem);
                FillCommonHitData(queryHit, hit.mBodyID2, hit.mSubShapeID2, scene);

                if (!coalescer.ShouldReport(hit.mBodyID2, queryHit.m_shape))
                {
                    continue;
                }

                const AzPhysics::SceneQuery::QueryHitType hitType =
                    AppendHitIfAccepted(result, request, queryHit, scene);

                if (!request.m_reportMultipleHits && !result.m_hits.empty())
                {
                    // Single-hit mode: the nearest candidate the callback accepted.
                    break;
                }
                if (hitType == AzPhysics::SceneQuery::QueryHitType::Block)
                {
                    break;
                }
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
                FillShapeCastHit(queryHit, hit, request, physicsSystem);
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
        const SceneQueryPreNarrowPhaseBodyFilter bodyFilter(request, scene);
        query.CollideShape(shape, JPH::Vec3::sReplicate(1.0f), pose, settings, JPH::RVec3::sZero(), collector,
            broadPhaseLayerFilter, objectLayerFilter, bodyFilter, shapeFilter);

        // An unbounded overlap hands each hit to the caller as it is found instead of
        // building a vector, which is the whole point of it: a query expecting thousands
        // of hits should not pay for storing them, and m_maxResults does not apply. The
        // engine states the protocol exactly - each hit in turn, a false return ends the
        // query, and a final empty optional says there are no more (PhysicsSceneQueries.h).
        // It was read by nothing here, so such a request quietly behaved like an ordinary
        // overlap and stopped at the cap, and the callback never fired at all.
        const bool unbounded = request.m_unboundedOverlapHitCallback != nullptr;
        bool deliveredAnyHit = false;
        bool callerWantsMore = true;

        // CollideShape reports every contact point; an overlap query reports one hit per body.
        AZStd::unordered_set<AZ::u32> reportedBodies;
        for (const auto& hit : collector.mHits)
        {
            if (!callerWantsMore)
            {
                break;
            }
            if (!reportedBodies.insert(hit.mBodyID2.GetIndexAndSequenceNumber()).second)
            {
                continue;
            }

            AzPhysics::SceneQueryHit queryHit;
            queryHit.m_resultFlags = AzPhysics::SceneQuery::ResultFlags(0);
            FillCommonHitData(queryHit, hit.mBodyID2, hit.mSubShapeID2, scene);

            // A soft body was already put to the callback before narrow phase, and asking
            // again would be the double call this filter exists to avoid.
            const bool alreadyFiltered = queryHit.m_bodyHandle != AzPhysics::InvalidSimulatedBodyHandle &&
                azrtti_cast<const JoltSoftBody*>(scene->GetSimulatedBodyFromHandle(queryHit.m_bodyHandle)) != nullptr;

            if (unbounded)
            {
                if (!alreadyFiltered && !PassesFilterCallback(request, queryHit, scene))
                {
                    continue;
                }
                deliveredAnyHit = true;
                callerWantsMore =
                    request.m_unboundedOverlapHitCallback(AZStd::optional<AzPhysics::SceneQueryHit>(AZStd::move(queryHit)));
            }
            else if (alreadyFiltered)
            {
                if (result.m_hits.size() < ResultCapFor(request, scene))
                {
                    result.m_hits.push_back(queryHit);
                }
            }
            else
            {
                AppendHitIfAccepted(result, request, queryHit, scene);
            }
        }

        if (unbounded)
        {
            // "then called with {}, then never called again" - but only if the caller did
            // not end the query itself, where the engine leaves the final call unspecified.
            if (callerWantsMore)
            {
                request.m_unboundedOverlapHitCallback(AZStd::optional<AzPhysics::SceneQueryHit>{});
            }
            return deliveredAnyHit;
        }

        return !result.m_hits.empty();
    }

} // namespace JoltPhysics
