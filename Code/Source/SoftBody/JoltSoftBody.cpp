#include <SoftBody/JoltSoftBody.h>

#include <AzCore/Debug/Profiler.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/parallel/lock.h>

#include <cmath>

#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Physics/WindBus.h>

#include <Scene/JoltScene.h>
#include <Shape/JoltMeshUtils.h>
#include <System/CollisionLayerFilters.h>
#include <Utils/Conversions.h>
#include <Utils/JoltDiagnostics.h>

#include <Jolt/Core/TempAllocator.h>

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>

namespace JoltPhysics
{
    namespace
    {
        JPH::RVec3 ToJoltR(const AZ::Vector3& v)
        {
            return JPH::RVec3(v.GetX(), v.GetY(), v.GetZ());
        }

        AZ::Vector3 FromJolt(const JPH::Vec3& v)
        {
            return AZ::Vector3(v.GetX(), v.GetY(), v.GetZ());
        }

        JPH::Quat ToJolt(const AZ::Quaternion& q)
        {
            return JPH::Quat(q.GetX(), q.GetY(), q.GetZ(), q.GetW());
        }

        //! Jolt collides a convex shape against a soft body by walking its faces and seeding
        //! GJK with each triangle's normal - the raw cross product, not a normalised one -
        //! and it asserts if that seed is near zero, because a face with no area gives no
        //! direction to separate along. `SoftBodyShape::sCollideConvexVsSoftBody` walks the
        //! faces, `CollideConvexVsTriangles::Collide` computes the normal, and the assert
        //! itself is `JPH_ASSERT(!ioV.IsNearZero())` in `EPAPenetrationDepth.h`, whose
        //! comment names a degenerate triangle as the likely cause.
        //!
        //! The tolerance is Vec3::IsNearZero's default of 1e-12 on the squared length, so a
        //! triangle is unusable to Jolt below about 5e-7 m^2 - roughly a millimetre on a
        //! side. That is Jolt's limit rather than a threshold worth choosing, so match it
        //! exactly and drop no more than it cannot handle.
        //!
        //! This matters because nothing is checked until something touches the body. A
        //! sliver sits there simulating quite happily until the first collision query
        //! reaches it, and Jolt then reports an assert from a stack that names neither the
        //! mesh nor us. Art meshes carry slivers all the time: hair cards, crests, fans
        //! collapsed to a point.
        //!
        //! Every face this gem builds goes through here - the procedural shapes via AddQuad
        //! below as well as authored geometry - because the procedural ones are no more
        //! immune than a mesh: a Cloth 1 mm on a side, or one whose Size has a zero
        //! component, produces exactly the same unusable triangle from values the editor
        //! is happy to offer.
        constexpr float MinFaceCrossLengthSq = 1.0e-12f;

        bool AddTriangle(
            JPH::SoftBodySharedSettings& settings, AZStd::vector<AZ::u32>& indices, AZ::u32 a, AZ::u32 b, AZ::u32 c)
        {
            if (a == b || b == c || a == c)
            {
                return false;
            }
            const size_t vertexCount = settings.mVertices.size();
            if (a >= vertexCount || b >= vertexCount || c >= vertexCount)
            {
                return false;
            }

            const auto position = [&settings](AZ::u32 index)
            {
                const JPH::Float3& p = settings.mVertices[index].mPosition;
                return AZ::Vector3(p.x, p.y, p.z);
            };
            const AZ::Vector3 edge1 = position(b) - position(a);
            const AZ::Vector3 edge2 = position(c) - position(a);
            if (edge1.Cross(edge2).GetLengthSq() <= MinFaceCrossLengthSq)
            {
                return false;
            }

            settings.AddFace(JPH::SoftBodySharedSettings::Face(a, b, c));
            indices.insert(indices.end(), { a, b, c });
            return true;
        }

        //! Two triangles per grid quad, wound counter-clockwise seen from +Z so the cloth's
        //! front face points the same way the entity does. Returns how many of the two were
        //! unusable, so a caller building a whole surface can report the total.
        size_t AddQuad(
            JPH::SoftBodySharedSettings& settings,
            AZStd::vector<AZ::u32>& indices,
            AZ::u32 a,
            AZ::u32 b,
            AZ::u32 c,
            AZ::u32 d)
        {
            size_t dropped = 0;
            dropped += AddTriangle(settings, indices, a, b, c) ? 0 : 1;
            dropped += AddTriangle(settings, indices, a, c, d) ? 0 : 1;
            return dropped;
        }

        //! Fills the triangle list from faces Jolt generated itself, for the shapes built by
        //! a Jolt helper rather than here. Jolt's own generators do not produce degenerate
        //! faces, so this copies rather than filters; it is kept honest by
        //! DropDegenerateFaces below, which the caller runs over the result.
        void CollectFaces(const JPH::SoftBodySharedSettings& settings, AZStd::vector<AZ::u32>& indices)
        {
            indices.clear();
            indices.reserve(settings.mFaces.size() * 3);
            for (const JPH::SoftBodySharedSettings::Face& face : settings.mFaces)
            {
                indices.insert(indices.end(), { face.mVertex[0], face.mVertex[1], face.mVertex[2] });
            }
        }

        //! Rebuilds a face list keeping only what Jolt can collide against, for geometry
        //! that arrived already assembled (Jolt's own shape generators, or a cooked mesh).
        //! Returns the number dropped. Faces are rebuilt in place rather than erased one at
        //! a time because Jolt's Face carries no removal API.
        size_t DropDegenerateFaces(JPH::SoftBodySharedSettings& settings, AZStd::vector<AZ::u32>& indices)
        {
            JPH::Array<JPH::SoftBodySharedSettings::Face> kept;
            kept.reserve(settings.mFaces.size());
            AZStd::vector<AZ::u32> keptIndices;
            keptIndices.reserve(indices.size());

            size_t dropped = 0;
            for (const JPH::SoftBodySharedSettings::Face& face : settings.mFaces)
            {
                const AZ::u32 a = face.mVertex[0];
                const AZ::u32 b = face.mVertex[1];
                const AZ::u32 c = face.mVertex[2];
                const size_t vertexCount = settings.mVertices.size();
                bool usable = a != b && b != c && a != c && a < vertexCount && b < vertexCount && c < vertexCount;
                if (usable)
                {
                    const auto position = [&settings](AZ::u32 index)
                    {
                        const JPH::Float3& p = settings.mVertices[index].mPosition;
                        return AZ::Vector3(p.x, p.y, p.z);
                    };
                    usable = (position(b) - position(a)).Cross(position(c) - position(a)).GetLengthSq() >
                        MinFaceCrossLengthSq;
                }

                if (usable)
                {
                    kept.push_back(face);
                    keptIndices.insert(keptIndices.end(), { a, b, c });
                }
                else
                {
                    ++dropped;
                }
            }

            if (dropped > 0)
            {
                settings.mFaces = AZStd::move(kept);
                indices = AZStd::move(keptIndices);
            }
            return dropped;
        }

        //! Particles that no surviving face refers to. Left free they would fall out of the
        //! body forever - Jolt builds its edge, shear and bend constraints by walking faces,
        //! so a particle with no face has no constraint holding it to anything, and gravity
        //! is the only thing still acting on it. Their indices have to stay valid, because
        //! JoltCloth maps render vertices onto particle indices and would scatter the whole
        //! mesh wrong if this compacted the array, so they are pinned where they are instead
        //! of removed.
        size_t PinOrphanParticles(JPH::SoftBodySharedSettings& settings)
        {
            AZStd::vector<bool> referenced(settings.mVertices.size(), false);
            for (const JPH::SoftBodySharedSettings::Face& face : settings.mFaces)
            {
                for (const JPH::uint32 vertex : face.mVertex)
                {
                    if (vertex < referenced.size())
                    {
                        referenced[vertex] = true;
                    }
                }
            }

            size_t orphans = 0;
            for (size_t vertex = 0; vertex < settings.mVertices.size(); ++vertex)
            {
                if (!referenced[vertex] && settings.mVertices[vertex].mInvMass != 0.0f)
                {
                    settings.mVertices[vertex].mInvMass = 0.0f;
                    ++orphans;
                }
            }
            return orphans;
        }

        //! Spreads the requested total mass over the free particles. Jolt works in inverse
        //! mass, and a zero inverse mass is what pins a particle: a pinned particle has
        //! infinite mass by definition, so it takes no share of the total. Dividing by the
        //! full vertex count instead would silently shave the pinned particles' share off
        //! the body - a corner-pinned 6x6 cloth would weigh 32/36 of what was configured.
        //! Returns the inverse mass each free particle was given, so a runtime unpin can
        //! restore exactly that share.
        float DistributeMass(JPH::SoftBodySharedSettings& settings, float totalMass)
        {
            size_t freeCount = 0;
            for (const JPH::SoftBodySharedSettings::Vertex& vertex : settings.mVertices)
            {
                if (vertex.mInvMass > 0.0f)
                {
                    ++freeCount;
                }
            }

            // A fully pinned body has nothing to distribute over; the share a runtime
            // unpin would then restore is what an even split over everything would give.
            const size_t shareCount = freeCount > 0 ? freeCount : settings.mVertices.size();
            if (shareCount == 0)
            {
                return 1.0f;
            }

            const float perVertexMass = AZ::GetMax(totalMass, 0.001f) / static_cast<float>(shareCount);
            const float inverseMass = 1.0f / perVertexMass;
            for (JPH::SoftBodySharedSettings::Vertex& vertex : settings.mVertices)
            {
                if (vertex.mInvMass > 0.0f)
                {
                    vertex.mInvMass = inverseMass;
                }
            }
            return inverseMass;
        }

        //! The first triangle mesh in a cooked .joltmesh asset - the surface a Mesh-shaped
        //! soft body simulates. Convex entries are skipped: a hull has no interior surface
        //! to drape.
        const Physics::CookedMeshShapeConfiguration* FindTriangleMeshConfiguration(const Pipeline::JoltMeshAsset* asset)
        {
            if (!asset)
            {
                return nullptr;
            }
            for (const auto& shapePair : asset->m_assetData.m_colliderShapes)
            {
                if (const auto* cooked = azrtti_cast<const Physics::CookedMeshShapeConfiguration*>(shapePair.second.get());
                    cooked && cooked->GetMeshType() == Physics::CookedMeshShapeConfiguration::MeshType::TriangleMesh)
                {
                    return cooked;
                }
            }
            return nullptr;
        }

        //! Merges vertices that share a position (within a tenth of a millimetre) and
        //! remaps the triangles. Render-derived meshes split vertices along every normal
        //! and UV seam; simulated unwelded, the sheet would tear apart at each seam
        //! because the constraint generation only connects vertices that share faces.
        //! Degenerate triangles produced by the welding are dropped.
        void WeldVertices(
            const AZStd::vector<AZ::Vector3>& vertices,
            const AZStd::vector<AZ::u32>& indices,
            AZStd::vector<AZ::Vector3>& outVertices,
            AZStd::vector<AZ::u32>& outIndices)
        {
            struct WeldKey
            {
                AZ::s64 m_x;
                AZ::s64 m_y;
                AZ::s64 m_z;
                bool operator==(const WeldKey& other) const
                {
                    return m_x == other.m_x && m_y == other.m_y && m_z == other.m_z;
                }
            };
            struct WeldKeyHasher
            {
                size_t operator()(const WeldKey& key) const
                {
                    size_t seed = 0;
                    AZStd::hash_combine(seed, key.m_x, key.m_y, key.m_z);
                    return seed;
                }
            };

            constexpr float weldGridSize = 1.0e-4f;
            AZStd::unordered_map<WeldKey, AZ::u32, WeldKeyHasher> weldedIndexByPosition;
            AZStd::vector<AZ::u32> remap(vertices.size());

            for (size_t i = 0; i < vertices.size(); ++i)
            {
                const AZ::Vector3& position = vertices[i];
                const WeldKey key{
                    static_cast<AZ::s64>(AZStd::round(position.GetX() / weldGridSize)),
                    static_cast<AZ::s64>(AZStd::round(position.GetY() / weldGridSize)),
                    static_cast<AZ::s64>(AZStd::round(position.GetZ() / weldGridSize)),
                };
                auto [it, inserted] = weldedIndexByPosition.try_emplace(key, static_cast<AZ::u32>(outVertices.size()));
                if (inserted)
                {
                    outVertices.push_back(position);
                }
                remap[i] = it->second;
            }

            outIndices.reserve(indices.size());
            for (size_t i = 0; i + 2 < indices.size(); i += 3)
            {
                const AZ::u32 a = remap[indices[i]];
                const AZ::u32 b = remap[indices[i + 1]];
                const AZ::u32 c = remap[indices[i + 2]];
                if (a != b && b != c && a != c)
                {
                    outIndices.insert(outIndices.end(), { a, b, c });
                }
            }
        }
    } // namespace

    void JoltSoftBodyConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltSoftBodyConfiguration, AzPhysics::SimulatedBodyConfiguration>()
                ->Version(1)
                ->Field("Settings", &JoltSoftBodyConfiguration::m_settings)
                ;
        }
    }

    JoltSoftBody::~JoltSoftBody()
    {
        Detach();
    }

    JoltSoftBody::JoltSoftBody(const JoltSoftBodyConfiguration& configuration)
        : m_settings(configuration.m_settings)
        , m_worldTransform(AZ::Transform::CreateFromQuaternionAndTranslation(
              configuration.m_orientation, configuration.m_position))
    {
        m_entityId = configuration.m_entityId;
    }

    void JoltSoftBody::CreateInScene(JoltScene* scene)
    {
        m_scene = scene;
        if (!scene)
        {
            return;
        }

        AttachToPhysicsSystem(
            scene->GetJoltPhysicsSystem(),
            AcquireObjectLayer(
                m_settings.m_collisionLayer, m_settings.m_collisionGroupId, /*isMoving*/ true,
                JoltBodyClass::SoftBody));
    }

    void JoltSoftBody::RemoveFromJoltWorld()
    {
        AZStd::lock_guard lock(m_mutex);
        DestroyBody();
    }

    JPH::BodyID JoltSoftBody::GetBodyId() const
    {
        AZStd::lock_guard lock(m_mutex);
        return m_bodyId;
    }

    AZ::Crc32 JoltSoftBody::GetNativeType() const
    {
        return AZ_CRC_CE("JoltSoftBody");
    }

    void* JoltSoftBody::GetNativePointer() const
    {
        // The JPH::BodyID, the same contract as every other body type in this gem, so a
        // generic consumer can cast any body's native pointer to JPH::BodyID*. The id is
        // invalid while the body is detached.
        return const_cast<JPH::BodyID*>(&m_bodyId);
    }

    AZ::EntityId JoltSoftBody::GetEntityId() const
    {
        return m_entityId;
    }

    AZ::Transform JoltSoftBody::GetTransform() const
    {
        AZStd::lock_guard lock(m_mutex);
        return m_worldTransform;
    }

    AZ::Vector3 JoltSoftBody::GetPosition() const
    {
        AZStd::lock_guard lock(m_mutex);
        return m_worldTransform.GetTranslation();
    }

    AZ::Quaternion JoltSoftBody::GetOrientation() const
    {
        AZStd::lock_guard lock(m_mutex);
        return m_worldTransform.GetRotation();
    }

    AZ::Aabb JoltSoftBody::GetAabb() const
    {
        return GetWorldBounds();
    }

    AzPhysics::SceneQueryHit JoltSoftBody::RayCast(const AzPhysics::RayCastRequest& request)
    {
        AzPhysics::SceneQueryHit queryHit;

        AZStd::lock_guard lock(m_mutex);
        if (!m_physicsSystem || m_bodyId.IsInvalid())
        {
            return queryHit;
        }

        JPH::BodyLockRead bodyLock(m_physicsSystem->GetBodyLockInterface(), m_bodyId);
        if (!bodyLock.Succeeded())
        {
            return queryHit;
        }

        const JPH::Body& body = bodyLock.GetBody();

        // Cast in body-local space, as the rigid bodies do. A soft body's shape tracks its
        // particles, so this hits the deformed surface rather than the rest shape.
        const JPH::Mat44 worldToLocal = body.GetInverseCenterOfMassTransform();
        const JPH::Vec3 localStart = worldToLocal * Conversions::ToJolt(request.m_start);
        const JPH::Vec3 localDirection =
            worldToLocal.Multiply3x3(Conversions::ToJolt(request.m_direction * request.m_distance));

        // The collector overload rather than the simple one: only it consults the
        // body's double-sided flag, the simple overload hits back faces regardless.
        JPH::RayCast ray(localStart, localDirection);
        JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector;
        body.GetShape()->CastRay(ray, JPH::RayCastSettings(), JPH::SubShapeIDCreator(), collector);
        if (collector.HadHit())
        {
            const JPH::RayCastResult& hit = collector.mHit;
            queryHit.m_distance = hit.mFraction * request.m_distance;
            queryHit.m_position = request.m_start + request.m_direction * (hit.mFraction * request.m_distance);
            queryHit.m_normal = Conversions::FromJolt(
                body.GetWorldSpaceSurfaceNormal(hit.mSubShapeID2, Conversions::ToJolt(queryHit.m_position)));
            queryHit.m_resultFlags = AzPhysics::SceneQuery::ResultFlags::Distance |
                AzPhysics::SceneQuery::ResultFlags::Position | AzPhysics::SceneQuery::ResultFlags::Normal |
                AzPhysics::SceneQuery::ResultFlags::BodyHandle | AzPhysics::SceneQuery::ResultFlags::EntityId;
            queryHit.m_bodyHandle = m_bodyHandle;
            queryHit.m_entityId = m_entityId;
        }

        return queryHit;
    }

    bool JoltSoftBody::Attach(AzPhysics::SceneHandle sceneHandle)
    {
        auto* systemInterface = AZ::Interface<AzPhysics::SystemInterface>::Get();
        if (!systemInterface || sceneHandle == AzPhysics::InvalidSceneHandle)
        {
            return false;
        }

        // The cast is safe because this gem owns the scene type. A scene belonging to some
        // other physics backend fails the cast and the soft body simply does nothing.
        auto* joltScene = azrtti_cast<JoltScene*>(systemInterface->GetScene(sceneHandle));
        if (!joltScene)
        {
            return false;
        }

        // Straight to the layer registry every body in this gem goes through. Soft bodies
        // used to reach it through JoltPhysicsSystemRequestBus::AcquireObjectLayer when they
        // lived in a separate gem; inside the gem that round trip buys nothing.
        // Soft bodies are always moving: there is no static variety.
        const JPH::ObjectLayer objectLayer =
            AcquireObjectLayer(
                m_settings.m_collisionLayer, m_settings.m_collisionGroupId, /*isMoving*/ true,
                JoltBodyClass::SoftBody);

        return AttachToPhysicsSystem(joltScene->GetJoltPhysicsSystem(), objectLayer);
    }

    bool JoltSoftBody::AttachToPhysicsSystem(JPH::PhysicsSystem* physicsSystem, JPH::ObjectLayer objectLayer)
    {
        Detach();

        if (!physicsSystem)
        {
            return false;
        }

        AZStd::lock_guard lock(m_mutex);
        m_physicsSystem = physicsSystem;
        m_objectLayer = objectLayer;
        return CreateBody();
    }

    void JoltSoftBody::Detach()
    {
        AZStd::lock_guard lock(m_mutex);
        DestroyBody();
        m_physicsSystem = nullptr;
    }

    bool JoltSoftBody::IsAttached() const
    {
        AZStd::lock_guard lock(m_mutex);
        return m_physicsSystem != nullptr && !m_bodyId.IsInvalid();
    }

    void JoltSoftBody::SetTransform(const AZ::Transform& worldTransform)
    {
        AZStd::lock_guard lock(m_mutex);
        m_worldTransform = worldTransform;

        // Teleporting a live soft body would leave its particles behind and the solver
        // would snap them back, so the placement is applied by rebuilding instead.
        if (m_physicsSystem && !m_bodyId.IsInvalid())
        {
            DestroyBody();
            CreateBody();
        }
    }

    void JoltSoftBody::SetSettings(const JoltSoftBodySettings& settings)
    {
        AZStd::lock_guard lock(m_mutex);

        // Only the baked fields need the particle layout regenerated; changing damping or
        // pressure every frame must not rebuild the body.
        const bool needsRebuild = settings.m_shape != m_settings.m_shape || settings.m_pinning != m_settings.m_pinning ||
            settings.m_meshAsset.GetId() != m_settings.m_meshAsset.GetId() ||
            !settings.m_size.IsClose(m_settings.m_size) || settings.m_resolution != m_settings.m_resolution ||
            !AZ::IsClose(settings.m_mass, m_settings.m_mass) || !AZ::IsClose(settings.m_compliance, m_settings.m_compliance) ||
            settings.m_lraType != m_settings.m_lraType ||
            settings.m_allowSleeping != m_settings.m_allowSleeping;

        const bool layerChanged = settings.m_collisionLayer != m_settings.m_collisionLayer ||
            settings.m_collisionGroupId != m_settings.m_collisionGroupId;

        m_settings = settings;

        if (!m_physicsSystem)
        {
            return;
        }

        if (needsRebuild)
        {
            // CreateBody reads m_objectLayer, so resolve it before rebuilding rather than
            // after, or the new body would land on the old layer.
            if (layerChanged)
            {
                RefreshObjectLayer();
            }
            DestroyBody();
            CreateBody();
        }
        else
        {
            if (layerChanged)
            {
                RefreshObjectLayer();
            }
            ApplyLiveSettings();
        }
    }

    JoltSoftBodySettings JoltSoftBody::GetSettings() const
    {
        AZStd::lock_guard lock(m_mutex);
        return m_settings;
    }

    void JoltSoftBody::SetPressure(float pressure)
    {
        AZStd::lock_guard lock(m_mutex);
        m_settings.m_pressure = pressure;
        ApplyLiveSettings();
    }

    void JoltSoftBody::SetLinearDamping(float damping)
    {
        AZStd::lock_guard lock(m_mutex);
        m_settings.m_linearDamping = AZ::GetMax(damping, 0.0f);
        ApplyLiveSettings();
    }

    void JoltSoftBody::SetGravityFactor(float factor)
    {
        AZStd::lock_guard lock(m_mutex);
        m_settings.m_gravityFactor = factor;
        ApplyLiveSettings();
    }

    AZ::Vector3 JoltSoftBody::GetLastWindImpulse() const
    {
        AZStd::lock_guard lock(m_mutex);
        return m_lastWindImpulse;
    }

    void JoltSoftBody::SetWindInfluence(float influence)
    {
        AZStd::lock_guard lock(m_mutex);
        // Not pushed anywhere: wind is this gem's own force, not a Jolt body property,
        // so the value is simply read the next time ApplyWind runs.
        m_settings.m_windInfluence = AZ::GetMax(influence, 0.0f);
    }

    void JoltSoftBody::ApplyWind(float deltaTime)
    {
        {
            AZStd::lock_guard lock(m_mutex);
            // Zeroed before any early-out: when the wind dies, the sail's pull on
            // whatever it is rigged to must die with it rather than hold its last value.
            m_lastWindImpulse = AZ::Vector3::CreateZero();

            if (!m_physicsSystem || m_bodyId.IsInvalid() || m_settings.m_windInfluence <= 0.0f)
            {
                return;
            }
        }

        auto* windRequests = AZ::Interface<Physics::WindRequests>::Get();
        if (windRequests == nullptr)
        {
            return;
        }

        // Sampled over the bounds rather than at the position: a sail spans space, and the
        // Aabb overload is how a local wind region that covers half of it still counts.
        const AZ::Aabb bounds = GetAabb();
        if (!bounds.IsValid())
        {
            return;
        }

        const AZ::Vector3 wind = windRequests->GetWind(bounds);
        if (wind.IsClose(AZ::Vector3::CreateZero()))
        {
            // Still air is free. The cost of skipping drag-in-still-air is that a cloth
            // waved through a windless level feels no air resistance beyond its own
            // damping, which is what the damping setting is for.
            return;
        }

        AZ_PROFILE_FUNCTION(Physics);

        AZStd::lock_guard lock(m_mutex);
        JPH::Vec3 totalImpulseLocal = JPH::Vec3::sZero();
        JPH::Quat bodyRotation = JPH::Quat::sIdentity();
        {
            JPH::BodyLockWrite bodyLock(m_physicsSystem->GetBodyLockInterface(), m_bodyId);
            if (!bodyLock.Succeeded() || !bodyLock.GetBody().IsSoftBody())
            {
                return;
            }

            JPH::Body& body = bodyLock.GetBody();
            auto* motionProperties = static_cast<JPH::SoftBodyMotionProperties*>(body.GetMotionProperties());
            auto& vertices = motionProperties->GetVertices();
            bodyRotation = body.GetRotation();

            // Particles live in the body's centre-of-mass frame, so the wind comes into
            // that frame once rather than every vertex going out to world.
            const JPH::Vec3 windLocal = bodyRotation.Conjugated() * Conversions::ToJolt(wind);

            // Pressure = 1/2 rho v^2 with sea-level air. The magnitude matters less than
            // the shape - quadratic in the *relative* normal speed, so a sail already
            // moving with the wind stops accelerating instead of running away.
            constexpr float airDensity = 1.2f;
            const float pressureScale = 0.5f * airDensity * m_settings.m_windInfluence;

            for (const JPH::SoftBodySharedSettings::Face& face : motionProperties->GetFaces())
            {
                JPH::SoftBodyVertex& v0 = vertices[face.mVertex[0]];
                JPH::SoftBodyVertex& v1 = vertices[face.mVertex[1]];
                JPH::SoftBodyVertex& v2 = vertices[face.mVertex[2]];

                // Length is twice the face area, direction is the face normal.
                const JPH::Vec3 areaNormal =
                    0.5f * (v1.mPosition - v0.mPosition).Cross(v2.mPosition - v0.mPosition);
                const float area = areaNormal.Length();
                if (area < 1.0e-8f)
                {
                    continue;
                }
                const JPH::Vec3 normal = areaNormal / area;

                const JPH::Vec3 relativeWind =
                    windLocal - (v0.mVelocity + v1.mVelocity + v2.mVelocity) / 3.0f;
                const float normalSpeed = relativeWind.Dot(normal);

                // Signed square keeps both sides of the cloth pressed leeward, whichever
                // way the triangle happens to wind.
                const JPH::Vec3 faceImpulse = normal *
                    (pressureScale * normalSpeed * AZ::GetAbs(normalSpeed) * area * deltaTime / 3.0f);
                if (faceImpulse.IsNaN())
                {
                    continue;
                }

                const float absNormalSpeed = AZ::GetAbs(normalSpeed);
                for (JPH::SoftBodyVertex* vertex : { &v0, &v1, &v2 })
                {
                    // Zero inverse mass is a pinned particle; the wind blows past it.
                    JPH::Vec3 velocityDelta = faceImpulse * vertex->mInvMass;

                    // Clamped so one step can at most cancel the relative wind, never
                    // push past it. Quadratic drag integrated explicitly is unstable
                    // without this: once anything - a weld dragging the cloth behind a
                    // fast boat - takes the relative speed high enough that k*v^2*dt
                    // exceeds 2v, every step amplifies the last, and the first sail
                    // rigged to a hull turned itself into a trillion-metre NaN in nine
                    // seconds. The clamp is also the physical truth: wind cannot blow a
                    // leaf faster than itself.
                    const float deltaLength = velocityDelta.Length();
                    if (deltaLength > absNormalSpeed && deltaLength > 0.0f)
                    {
                        velocityDelta *= absNormalSpeed / deltaLength;
                    }

                    vertex->mVelocity += velocityDelta;

                    // The impulse actually applied, not the one asked for - the rigged
                    // hull must receive what the canvas truly absorbed.
                    if (vertex->mInvMass > 0.0f)
                    {
                        totalImpulseLocal += velocityDelta / vertex->mInvMass;
                    }
                }
            }
        }

        m_lastWindImpulse = Conversions::FromJolt(bodyRotation * totalImpulseLocal);

        // Wind is a continuous force: a body it acts on must not sleep through it, or a
        // flag freezes mid-air the moment it settles for an instant. Outside the body
        // lock, since ActivateBody takes its own.
        m_physicsSystem->GetBodyInterface().ActivateBody(m_bodyId);
    }

    void JoltSoftBody::SetNumIterations(AZ::u32 iterations)
    {
        AZStd::lock_guard lock(m_mutex);
        m_settings.m_numIterations = AZ::GetMax(iterations, 1u);
        ApplyLiveSettings();
    }

    void JoltSoftBody::SetFriction(float friction)
    {
        AZStd::lock_guard lock(m_mutex);
        m_settings.m_friction = AZ::GetMax(friction, 0.0f);
        ApplyLiveSettings();
    }

    void JoltSoftBody::SetRestitution(float restitution)
    {
        AZStd::lock_guard lock(m_mutex);
        m_settings.m_restitution = AZ::GetMax(restitution, 0.0f);
        ApplyLiveSettings();
    }

    void JoltSoftBody::SetCollisionLayer(const AzPhysics::CollisionLayer& layer)
    {
        AZStd::lock_guard lock(m_mutex);
        m_settings.m_collisionLayer = layer;
        RefreshObjectLayer();
    }

    void JoltSoftBody::SetCollisionGroupId(const AzPhysics::CollisionGroups::Id& groupId)
    {
        AZStd::lock_guard lock(m_mutex);
        m_settings.m_collisionGroupId = groupId;
        RefreshObjectLayer();
    }

    JPH::ObjectLayer JoltSoftBody::GetObjectLayer() const
    {
        AZStd::lock_guard lock(m_mutex);
        return m_objectLayer;
    }

    void JoltSoftBody::RefreshObjectLayer()
    {
        m_objectLayer = AcquireObjectLayer(
                m_settings.m_collisionLayer, m_settings.m_collisionGroupId, /*isMoving*/ true,
                JoltBodyClass::SoftBody);

        if (m_physicsSystem && !m_bodyId.IsInvalid())
        {
            // Jolt can move a live body between object layers, so this needs no rebuild.
            // The broadphase is updated as part of the call.
            m_physicsSystem->GetBodyInterface().SetObjectLayer(m_bodyId, m_objectLayer);
        }
    }

    AZ::u32 JoltSoftBody::GetBuildGeneration() const
    {
        AZStd::lock_guard lock(m_mutex);
        return m_buildGeneration;
    }

    AZ::u32 JoltSoftBody::GetVertexCount() const
    {
        AZStd::lock_guard lock(m_mutex);
        if (!m_physicsSystem || m_bodyId.IsInvalid())
        {
            return 0;
        }

        // A real read lock, not the no-lock interface a step listener would use: this runs
        // on the main thread while the solver may be integrating these very particles.
        JPH::BodyLockRead bodyLock(m_physicsSystem->GetBodyLockInterface(), m_bodyId);
        if (!bodyLock.Succeeded())
        {
            return 0;
        }

        const JPH::Body& body = bodyLock.GetBody();
        if (!body.IsSoftBody())
        {
            return 0;
        }

        const auto* motionProperties = static_cast<const JPH::SoftBodyMotionProperties*>(body.GetMotionProperties());
        return static_cast<AZ::u32>(motionProperties->GetVertices().size());
    }

    AZ::Vector3 JoltSoftBody::GetVertexPosition(AZ::u32 index) const
    {
        AZStd::lock_guard lock(m_mutex);
        if (!m_physicsSystem || m_bodyId.IsInvalid())
        {
            return AZ::Vector3::CreateZero();
        }

        JPH::BodyLockRead bodyLock(m_physicsSystem->GetBodyLockInterface(), m_bodyId);
        if (!bodyLock.Succeeded())
        {
            return AZ::Vector3::CreateZero();
        }

        const JPH::Body& body = bodyLock.GetBody();
        if (!body.IsSoftBody())
        {
            return AZ::Vector3::CreateZero();
        }

        const auto* motionProperties = static_cast<const JPH::SoftBodyMotionProperties*>(body.GetMotionProperties());
        const auto& vertices = motionProperties->GetVertices();
        if (index >= vertices.size())
        {
            return AZ::Vector3::CreateZero();
        }

        // Particle positions are relative to the body's centre of mass, not the world.
        const JPH::RMat44 centerOfMassTransform = body.GetCenterOfMassTransform();
        return FromJolt(JPH::Vec3(centerOfMassTransform * vertices[index].mPosition));
    }

    bool JoltSoftBody::SetVertexPinned(AZ::u32 index, bool pinned)
    {
        AZStd::lock_guard lock(m_mutex);
        if (!m_physicsSystem || m_bodyId.IsInvalid())
        {
            return false;
        }

        {
            JPH::BodyLockWrite bodyLock(m_physicsSystem->GetBodyLockInterface(), m_bodyId);
            if (!bodyLock.Succeeded() || !bodyLock.GetBody().IsSoftBody())
            {
                return false;
            }

            auto* motionProperties = static_cast<JPH::SoftBodyMotionProperties*>(bodyLock.GetBody().GetMotionProperties());
            auto& vertices = motionProperties->GetVertices();
            if (index >= vertices.size())
            {
                return false;
            }

            if (pinned)
            {
                // Zero inverse mass is the pin; the velocity goes too, or the solver would
                // report a moving particle that never moves.
                vertices[index].mInvMass = 0.0f;
                vertices[index].mVelocity = JPH::Vec3::sZero();
            }
            else
            {
                // The same share of the body's mass every other free particle carries.
                vertices[index].mInvMass = m_perVertexInvMass;
            }
        }

        // Outside the body lock (ActivateBody takes its own write lock on this body): a
        // change to a sleeping body would otherwise sit invisible until something woke it.
        m_physicsSystem->GetBodyInterface().ActivateBody(m_bodyId);
        return true;
    }

    bool JoltSoftBody::IsVertexPinned(AZ::u32 index) const
    {
        AZStd::lock_guard lock(m_mutex);
        if (!m_physicsSystem || m_bodyId.IsInvalid())
        {
            return false;
        }

        JPH::BodyLockRead bodyLock(m_physicsSystem->GetBodyLockInterface(), m_bodyId);
        if (!bodyLock.Succeeded() || !bodyLock.GetBody().IsSoftBody())
        {
            return false;
        }

        const auto* motionProperties =
            static_cast<const JPH::SoftBodyMotionProperties*>(bodyLock.GetBody().GetMotionProperties());
        const auto& vertices = motionProperties->GetVertices();
        return index < vertices.size() && vertices[index].mInvMass == 0.0f;
    }

    bool JoltSoftBody::SetVertexVelocity(AZ::u32 index, const AZ::Vector3& velocity)
    {
        AZStd::lock_guard lock(m_mutex);
        if (!m_physicsSystem || m_bodyId.IsInvalid())
        {
            return false;
        }

        {
            JPH::BodyLockWrite bodyLock(m_physicsSystem->GetBodyLockInterface(), m_bodyId);
            if (!bodyLock.Succeeded() || !bodyLock.GetBody().IsSoftBody())
            {
                return false;
            }

            JPH::Body& body = bodyLock.GetBody();
            auto* motionProperties = static_cast<JPH::SoftBodyMotionProperties*>(body.GetMotionProperties());
            auto& vertices = motionProperties->GetVertices();
            if (index >= vertices.size() || vertices[index].mInvMass == 0.0f)
            {
                return false;
            }

            // Particle velocities are stored in the body's centre-of-mass frame; only the
            // rotation matters for a velocity, but a soft body's rotation is usually
            // baked to identity anyway.
            const JPH::Mat44 worldToLocal = body.GetInverseCenterOfMassTransform();
            vertices[index].mVelocity = worldToLocal.Multiply3x3(Conversions::ToJolt(velocity));
        }

        m_physicsSystem->GetBodyInterface().ActivateBody(m_bodyId);
        return true;
    }

    AZ::Vector3 JoltSoftBody::GetVertexVelocity(AZ::u32 index) const
    {
        AZStd::lock_guard lock(m_mutex);
        if (!m_physicsSystem || m_bodyId.IsInvalid())
        {
            return AZ::Vector3::CreateZero();
        }

        JPH::BodyLockRead bodyLock(m_physicsSystem->GetBodyLockInterface(), m_bodyId);
        if (!bodyLock.Succeeded() || !bodyLock.GetBody().IsSoftBody())
        {
            return AZ::Vector3::CreateZero();
        }

        const JPH::Body& body = bodyLock.GetBody();
        const auto* motionProperties = static_cast<const JPH::SoftBodyMotionProperties*>(body.GetMotionProperties());
        const auto& vertices = motionProperties->GetVertices();
        if (index >= vertices.size())
        {
            return AZ::Vector3::CreateZero();
        }

        return FromJolt(body.GetCenterOfMassTransform().Multiply3x3(vertices[index].mVelocity));
    }

    void JoltSoftBody::SetCustomGeometry(
        const AZStd::vector<AZ::Vector3>& vertices, const AZStd::vector<AZ::u32>& indices)
    {
        AZStd::lock_guard lock(m_mutex);
        m_customVertices = vertices;
        m_customIndices = indices;

        // Handing over geometry is what selects the shape - a caller with a mesh should not
        // also have to remember to change a drop-down for it to be used, and a Custom body
        // whose geometry was cleared has nothing to fall back to anyway.
        m_settings.m_shape = JoltSoftBodyShape::Custom;

        // The particle layout is the geometry, so new geometry means a new body - and
        // usually there is no body yet to replace. A Custom body has nothing to build from
        // until this call, so unlike every other rebuild here, this one has to run when the
        // body is missing rather than only when it exists.
        if (m_physicsSystem)
        {
            DestroyBody();
            CreateBody();
        }
    }

    bool JoltSoftBody::HasCustomGeometry() const
    {
        AZStd::lock_guard lock(m_mutex);
        return !m_customVertices.empty() && !m_customIndices.empty();
    }

    void JoltSoftBody::SetSkinningData(
        const AZStd::vector<AZ::Transform>& jointInvBinds,
        const AZStd::vector<JoltSoftBodySkinnedVertex>& skinnedVertices)
    {
        AZStd::lock_guard lock(m_mutex);
        m_jointInvBinds = jointInvBinds;
        m_skinnedVertices = skinnedVertices;

        // The constraints live in the shared settings, so new data means a new body.
        if (m_physicsSystem && !m_bodyId.IsInvalid())
        {
            DestroyBody();
            CreateBody();
        }
    }

    bool JoltSoftBody::HasSkinningData() const
    {
        AZStd::lock_guard lock(m_mutex);
        return !m_skinnedVertices.empty() && !m_jointInvBinds.empty();
    }

    void JoltSoftBody::ApplySkinningData(JPH::SoftBodySharedSettings& settings) const
    {
        if (m_skinnedVertices.empty() || m_jointInvBinds.empty())
        {
            return;
        }

        settings.mInvBindMatrices.clear();
        settings.mInvBindMatrices.reserve(m_jointInvBinds.size());
        for (AZ::u32 jointIndex = 0; jointIndex < m_jointInvBinds.size(); ++jointIndex)
        {
            const AZ::Transform& invBind = m_jointInvBinds[jointIndex];
            settings.mInvBindMatrices.push_back(JPH::SoftBodySharedSettings::InvBind(
                jointIndex,
                JPH::Mat44::sRotationTranslation(
                    Conversions::ToJolt(invBind.GetRotation()), Conversions::ToJolt(invBind.GetTranslation()))));
        }

        const size_t vertexCount = settings.mVertices.size();
        const size_t jointCount = m_jointInvBinds.size();
        for (const JoltSoftBodySkinnedVertex& skinnedVertex : m_skinnedVertices)
        {
            if (skinnedVertex.m_vertexIndex >= vertexCount)
            {
                AZ_Warning("JoltPhysics", false,
                    "Skinned vertex index %u is out of range for %zu particles and is skipped.",
                    skinnedVertex.m_vertexIndex, vertexCount);
                continue;
            }

            JPH::SoftBodySharedSettings::Skinned skinned;
            skinned.mVertex = skinnedVertex.m_vertexIndex;
            skinned.mMaxDistance = AZ::GetMax(skinnedVertex.m_maxDistance, 0.0f);
            // Jolt disables the backstop when its distance reaches the max distance, so
            // the default (max float) leaves it off without needing a flag of its own.
            skinned.mBackStopDistance = AZ::GetMax(skinnedVertex.m_backstopDistance, 0.0f);
            skinned.mBackStopRadius = AZ::GetMax(skinnedVertex.m_backstopRadius, 0.0f);

            AZ::u32 weightCount = 0;
            for (const JoltSoftBodySkinInfluence& influence : skinnedVertex.m_influences)
            {
                if (weightCount >= JPH::SoftBodySharedSettings::Skinned::cMaxSkinWeights)
                {
                    break;
                }
                if (influence.m_jointIndex >= jointCount || influence.m_weight <= 0.0f)
                {
                    continue;
                }
                skinned.mWeights[weightCount++] =
                    JPH::SoftBodySharedSettings::SkinWeight(influence.m_jointIndex, influence.m_weight);
            }
            if (weightCount == 0)
            {
                continue;
            }

            skinned.NormalizeWeights();
            settings.mSkinnedConstraints.push_back(skinned);
        }

        settings.CalculateSkinnedConstraintNormals();
    }

    bool JoltSoftBody::UpdateSkinnedJoints(const AZStd::vector<AZ::Transform>& jointTransforms, bool hardSkinAll)
    {
        AZStd::lock_guard lock(m_mutex);
        if (!m_physicsSystem || m_bodyId.IsInvalid() || m_skinnedVertices.empty())
        {
            return false;
        }
        if (jointTransforms.size() < m_jointInvBinds.size())
        {
            AZ_Warning("JoltPhysics", false,
                "UpdateSkinnedJoints received %zu joint transforms for %zu joints; skinning is skipped.",
                jointTransforms.size(), m_jointInvBinds.size());
            return false;
        }

        AZStd::vector<JPH::Mat44> jointMatrices;
        jointMatrices.reserve(jointTransforms.size());
        for (const AZ::Transform& jointTransform : jointTransforms)
        {
            jointMatrices.push_back(JPH::Mat44::sRotationTranslation(
                Conversions::ToJolt(jointTransform.GetRotation()), Conversions::ToJolt(jointTransform.GetTranslation())));
        }

        {
            JPH::BodyLockWrite bodyLock(m_physicsSystem->GetBodyLockInterface(), m_bodyId);
            if (!bodyLock.Succeeded() || !bodyLock.GetBody().IsSoftBody())
            {
                return false;
            }

            JPH::Body& body = bodyLock.GetBody();
            auto* motionProperties = static_cast<JPH::SoftBodyMotionProperties*>(body.GetMotionProperties());

            // Malloc-backed and stateless, so it works with or without a scene;
            // SkinVertices only needs scratch space for the duration of the call.
            static JPH::TempAllocatorMalloc tempAllocator;
            motionProperties->SkinVertices(
                body.GetCenterOfMassTransform(), jointMatrices.data(),
                static_cast<JPH::uint>(jointMatrices.size()), hardSkinAll, tempAllocator);
        }

        m_physicsSystem->GetBodyInterface().ActivateBody(m_bodyId);
        return true;
    }

    AZ::Transform JoltSoftBody::GetSkinningFrame() const
    {
        AZStd::lock_guard lock(m_mutex);
        if (!m_physicsSystem || m_bodyId.IsInvalid())
        {
            return AZ::Transform::CreateIdentity();
        }

        JPH::BodyLockRead bodyLock(m_physicsSystem->GetBodyLockInterface(), m_bodyId);
        if (!bodyLock.Succeeded())
        {
            return AZ::Transform::CreateIdentity();
        }

        const JPH::RMat44 centerOfMass = bodyLock.GetBody().GetCenterOfMassTransform();
        return AZ::Transform::CreateFromQuaternionAndTranslation(
            Conversions::FromJolt(centerOfMass.GetQuaternion()),
            Conversions::FromJolt(centerOfMass.GetTranslation()));
    }

    void JoltSoftBody::SetSkinConstraintsEnabled(bool enabled)
    {
        AZStd::lock_guard lock(m_mutex);
        m_skinConstraintsEnabled = enabled;
        ApplyLiveSettings();
    }

    void JoltSoftBody::SetCollisionGroup(const JPH::CollisionGroup& group)
    {
        AZStd::lock_guard lock(m_mutex);
        m_collisionGroup = group;
        if (!m_physicsSystem || m_bodyId.IsInvalid())
        {
            return;
        }

        JPH::BodyLockWrite bodyLock(m_physicsSystem->GetBodyLockInterface(), m_bodyId);
        if (bodyLock.Succeeded())
        {
            bodyLock.GetBody().SetCollisionGroup(group);
        }
    }

    JPH::CollisionGroup JoltSoftBody::GetCollisionGroup() const
    {
        AZStd::lock_guard lock(m_mutex);
        return m_collisionGroup;
    }

    bool JoltSoftBody::CopyVertexPositions(AZStd::vector<AZ::Vector3>& outPositions) const
    {
        outPositions.clear();

        AZStd::lock_guard lock(m_mutex);
        if (!m_physicsSystem || m_bodyId.IsInvalid())
        {
            return false;
        }

        JPH::BodyLockRead bodyLock(m_physicsSystem->GetBodyLockInterface(), m_bodyId);
        if (!bodyLock.Succeeded())
        {
            return false;
        }

        const JPH::Body& body = bodyLock.GetBody();
        if (!body.IsSoftBody())
        {
            return false;
        }

        const auto* motionProperties = static_cast<const JPH::SoftBodyMotionProperties*>(body.GetMotionProperties());
        const auto& vertices = motionProperties->GetVertices();
        const JPH::RMat44 centerOfMassTransform = body.GetCenterOfMassTransform();

        outPositions.reserve(vertices.size());
        for (const JPH::SoftBodyVertex& vertex : vertices)
        {
            outPositions.push_back(FromJolt(JPH::Vec3(centerOfMassTransform * vertex.mPosition)));
        }
        return true;
    }

    AZ::Aabb JoltSoftBody::GetWorldBounds() const
    {
        AZStd::vector<AZ::Vector3> positions;
        if (!CopyVertexPositions(positions))
        {
            return AZ::Aabb::CreateNull();
        }

        AZ::Aabb bounds = AZ::Aabb::CreateNull();
        for (const AZ::Vector3& position : positions)
        {
            bounds.AddPoint(position);
        }
        return bounds;
    }

    JPH::Ref<JPH::SoftBodySharedSettings> JoltSoftBody::BuildSharedSettings(
        AZStd::vector<AZ::u32>& outTriangleIndices, float& outPerVertexInvMass) const
    {
        outTriangleIndices.clear();
        outPerVertexInvMass = 1.0f;

        // Counted across every branch and reported once at the end, so a Cloth built at a
        // degenerate size is as visible as a mesh full of slivers.
        size_t droppedFaces = 0;
        size_t totalFaces = 0;

        const AZ::u32 resolution = AZ::GetMax(m_settings.m_resolution, 2u);

        if (m_settings.m_shape == JoltSoftBodyShape::Cube)
        {
            // sCreateCube returns pre-optimised settings, and skinned constraints must be
            // added before Optimize - so the Cube shape cannot be skinned.
            AZ_Warning("JoltPhysics", m_skinnedVertices.empty(),
                "Soft body skinning is not supported for the Cube shape and is ignored.");

            // Jolt's own helper: a solid grid with edge constraints, volume constraints and
            // faces, already finalised and optimised. Its single spacing argument is why a
            // Cube uses only the X extent.
            const float spacing = AZ::GetMax(m_settings.m_size.GetX(), 0.01f) / static_cast<float>(resolution - 1);
            JPH::Ref<JPH::SoftBodySharedSettings> settings = JPH::SoftBodySharedSettings::sCreateCube(resolution, spacing);

            // sCreateCube centres nothing, so shift the grid onto the entity origin.
            const float halfSize = 0.5f * spacing * static_cast<float>(resolution - 1);
            for (JPH::SoftBodySharedSettings::Vertex& vertex : settings->mVertices)
            {
                vertex.mPosition = JPH::Float3(
                    vertex.mPosition.x - halfSize, vertex.mPosition.y - halfSize, vertex.mPosition.z - halfSize);
            }

            outPerVertexInvMass = DistributeMass(*settings, m_settings.m_mass);
            CollectFaces(*settings, outTriangleIndices);
            // sCreateCube is Jolt's own generator and does not make slivers, but a Cube of
            // zero size would, and the check costs one cross product per face.
            totalFaces = settings->mFaces.size();
            droppedFaces = DropDegenerateFaces(*settings, outTriangleIndices);
            ReportDegenerateFaces(droppedFaces, totalFaces, PinOrphanParticles(*settings));
            return settings->mFaces.empty() ? nullptr : settings;
        }

        JPH::Ref<JPH::SoftBodySharedSettings> settings = new JPH::SoftBodySharedSettings();

        if (m_settings.m_shape == JoltSoftBodyShape::Mesh || m_settings.m_shape == JoltSoftBodyShape::Custom)
        {
            // Both shapes are a triangle soup someone else produced; only where it comes
            // from differs. Neither honours m_size or the pinning presets - a supplied
            // surface has no canonical corners or top edge, so pin its particles at
            // runtime instead.
            AZStd::vector<AZ::Vector3> rawVertices;
            AZStd::vector<AZ::u32> rawIndices;

            if (m_settings.m_shape == JoltSoftBodyShape::Mesh)
            {
                const Physics::CookedMeshShapeConfiguration* meshConfiguration =
                    FindTriangleMeshConfiguration(m_settings.m_meshAsset.Get());
                if (!meshConfiguration)
                {
                    AZ_Warning("JoltPhysics", false,
                        "Soft body mesh asset is missing, not loaded, or carries no triangle mesh; no body is built.");
                    return nullptr;
                }

                if (!JoltMeshUtils::UnpackTriangleMesh(meshConfiguration->GetCookedMeshData(), rawVertices, rawIndices))
                {
                    AZ_Warning("JoltPhysics", false,
                        "Soft body mesh asset holds a malformed cooked blob; no body is built.");
                    return nullptr;
                }
            }
            else
            {
                if (m_customVertices.empty() || m_customIndices.empty())
                {
                    // Not a warning: a Custom body exists before the thing that feeds it
                    // has anything to feed, and that gap is a frame or two on every load.
                    return nullptr;
                }
                rawVertices = m_customVertices;
                rawIndices = m_customIndices;
            }

            // Weld before building: the seams a render mesh splits on must simulate as
            // single particles or the sheet tears along every one of them. Geometry whose
            // positions are already unique passes through unchanged, in order, which is
            // what lets a caller that welded the mesh itself use its own vertex indices as
            // particle indices.
            AZStd::vector<AZ::Vector3> weldedVertices;
            AZStd::vector<AZ::u32> weldedIndices;
            WeldVertices(rawVertices, rawIndices, weldedVertices, weldedIndices);
            if (weldedVertices.empty() || weldedIndices.empty())
            {
                return nullptr;
            }

            for (const AZ::Vector3& position : weldedVertices)
            {
                JPH::SoftBodySharedSettings::Vertex vertex;
                vertex.mPosition = JPH::Float3(position.GetX(), position.GetY(), position.GetZ());
                vertex.mInvMass = 1.0f;
                settings->mVertices.push_back(vertex);
            }
            totalFaces = weldedIndices.size() / 3;
            for (size_t i = 0; i + 2 < weldedIndices.size(); i += 3)
            {
                if (!AddTriangle(
                        *settings, outTriangleIndices, weldedIndices[i], weldedIndices[i + 1], weldedIndices[i + 2]))
                {
                    ++droppedFaces;
                }
            }

            if (settings->mFaces.empty())
            {
                ReportDegenerateFaces(droppedFaces, totalFaces, 0);
                return nullptr;
            }
        }
        else if (m_settings.m_shape == JoltSoftBodyShape::Cloth)
        {
            const float sizeX = AZ::GetMax(m_settings.m_size.GetX(), 0.01f);
            const float sizeY = AZ::GetMax(m_settings.m_size.GetY(), 0.01f);
            const float step = 1.0f / static_cast<float>(resolution - 1);

            for (AZ::u32 y = 0; y < resolution; ++y)
            {
                for (AZ::u32 x = 0; x < resolution; ++x)
                {
                    const float fx = (static_cast<float>(x) * step - 0.5f) * sizeX;
                    const float fy = (static_cast<float>(y) * step - 0.5f) * sizeY;

                    const bool isEdgeX = x == 0 || x == resolution - 1;
                    const bool isTopRow = y == resolution - 1;
                    bool pinned = false;
                    switch (m_settings.m_pinning)
                    {
                    case JoltSoftBodyPinning::Corners:
                        pinned = isEdgeX && (y == 0 || isTopRow);
                        break;
                    case JoltSoftBodyPinning::TopEdge:
                        pinned = isTopRow;
                        break;
                    case JoltSoftBodyPinning::None:
                        break;
                    }

                    JPH::SoftBodySharedSettings::Vertex vertex;
                    vertex.mPosition = JPH::Float3(fx, fy, 0.0f);
                    // Zero inverse mass is how Jolt pins a particle: infinite mass, so no
                    // force moves it. DistributeMass leaves these alone.
                    vertex.mInvMass = pinned ? 0.0f : 1.0f;
                    settings->mVertices.push_back(vertex);
                }
            }

            const auto index = [resolution](AZ::u32 x, AZ::u32 y)
            {
                return y * resolution + x;
            };
            for (AZ::u32 y = 0; y + 1 < resolution; ++y)
            {
                for (AZ::u32 x = 0; x + 1 < resolution; ++x)
                {
                    totalFaces += 2;
                    droppedFaces += AddQuad(
                        *settings, outTriangleIndices, index(x, y), index(x + 1, y), index(x + 1, y + 1), index(x, y + 1));
                }
            }
        }
        else // Balloon: a closed UV sphere, so internal pressure has a volume to act on.
        {
            const float radius = AZ::GetMax(m_settings.m_size.GetX(), 0.01f) * 0.5f;
            const AZ::u32 rings = AZ::GetMax(resolution, 3u); // latitude bands
            const AZ::u32 segments = AZ::GetMax(resolution * 2u, 4u); // longitude divisions

            const auto addVertex = [&settings](const AZ::Vector3& position)
            {
                JPH::SoftBodySharedSettings::Vertex vertex;
                vertex.mPosition = JPH::Float3(position.GetX(), position.GetY(), position.GetZ());
                vertex.mInvMass = 1.0f;
                settings->mVertices.push_back(vertex);
            };

            const AZ::u32 northPole = 0;
            addVertex(AZ::Vector3(0.0f, 0.0f, radius));

            // Interior rings only: the poles are single vertices, so a ring at theta = 0 or
            // pi would duplicate them and produce degenerate faces.
            const AZ::u32 firstRingVertex = 1;
            for (AZ::u32 ring = 1; ring < rings; ++ring)
            {
                const float theta = AZ::Constants::Pi * static_cast<float>(ring) / static_cast<float>(rings);
                const float sinTheta = std::sin(theta);
                const float cosTheta = std::cos(theta);
                for (AZ::u32 segment = 0; segment < segments; ++segment)
                {
                    const float phi = AZ::Constants::TwoPi * static_cast<float>(segment) / static_cast<float>(segments);
                    addVertex(AZ::Vector3(radius * sinTheta * std::cos(phi), radius * sinTheta * std::sin(phi), radius * cosTheta));
                }
            }

            const AZ::u32 southPole = static_cast<AZ::u32>(settings->mVertices.size());
            addVertex(AZ::Vector3(0.0f, 0.0f, -radius));

            const auto ringVertex = [firstRingVertex, segments](AZ::u32 ring, AZ::u32 segment)
            {
                return firstRingVertex + (ring - 1) * segments + (segment % segments);
            };

            // Wound so the face normals point outwards. This is not cosmetic: Jolt derives
            // the enclosed volume from the face winding and skips pressure entirely when
            // that volume comes out negative, so an inward-wound sphere does not collapse -
            // it simply never inflates, with no warning.
            for (AZ::u32 segment = 0; segment < segments; ++segment)
            {
                ++totalFaces;
                droppedFaces += AddTriangle(
                    *settings, outTriangleIndices, northPole, ringVertex(1, segment), ringVertex(1, segment + 1)) ? 0 : 1;
            }

            for (AZ::u32 ring = 1; ring + 1 < rings; ++ring)
            {
                for (AZ::u32 segment = 0; segment < segments; ++segment)
                {
                    totalFaces += 2;
                    droppedFaces += AddQuad(
                        *settings, outTriangleIndices, ringVertex(ring, segment), ringVertex(ring + 1, segment),
                        ringVertex(ring + 1, segment + 1), ringVertex(ring, segment + 1));
                }
            }

            for (AZ::u32 segment = 0; segment < segments; ++segment)
            {
                ++totalFaces;
                droppedFaces += AddTriangle(
                    *settings, outTriangleIndices, southPole, ringVertex(rings - 1, segment + 1),
                    ringVertex(rings - 1, segment)) ? 0 : 1;
            }
        }

        // Constraints are generated from the faces: edges hold the sheet together, shear
        // edges stop quads collapsing diagonally, bend constraints resist folding.
        JPH::SoftBodySharedSettings::VertexAttributes attributes;
        attributes.mCompliance = m_settings.m_compliance;
        attributes.mShearCompliance = m_settings.m_compliance;
        attributes.mBendCompliance = m_settings.m_compliance;
        switch (m_settings.m_lraType)
        {
        case JoltSoftBodyLraType::EuclideanDistance:
            attributes.mLRAType = JPH::SoftBodySharedSettings::ELRAType::EuclideanDistance;
            break;
        case JoltSoftBodyLraType::GeodesicDistance:
            attributes.mLRAType = JPH::SoftBodySharedSettings::ELRAType::GeodesicDistance;
            break;
        case JoltSoftBodyLraType::None:
            break;
        }
        // Before CreateConstraints, which walks the faces: a particle pinned here takes no
        // share of the mass below, which is what stops an orphan skewing the distribution.
        const size_t orphans = PinOrphanParticles(*settings);
        ReportDegenerateFaces(droppedFaces, totalFaces, orphans);

        settings->CreateConstraints(&attributes, 1, JPH::SoftBodySharedSettings::EBendType::Distance);

        // After the faces and constraints exist (the skinned-normal calculation reads
        // the faces) and before Optimize, which remaps the constraint order.
        ApplySkinningData(*settings);

        outPerVertexInvMass = DistributeMass(*settings, m_settings.m_mass);
        settings->Optimize();
        return settings;
    }

    void JoltSoftBody::ReportDegenerateFaces(size_t dropped, size_t total, size_t orphans) const
    {
        if (dropped == 0 && orphans == 0)
        {
            return;
        }

        // Named, because a level with fifty banners in it makes an anonymous warning close
        // to useless - and because the author's next question is always "which one".
        const AZStd::string where = Internal::NameClause(m_entityId);
        AZ_Warning("JoltPhysics", dropped == 0,
            "Soft body%s: dropped %zu of %zu faces, which have no area for Jolt to collide against. Slivers and "
            "collapsed triangles are common in art meshes; Jolt cannot simulate them, so the surface has holes "
            "where they were.",
            where.c_str(), dropped, total);
        AZ_Warning("JoltPhysics", orphans == 0,
            "Soft body%s: %zu particles belong to no surviving face. Jolt builds its constraints from faces, so "
            "these have nothing holding them and would fall away on their own; they are pinned where they are "
            "instead.",
            where.c_str(), orphans);
    }

    bool JoltSoftBody::CreateBody()
    {
        if (!m_physicsSystem)
        {
            return false;
        }

        AZStd::vector<AZ::u32> triangleIndices;
        float perVertexInvMass = 1.0f;
        JPH::Ref<JPH::SoftBodySharedSettings> sharedSettings = BuildSharedSettings(triangleIndices, perVertexInvMass);
        if (!sharedSettings || sharedSettings->mVertices.empty())
        {
            return false;
        }

        // The body's rotation is folded into the shared settings' own vertices here rather
        // than handed to Jolt, and the body is created upright.
        //
        // Jolt offers to do this (mMakeRotationIdentity, on by default) but bakes only the
        // *simulated* particles, leaving the shared settings unrotated - and the skinned
        // constraints read their rest pose straight out of the shared settings. So a
        // rotated, skinned body has its particles in one space and its skinning targets
        // computed in another, and every hard-skinned particle is dragged to where it
        // would have been unrotated. A sail hung from a yard collapsed onto its back
        // within a frame of being rigged, edge-on to the wind and catching nothing.
        //
        // Doing it here keeps the two in step, and keeps the body's frame identity-rotated
        // and therefore stable across the rebuild that installing skinning data causes -
        // which matters, because the inverse binds are computed against that frame just
        // before it.
        //
        // Safe after constraint generation: constraints store rest *lengths*, which a
        // rotation does not change.
        const AZ::Quaternion bodyRotation = m_worldTransform.GetRotation();
        if (!bodyRotation.IsIdentity())
        {
            for (JPH::SoftBodySharedSettings::Vertex& vertex : sharedSettings->mVertices)
            {
                const AZ::Vector3 rotated = bodyRotation.TransformVector(
                    AZ::Vector3(vertex.mPosition.x, vertex.mPosition.y, vertex.mPosition.z));
                vertex.mPosition = JPH::Float3(rotated.GetX(), rotated.GetY(), rotated.GetZ());
            }
        }

        JPH::SoftBodyCreationSettings creationSettings(
            sharedSettings, ToJoltR(m_worldTransform.GetTranslation()), JPH::Quat::sIdentity(), m_objectLayer);
        creationSettings.mNumIterations = AZ::GetMax(m_settings.m_numIterations, 1u);
        creationSettings.mLinearDamping = AZ::GetMax(m_settings.m_linearDamping, 0.0f);
        creationSettings.mPressure = m_settings.m_pressure;
        creationSettings.mGravityFactor = m_settings.m_gravityFactor;
        creationSettings.mFriction = AZ::GetMax(m_settings.m_friction, 0.0f);
        creationSettings.mRestitution = AZ::GetMax(m_settings.m_restitution, 0.0f);
        creationSettings.mVertexRadius = AZ::GetMax(m_settings.m_vertexRadius, 0.0f);
        creationSettings.mMaxLinearVelocity = AZ::GetMax(m_settings.m_maxLinearVelocity, 0.0f);
        // A skinned body must not chase its own frame. Jolt does its skinning maths in the
        // body's centre-of-mass frame, and mUpdatePosition moves that frame to follow the
        // particles every step - so the frame the skinned targets were computed against
        // drifts out from under them, which drags the cloth further, which moves the frame
        // again. A sail rigged to a boat collapsed into a flat sheet within a second of
        // this, and the wind then had nothing to push on.
        creationSettings.mUpdatePosition = m_settings.m_updatePosition && m_skinnedVertices.empty();
        creationSettings.mFacesDoubleSided = m_settings.m_doubleSidedFaces;
        creationSettings.mAllowSleeping = m_settings.m_allowSleeping;
        creationSettings.mCollisionGroup = m_collisionGroup;
        // The entity id, the same convention as the rigid bodies: it is how contact
        // listeners and per-particle notifications resolve a body back to its entity.
        creationSettings.mUserData = static_cast<AZ::u64>(m_entityId);

        // Creating and adding a body is only legal outside the physics step. This is called
        // from component activation and from settings changes, both on the main thread.
        m_bodyId = m_physicsSystem->GetBodyInterface().CreateAndAddSoftBody(creationSettings, JPH::EActivation::Activate);
        if (m_bodyId.IsInvalid())
        {
            return false;
        }

        m_triangleIndices = AZStd::move(triangleIndices);
        m_perVertexInvMass = perVertexInvMass;
        ++m_buildGeneration;

        // Re-applies the live values a rebuild cannot carry through the creation
        // settings - today that is only the skin-constraints toggle.
        ApplyLiveSettings();
        return true;
    }

    void JoltSoftBody::DestroyBody()
    {
        if (!m_physicsSystem || m_bodyId.IsInvalid())
        {
            return;
        }

        JPH::BodyInterface& bodyInterface = m_physicsSystem->GetBodyInterface();
        if (bodyInterface.IsAdded(m_bodyId))
        {
            bodyInterface.RemoveBody(m_bodyId);
        }
        bodyInterface.DestroyBody(m_bodyId);

        m_bodyId = JPH::BodyID();
        m_triangleIndices.clear();
    }

    void JoltSoftBody::ApplyLiveSettings()
    {
        if (!m_physicsSystem || m_bodyId.IsInvalid())
        {
            return;
        }

        bool needsWaking = false;
        {
            JPH::BodyLockWrite bodyLock(m_physicsSystem->GetBodyLockInterface(), m_bodyId);
            if (!bodyLock.Succeeded())
            {
                return;
            }

            JPH::Body& body = bodyLock.GetBody();
            if (!body.IsSoftBody())
            {
                return;
            }

            auto* motionProperties = static_cast<JPH::SoftBodyMotionProperties*>(body.GetMotionProperties());
            motionProperties->SetNumIterations(AZ::GetMax(m_settings.m_numIterations, 1u));
            motionProperties->SetPressure(m_settings.m_pressure);
            motionProperties->SetLinearDamping(AZ::GetMax(m_settings.m_linearDamping, 0.0f));
            motionProperties->SetGravityFactor(m_settings.m_gravityFactor);
            motionProperties->SetVertexRadius(AZ::GetMax(m_settings.m_vertexRadius, 0.0f));
            motionProperties->SetMaxLinearVelocity(AZ::GetMax(m_settings.m_maxLinearVelocity, 0.0f));
            motionProperties->SetUpdatePosition(m_settings.m_updatePosition);
            motionProperties->SetFacesDoubleSided(m_settings.m_doubleSidedFaces);
            motionProperties->SetEnableSkinConstraints(m_skinConstraintsEnabled);

            // Friction and restitution live on the body, where a rigid body's material
            // would put them.
            body.SetFriction(AZ::GetMax(m_settings.m_friction, 0.0f));
            body.SetRestitution(AZ::GetMax(m_settings.m_restitution, 0.0f));

            needsWaking = !body.IsActive();
        }

        // Outside the lock: BodyInterface::ActivateBody takes a write lock on this same
        // body, so calling it above would deadlock against the lock we were holding.
        // A sleeping body ignores new settings until something wakes it, which otherwise
        // reads as the setter having done nothing.
        if (needsWaking)
        {
            m_physicsSystem->GetBodyInterface().ActivateBody(m_bodyId);
        }
    }
} // namespace JoltPhysics
