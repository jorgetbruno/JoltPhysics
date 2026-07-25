#include <SoftBody/JoltSoftBody.h>

#include <AzCore/Math/MathUtils.h>
#include <AzCore/std/parallel/lock.h>

#include <cmath>

#include <AzFramework/Physics/PhysicsSystem.h>

#include <Scene/JoltScene.h>
#include <System/CollisionLayerFilters.h>
#include <Utils/Conversions.h>

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
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

        //! Two triangles per grid quad, wound counter-clockwise seen from +Z so the cloth's
        //! front face points the same way the entity does.
        void AddQuad(
            JPH::SoftBodySharedSettings& settings,
            AZStd::vector<AZ::u32>& indices,
            AZ::u32 a,
            AZ::u32 b,
            AZ::u32 c,
            AZ::u32 d)
        {
            settings.AddFace(JPH::SoftBodySharedSettings::Face(a, b, c));
            settings.AddFace(JPH::SoftBodySharedSettings::Face(a, c, d));
            indices.insert(indices.end(), { a, b, c, a, c, d });
        }

        void AddTriangle(
            JPH::SoftBodySharedSettings& settings, AZStd::vector<AZ::u32>& indices, AZ::u32 a, AZ::u32 b, AZ::u32 c)
        {
            settings.AddFace(JPH::SoftBodySharedSettings::Face(a, b, c));
            indices.insert(indices.end(), { a, b, c });
        }

        //! Fills the triangle list from faces Jolt generated itself, for the shapes built by
        //! a Jolt helper rather than here.
        void CollectFaces(const JPH::SoftBodySharedSettings& settings, AZStd::vector<AZ::u32>& indices)
        {
            indices.clear();
            indices.reserve(settings.mFaces.size() * 3);
            for (const JPH::SoftBodySharedSettings::Face& face : settings.mFaces)
            {
                indices.insert(indices.end(), { face.mVertex[0], face.mVertex[1], face.mVertex[2] });
            }
        }

        //! Spreads the requested total mass over the particles. Jolt works in inverse mass,
        //! and a zero inverse mass is what pins a particle, so pinned ones are left alone.
        void DistributeMass(JPH::SoftBodySharedSettings& settings, float totalMass)
        {
            const size_t vertexCount = settings.mVertices.size();
            if (vertexCount == 0)
            {
                return;
            }

            const float perVertexMass = AZ::GetMax(totalMass, 0.001f) / static_cast<float>(vertexCount);
            const float inverseMass = 1.0f / perVertexMass;
            for (JPH::SoftBodySharedSettings::Vertex& vertex : settings.mVertices)
            {
                if (vertex.mInvMass > 0.0f)
                {
                    vertex.mInvMass = inverseMass;
                }
            }
        }
    } // namespace

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
            AcquireObjectLayer(m_settings.m_collisionLayer, m_settings.m_collisionGroupId, /*isMoving*/ true));
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
        AZStd::lock_guard lock(m_mutex);
        if (!m_physicsSystem || m_bodyId.IsInvalid())
        {
            return nullptr;
        }
        // The body itself, matching what the other body types return. Reading it needs a
        // lock, which is the caller's business once they have the pointer.
        return const_cast<JPH::BodyInterface*>(&m_physicsSystem->GetBodyInterface());
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

        JPH::RayCast ray(localStart, localDirection);
        JPH::RayCastResult hit;
        if (body.GetShape()->CastRay(ray, JPH::SubShapeIDCreator(), hit))
        {
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
            AcquireObjectLayer(m_settings.m_collisionLayer, m_settings.m_collisionGroupId, /*isMoving*/ true);

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
            !settings.m_size.IsClose(m_settings.m_size) || settings.m_resolution != m_settings.m_resolution ||
            !AZ::IsClose(settings.m_mass, m_settings.m_mass) || !AZ::IsClose(settings.m_compliance, m_settings.m_compliance) ||
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

    void JoltSoftBody::SetNumIterations(AZ::u32 iterations)
    {
        AZStd::lock_guard lock(m_mutex);
        m_settings.m_numIterations = AZ::GetMax(iterations, 1u);
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
        m_objectLayer = AcquireObjectLayer(m_settings.m_collisionLayer, m_settings.m_collisionGroupId, /*isMoving*/ true);

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

    JPH::Ref<JPH::SoftBodySharedSettings> JoltSoftBody::BuildSharedSettings(AZStd::vector<AZ::u32>& outTriangleIndices) const
    {
        outTriangleIndices.clear();

        const AZ::u32 resolution = AZ::GetMax(m_settings.m_resolution, 2u);

        if (m_settings.m_shape == JoltSoftBodyShape::Cube)
        {
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

            DistributeMass(*settings, m_settings.m_mass);
            CollectFaces(*settings, outTriangleIndices);
            return settings;
        }

        JPH::Ref<JPH::SoftBodySharedSettings> settings = new JPH::SoftBodySharedSettings();

        if (m_settings.m_shape == JoltSoftBodyShape::Cloth)
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
                    AddQuad(
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
                AddTriangle(
                    *settings, outTriangleIndices, northPole, ringVertex(1, segment), ringVertex(1, segment + 1));
            }

            for (AZ::u32 ring = 1; ring + 1 < rings; ++ring)
            {
                for (AZ::u32 segment = 0; segment < segments; ++segment)
                {
                    AddQuad(
                        *settings, outTriangleIndices, ringVertex(ring, segment), ringVertex(ring + 1, segment),
                        ringVertex(ring + 1, segment + 1), ringVertex(ring, segment + 1));
                }
            }

            for (AZ::u32 segment = 0; segment < segments; ++segment)
            {
                AddTriangle(
                    *settings, outTriangleIndices, southPole, ringVertex(rings - 1, segment + 1),
                    ringVertex(rings - 1, segment));
            }
        }

        // Constraints are generated from the faces: edges hold the sheet together, shear
        // edges stop quads collapsing diagonally, bend constraints resist folding.
        JPH::SoftBodySharedSettings::VertexAttributes attributes;
        attributes.mCompliance = m_settings.m_compliance;
        attributes.mShearCompliance = m_settings.m_compliance;
        attributes.mBendCompliance = m_settings.m_compliance;
        settings->CreateConstraints(&attributes, 1, JPH::SoftBodySharedSettings::EBendType::Distance);

        DistributeMass(*settings, m_settings.m_mass);
        settings->Optimize();
        return settings;
    }

    bool JoltSoftBody::CreateBody()
    {
        if (!m_physicsSystem)
        {
            return false;
        }

        AZStd::vector<AZ::u32> triangleIndices;
        JPH::Ref<JPH::SoftBodySharedSettings> sharedSettings = BuildSharedSettings(triangleIndices);
        if (!sharedSettings || sharedSettings->mVertices.empty())
        {
            return false;
        }

        JPH::SoftBodyCreationSettings creationSettings(
            sharedSettings, ToJoltR(m_worldTransform.GetTranslation()), ToJolt(m_worldTransform.GetRotation()), m_objectLayer);
        creationSettings.mNumIterations = AZ::GetMax(m_settings.m_numIterations, 1u);
        creationSettings.mLinearDamping = AZ::GetMax(m_settings.m_linearDamping, 0.0f);
        creationSettings.mPressure = m_settings.m_pressure;
        creationSettings.mGravityFactor = m_settings.m_gravityFactor;
        creationSettings.mAllowSleeping = m_settings.m_allowSleeping;

        // Creating and adding a body is only legal outside the physics step. This is called
        // from component activation and from settings changes, both on the main thread.
        m_bodyId = m_physicsSystem->GetBodyInterface().CreateAndAddSoftBody(creationSettings, JPH::EActivation::Activate);
        if (m_bodyId.IsInvalid())
        {
            return false;
        }

        m_triangleIndices = AZStd::move(triangleIndices);
        ++m_buildGeneration;
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
