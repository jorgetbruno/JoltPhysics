#include <Clients/Components/JoltSoftBodyComponent.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Physics/PropertyTypes.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/SystemBus.h>

#include <SoftBody/JoltSoftBodyRender.h>
#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void JoltSoftBodyComponent::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectEBusOnce(context, "JoltSoftBodyRequestBus",
            [](AZ::BehaviorContext* behaviorContext)
            {
                behaviorContext->EBus<JoltSoftBodyRequestBus>("JoltSoftBodyRequestBus")
                    ->Attribute(AZ::Script::Attributes::Category, "Jolt Physics")
                    ->Event("SetPressure", &JoltSoftBodyRequests::SetPressure)
                    ->Event("GetPressure", &JoltSoftBodyRequests::GetPressure)
                    ->Event("SetLinearDamping", &JoltSoftBodyRequests::SetLinearDamping)
                    ->Event("GetLinearDamping", &JoltSoftBodyRequests::GetLinearDamping)
                    ->Event("SetGravityFactor", &JoltSoftBodyRequests::SetGravityFactor)
                    ->Event("GetGravityFactor", &JoltSoftBodyRequests::GetGravityFactor)
                    ->Event("SetNumIterations", &JoltSoftBodyRequests::SetNumIterations)
                    ->Event("GetNumIterations", &JoltSoftBodyRequests::GetNumIterations)
                    ->Event("SetFriction", &JoltSoftBodyRequests::SetFriction)
                    ->Event("GetFriction", &JoltSoftBodyRequests::GetFriction)
                    ->Event("SetRestitution", &JoltSoftBodyRequests::SetRestitution)
                    ->Event("GetRestitution", &JoltSoftBodyRequests::GetRestitution)
                    ->Event("SetEnabled", &JoltSoftBodyRequests::SetEnabled)
                    ->Event("IsEnabled", &JoltSoftBodyRequests::IsEnabled)
                    // What a script polls to follow the deformation, which the entity
                    // transform alone does not describe. The bulk reads are what make
                    // script-driven rendering feasible: one body lock per frame, not one
                    // per particle.
                    ->Event("GetVertexCount", &JoltSoftBodyRequests::GetVertexCount)
                    ->Event("GetVertexPosition", &JoltSoftBodyRequests::GetVertexPosition)
                    ->Event("GetVertexPositions", &JoltSoftBodyRequests::GetVertexPositions)
                    ->Event("GetTriangleIndices", &JoltSoftBodyRequests::GetTriangleIndices)
                    ->Event("GetWorldBounds", &JoltSoftBodyRequests::GetWorldBounds)
                    // Runtime particle control: grab and release cloth, kick a balloon.
                    ->Event("SetVertexPinned", &JoltSoftBodyRequests::SetVertexPinned)
                    ->Event("IsVertexPinned", &JoltSoftBodyRequests::IsVertexPinned)
                    ->Event("SetVertexVelocity", &JoltSoftBodyRequests::SetVertexVelocity)
                    ->Event("GetVertexVelocity", &JoltSoftBodyRequests::GetVertexVelocity)
                    // SetCollisionLayer / SetCollisionGroupId take AzPhysics types that
                    // script cannot construct, so they stay C++-only.
                    ;
            });

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            // Version 3 only adds fields; older data loads with the new fields at their
            // defaults, so no converter is needed.
            serializeContext->Class<JoltSoftBodySettings>()
                ->Version(3)
                ->Field("Shape", &JoltSoftBodySettings::m_shape)
                ->Field("Pinning", &JoltSoftBodySettings::m_pinning)
                ->Field("Size", &JoltSoftBodySettings::m_size)
                ->Field("Resolution", &JoltSoftBodySettings::m_resolution)
                ->Field("Mass", &JoltSoftBodySettings::m_mass)
                ->Field("Compliance", &JoltSoftBodySettings::m_compliance)
                ->Field("LraType", &JoltSoftBodySettings::m_lraType)
                ->Field("NumIterations", &JoltSoftBodySettings::m_numIterations)
                ->Field("LinearDamping", &JoltSoftBodySettings::m_linearDamping)
                ->Field("Pressure", &JoltSoftBodySettings::m_pressure)
                ->Field("GravityFactor", &JoltSoftBodySettings::m_gravityFactor)
                ->Field("Friction", &JoltSoftBodySettings::m_friction)
                ->Field("Restitution", &JoltSoftBodySettings::m_restitution)
                ->Field("VertexRadius", &JoltSoftBodySettings::m_vertexRadius)
                ->Field("MaxLinearVelocity", &JoltSoftBodySettings::m_maxLinearVelocity)
                ->Field("UpdatePosition", &JoltSoftBodySettings::m_updatePosition)
                ->Field("DoubleSidedFaces", &JoltSoftBodySettings::m_doubleSidedFaces)
                ->Field("AllowSleeping", &JoltSoftBodySettings::m_allowSleeping)
                ->Field("CollisionLayer", &JoltSoftBodySettings::m_collisionLayer)
                ->Field("CollisionGroupId", &JoltSoftBodySettings::m_collisionGroupId)
                ;

            JoltSoftBodyConfiguration::Reflect(context);

            serializeContext->Class<JoltSoftBodyComponent, AZ::Component>()
                ->Version(1)
                ->Field("Settings", &JoltSoftBodyComponent::m_settings)
                ->Field("Visible", &JoltSoftBodyComponent::m_visible)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                // Enum values are attached per data element with EnumAttribute, the idiom the
                // vehicle and joint configurations use. editContext->Enum registers the enum
                // globally and asserts on a second call, and Reflect runs once per descriptor
                // registration - this component is registered by the runtime module and the
                // editor module both.
                editContext->Class<JoltSoftBodySettings>("Soft Body Settings", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &JoltSoftBodySettings::m_shape,
                        "Shape", "Cloth is a flat sheet, Cube keeps its bulk, Balloon is inflated by pressure. "
                        "Changing this rebuilds the body.")
                        ->EnumAttribute(JoltSoftBodyShape::Cloth, "Cloth")
                        ->EnumAttribute(JoltSoftBodyShape::Cube, "Cube")
                        ->EnumAttribute(JoltSoftBodyShape::Balloon, "Balloon")
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &JoltSoftBodySettings::m_pinning,
                        "Pinning", "Which cloth particles are fixed in place. A cloth with nothing pinned falls. "
                        "Only applies to the Cloth shape.")
                        ->EnumAttribute(JoltSoftBodyPinning::None, "None")
                        ->EnumAttribute(JoltSoftBodyPinning::Corners, "Corners")
                        ->EnumAttribute(JoltSoftBodyPinning::TopEdge, "Top edge")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodySettings::m_size,
                        "Size", "Extents in metres. Cloth uses X and Y; Cube and Balloon use X alone.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.01f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodySettings::m_resolution,
                        "Resolution", "Particles along each axis. Cost grows as the square of this for a cloth and "
                        "the cube of it for a cube, so raise it carefully.")
                        ->Attribute(AZ::Edit::Attributes::Min, 2)
                        ->Attribute(AZ::Edit::Attributes::Max, 32)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodySettings::m_mass,
                        "Mass", "Total mass, divided evenly over the particles.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.001f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " kg")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodySettings::m_compliance,
                        "Compliance", "Edge stiffness. 0 is inextensible; larger values stretch like rubber.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &JoltSoftBodySettings::m_lraType,
                        "Long range attachment", "Tethers every free particle to its closest pinned one at the "
                        "rest-pose distance, which keeps cloth from stretching no matter how few solver iterations "
                        "run. Needs pinned particles; Geodesic measures the tether along the surface, so a draped "
                        "sheet keeps its slack.")
                        ->EnumAttribute(JoltSoftBodyLraType::None, "None")
                        ->EnumAttribute(JoltSoftBodyLraType::EuclideanDistance, "Euclidean distance")
                        ->EnumAttribute(JoltSoftBodyLraType::GeodesicDistance, "Geodesic distance")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodySettings::m_numIterations,
                        "Solver iterations", "More iterations is stiffer and more expensive.")
                        ->Attribute(AZ::Edit::Attributes::Min, 1)
                        ->Attribute(AZ::Edit::Attributes::Max, 32)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodySettings::m_linearDamping,
                        "Linear damping", "Velocity lost per second. Higher settles the body sooner.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodySettings::m_pressure,
                        "Pressure", "Internal pressure. Only does anything on a closed shape, so leave it at 0 for "
                        "cloth and raise it to inflate a balloon.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodySettings::m_gravityFactor,
                        "Gravity factor", "Multiplier on gravity for this body alone. 0 makes it hang in the air.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodySettings::m_friction,
                        "Friction", "Surface friction when sliding over other bodies. A soft body has no physics "
                        "material, so this plays that role.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodySettings::m_restitution,
                        "Restitution", "Bounciness when colliding. 0 lands dead, 1 keeps all its energy.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 1.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodySettings::m_vertexRadius,
                        "Vertex radius", "Particle radius. A little padding keeps the surface from z-fighting with "
                        "whatever it rests on.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodySettings::m_maxLinearVelocity,
                        "Max particle velocity", "Cap on any particle's speed, the guard against the simulation "
                        "exploding on a hard impact.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m/s")
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &JoltSoftBodySettings::m_updatePosition,
                        "Update position", "Whether the body's position follows its particles. Turn off for "
                        "something anchored to the static world.")
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &JoltSoftBodySettings::m_doubleSidedFaces,
                        "Double sided faces", "Collide and raycast the surface from both sides. Almost always what "
                        "a thin cloth wants; a closed shape can turn it off.")
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &JoltSoftBodySettings::m_allowSleeping,
                        "Allow sleeping", "Lets the body stop simulating once it settles.")
                    // Same selectors the colliders and character controller use, so a soft
                    // body is filtered by the project's collision layers like any other body.
                    ->DataElement(Physics::Edit::CollisionLayerSelector, &JoltSoftBodySettings::m_collisionLayer,
                        "Collision Layer", "Which collision layer this soft body is on.")
                    ->DataElement(Physics::Edit::CollisionGroupSelector, &JoltSoftBodySettings::m_collisionGroupId,
                        "Collides With", "Which collision layers this soft body collides with.")
                    ;

                editContext->Class<JoltSoftBodyComponent>(
                    "Jolt Soft Body", "Deformable cloth or a pressurised body simulated by Jolt")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::RemoveableByUser, true)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodyComponent::m_settings,
                        "Soft body", "Soft body properties")
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &JoltSoftBodyComponent::m_visible,
                        "Visible", "Draw the simulated surface. A soft body has no mesh asset and deforms every "
                        "step, so this drawing is the only way to see it.")
                    ;
            }
        }
    }

    void JoltSoftBodyComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltSoftBodyService"));
    }

    void JoltSoftBodyComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltSoftBodyService"));
    }

    void JoltSoftBodyComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void JoltSoftBodyComponent::Activate()
    {
        Physics::DefaultWorldBus::BroadcastResult(
            m_attachedSceneHandle, &Physics::DefaultWorldRequests::GetDefaultSceneHandle);

        if (m_enabled)
        {
            CreateSoftBody();
        }

        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
        JoltSoftBodyRequestBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();
    }

    void JoltSoftBodyComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        JoltSoftBodyRequestBus::Handler::BusDisconnect();
        AZ::TransformNotificationBus::Handler::BusDisconnect();

        DestroySoftBody();
        m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
    }

    void JoltSoftBodyComponent::OnTransformChanged(const AZ::Transform& /*local*/, const AZ::Transform& world)
    {
        // Rebuilds the body at the new placement. Moving a soft body every frame would
        // rebuild it every frame, so this suits placement rather than animation.
        if (auto* softBody = GetSoftBody())
        {
            softBody->SetTransform(world);
        }
    }

    void JoltSoftBodyComponent::OnTick(float /*deltaTime*/, AZ::ScriptTimePoint /*time*/)
    {
        if (!m_visible)
        {
            return;
        }

        auto* softBody = GetSoftBody();
        if (!softBody || !softBody->CopyVertexPositions(m_vertexPositionCache))
        {
            return;
        }

        AzFramework::DebugDisplayRequestBus::BusPtr debugDisplayBus;
        AzFramework::DebugDisplayRequestBus::Bind(debugDisplayBus, AzFramework::g_defaultSceneEntityDebugDisplayId);
        if (auto* debugDisplay = AzFramework::DebugDisplayRequestBus::FindFirstHandler(debugDisplayBus))
        {
            DrawSoftBody(*debugDisplay, m_vertexPositionCache, softBody->GetTriangleIndices());
        }
    }

    JoltSoftBody* JoltSoftBodyComponent::GetSoftBody() const
    {
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        if (!sceneInterface || m_softBodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
        {
            return nullptr;
        }
        return azdynamic_cast<JoltSoftBody*>(
            sceneInterface->GetSimulatedBodyFromHandle(m_attachedSceneHandle, m_softBodyHandle));
    }

    void JoltSoftBodyComponent::CreateSoftBody()
    {
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        if (!sceneInterface || m_attachedSceneHandle == AzPhysics::InvalidSceneHandle)
        {
            return;
        }

        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        // The particles are generated in world space at creation, so the placement has to
        // be part of the configuration rather than applied afterwards.
        JoltSoftBodyConfiguration configuration;
        configuration.m_settings = m_settings;
        configuration.m_position = worldTransform.GetTranslation();
        configuration.m_orientation = worldTransform.GetRotation();
        configuration.m_entityId = GetEntityId();
        configuration.m_debugName = GetEntity() ? GetEntity()->GetName() : AZStd::string();

        m_softBodyHandle = sceneInterface->AddSimulatedBody(m_attachedSceneHandle, &configuration);
    }

    void JoltSoftBodyComponent::DestroySoftBody()
    {
        if (auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
            sceneInterface && m_softBodyHandle != AzPhysics::InvalidSimulatedBodyHandle)
        {
            sceneInterface->RemoveSimulatedBody(m_attachedSceneHandle, m_softBodyHandle);
        }
        m_softBodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
    }

    void JoltSoftBodyComponent::SetPressure(float pressure)
    {
        m_settings.m_pressure = pressure;
        if (auto* softBody = GetSoftBody())
        {
            softBody->SetPressure(pressure);
        }
    }

    float JoltSoftBodyComponent::GetPressure() const
    {
        return m_settings.m_pressure;
    }

    void JoltSoftBodyComponent::SetLinearDamping(float damping)
    {
        m_settings.m_linearDamping = damping;
        if (auto* softBody = GetSoftBody())
        {
            softBody->SetLinearDamping(damping);
        }
    }

    float JoltSoftBodyComponent::GetLinearDamping() const
    {
        return m_settings.m_linearDamping;
    }

    void JoltSoftBodyComponent::SetGravityFactor(float factor)
    {
        m_settings.m_gravityFactor = factor;
        if (auto* softBody = GetSoftBody())
        {
            softBody->SetGravityFactor(factor);
        }
    }

    float JoltSoftBodyComponent::GetGravityFactor() const
    {
        return m_settings.m_gravityFactor;
    }

    void JoltSoftBodyComponent::SetNumIterations(AZ::u32 iterations)
    {
        m_settings.m_numIterations = iterations;
        if (auto* softBody = GetSoftBody())
        {
            softBody->SetNumIterations(iterations);
        }
    }

    AZ::u32 JoltSoftBodyComponent::GetNumIterations() const
    {
        return m_settings.m_numIterations;
    }

    void JoltSoftBodyComponent::SetFriction(float friction)
    {
        m_settings.m_friction = friction;
        if (auto* softBody = GetSoftBody())
        {
            softBody->SetFriction(friction);
        }
    }

    float JoltSoftBodyComponent::GetFriction() const
    {
        return m_settings.m_friction;
    }

    void JoltSoftBodyComponent::SetRestitution(float restitution)
    {
        m_settings.m_restitution = restitution;
        if (auto* softBody = GetSoftBody())
        {
            softBody->SetRestitution(restitution);
        }
    }

    float JoltSoftBodyComponent::GetRestitution() const
    {
        return m_settings.m_restitution;
    }

    void JoltSoftBodyComponent::SetCollisionLayer(const AzPhysics::CollisionLayer& layer)
    {
        m_settings.m_collisionLayer = layer;
        if (auto* softBody = GetSoftBody())
        {
            softBody->SetCollisionLayer(layer);
        }
    }

    AzPhysics::CollisionLayer JoltSoftBodyComponent::GetCollisionLayer() const
    {
        return m_settings.m_collisionLayer;
    }

    void JoltSoftBodyComponent::SetCollisionGroupId(const AzPhysics::CollisionGroups::Id& groupId)
    {
        m_settings.m_collisionGroupId = groupId;
        if (auto* softBody = GetSoftBody())
        {
            softBody->SetCollisionGroupId(groupId);
        }
    }

    AzPhysics::CollisionGroups::Id JoltSoftBodyComponent::GetCollisionGroupId() const
    {
        return m_settings.m_collisionGroupId;
    }

    void JoltSoftBodyComponent::SetEnabled(bool enabled)
    {
        if (enabled == m_enabled)
        {
            return;
        }
        m_enabled = enabled;

        if (enabled)
        {
            CreateSoftBody();
        }
        else
        {
            DestroySoftBody();
        }
    }

    bool JoltSoftBodyComponent::IsEnabled() const
    {
        return m_enabled;
    }

    AZ::u32 JoltSoftBodyComponent::GetVertexCount() const
    {
        auto* softBody = GetSoftBody();
        return softBody ? softBody->GetVertexCount() : 0;
    }

    AZ::Aabb JoltSoftBodyComponent::GetWorldBounds() const
    {
        auto* softBody = GetSoftBody();
        return softBody ? softBody->GetWorldBounds() : AZ::Aabb::CreateNull();
    }

    AZ::Vector3 JoltSoftBodyComponent::GetVertexPosition(AZ::u32 index) const
    {
        auto* softBody = GetSoftBody();
        return softBody ? softBody->GetVertexPosition(index) : AZ::Vector3::CreateZero();
    }

    AZStd::vector<AZ::Vector3> JoltSoftBodyComponent::GetVertexPositions() const
    {
        AZStd::vector<AZ::Vector3> positions;
        if (auto* softBody = GetSoftBody())
        {
            softBody->CopyVertexPositions(positions);
        }
        return positions;
    }

    AZStd::vector<AZ::u32> JoltSoftBodyComponent::GetTriangleIndices() const
    {
        auto* softBody = GetSoftBody();
        return softBody ? softBody->GetTriangleIndices() : AZStd::vector<AZ::u32>();
    }

    bool JoltSoftBodyComponent::SetVertexPinned(AZ::u32 index, bool pinned)
    {
        auto* softBody = GetSoftBody();
        return softBody && softBody->SetVertexPinned(index, pinned);
    }

    bool JoltSoftBodyComponent::IsVertexPinned(AZ::u32 index) const
    {
        auto* softBody = GetSoftBody();
        return softBody && softBody->IsVertexPinned(index);
    }

    bool JoltSoftBodyComponent::SetVertexVelocity(AZ::u32 index, const AZ::Vector3& velocity)
    {
        auto* softBody = GetSoftBody();
        return softBody && softBody->SetVertexVelocity(index, velocity);
    }

    AZ::Vector3 JoltSoftBodyComponent::GetVertexVelocity(AZ::u32 index) const
    {
        auto* softBody = GetSoftBody();
        return softBody ? softBody->GetVertexVelocity(index) : AZ::Vector3::CreateZero();
    }
} // namespace JoltPhysics
