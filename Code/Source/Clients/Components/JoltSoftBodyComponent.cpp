#include <Clients/Components/JoltSoftBodyComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Physics/SystemBus.h>

#include <SoftBody/JoltSoftBodyRender.h>

namespace JoltPhysics
{
    void JoltSoftBodyComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltSoftBodySettings>()
                ->Version(1)
                ->Field("Shape", &JoltSoftBodySettings::m_shape)
                ->Field("Pinning", &JoltSoftBodySettings::m_pinning)
                ->Field("Size", &JoltSoftBodySettings::m_size)
                ->Field("Resolution", &JoltSoftBodySettings::m_resolution)
                ->Field("Mass", &JoltSoftBodySettings::m_mass)
                ->Field("Compliance", &JoltSoftBodySettings::m_compliance)
                ->Field("NumIterations", &JoltSoftBodySettings::m_numIterations)
                ->Field("LinearDamping", &JoltSoftBodySettings::m_linearDamping)
                ->Field("Pressure", &JoltSoftBodySettings::m_pressure)
                ->Field("GravityFactor", &JoltSoftBodySettings::m_gravityFactor)
                ->Field("AllowSleeping", &JoltSoftBodySettings::m_allowSleeping)
                ;

            serializeContext->Class<JoltSoftBodyComponent, AZ::Component>()
                ->Version(1)
                ->Field("Settings", &JoltSoftBodyComponent::m_settings)
                ->Field("Visible", &JoltSoftBodyComponent::m_visible)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Enum<JoltSoftBodyShape>("Soft Body Shape", "Which geometry the soft body is built from")
                    ->Value("Cloth", JoltSoftBodyShape::Cloth)
                    ->Value("Cube", JoltSoftBodyShape::Cube)
                    ->Value("Balloon", JoltSoftBodyShape::Balloon)
                    ;

                editContext->Enum<JoltSoftBodyPinning>("Cloth Pinning", "Which cloth particles are held in place")
                    ->Value("None", JoltSoftBodyPinning::None)
                    ->Value("Corners", JoltSoftBodyPinning::Corners)
                    ->Value("Top edge", JoltSoftBodyPinning::TopEdge)
                    ;

                editContext->Class<JoltSoftBodySettings>("Soft Body Settings", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &JoltSoftBodySettings::m_shape,
                        "Shape", "Cloth is a flat sheet, Cube keeps its bulk, Balloon is inflated by pressure. "
                        "Changing this rebuilds the body.")
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &JoltSoftBodySettings::m_pinning,
                        "Pinning", "Which cloth particles are fixed in place. A cloth with nothing pinned falls. "
                        "Only applies to the Cloth shape.")
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
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &JoltSoftBodySettings::m_allowSleeping,
                        "Allow sleeping", "Lets the body stop simulating once it settles.")
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

        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        // Settings and transform first: Attach builds the particles immediately, so both
        // have to be in place before it runs rather than after.
        m_softBody.SetSettings(m_settings);
        m_softBody.SetTransform(worldTransform);
        if (m_enabled)
        {
            m_softBody.Attach(m_attachedSceneHandle);
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

        m_softBody.Detach();
        m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
    }

    void JoltSoftBodyComponent::OnTransformChanged(const AZ::Transform& /*local*/, const AZ::Transform& world)
    {
        // Rebuilds the body at the new placement. Moving a soft body every frame would
        // rebuild it every frame, so this suits placement rather than animation.
        m_softBody.SetTransform(world);
    }

    void JoltSoftBodyComponent::OnTick(float /*deltaTime*/, AZ::ScriptTimePoint /*time*/)
    {
        if (!m_visible)
        {
            return;
        }

        if (!m_softBody.CopyVertexPositions(m_vertexPositionCache))
        {
            return;
        }

        AzFramework::DebugDisplayRequestBus::BusPtr debugDisplayBus;
        AzFramework::DebugDisplayRequestBus::Bind(debugDisplayBus, AzFramework::g_defaultSceneEntityDebugDisplayId);
        if (auto* debugDisplay = AzFramework::DebugDisplayRequestBus::FindFirstHandler(debugDisplayBus))
        {
            DrawSoftBody(*debugDisplay, m_vertexPositionCache, m_softBody.GetTriangleIndices());
        }
    }

    void JoltSoftBodyComponent::SetPressure(float pressure)
    {
        m_settings.m_pressure = pressure;
        m_softBody.SetPressure(pressure);
    }

    float JoltSoftBodyComponent::GetPressure() const
    {
        return m_settings.m_pressure;
    }

    void JoltSoftBodyComponent::SetLinearDamping(float damping)
    {
        m_settings.m_linearDamping = damping;
        m_softBody.SetLinearDamping(damping);
    }

    float JoltSoftBodyComponent::GetLinearDamping() const
    {
        return m_settings.m_linearDamping;
    }

    void JoltSoftBodyComponent::SetGravityFactor(float factor)
    {
        m_settings.m_gravityFactor = factor;
        m_softBody.SetGravityFactor(factor);
    }

    float JoltSoftBodyComponent::GetGravityFactor() const
    {
        return m_settings.m_gravityFactor;
    }

    void JoltSoftBodyComponent::SetNumIterations(AZ::u32 iterations)
    {
        m_settings.m_numIterations = iterations;
        m_softBody.SetNumIterations(iterations);
    }

    AZ::u32 JoltSoftBodyComponent::GetNumIterations() const
    {
        return m_settings.m_numIterations;
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
            m_softBody.Attach(m_attachedSceneHandle);
        }
        else
        {
            m_softBody.Detach();
        }
    }

    bool JoltSoftBodyComponent::IsEnabled() const
    {
        return m_enabled;
    }

    AZ::u32 JoltSoftBodyComponent::GetVertexCount() const
    {
        return m_softBody.GetVertexCount();
    }

    AZ::Aabb JoltSoftBodyComponent::GetWorldBounds() const
    {
        return m_softBody.GetWorldBounds();
    }

    AZ::Vector3 JoltSoftBodyComponent::GetVertexPosition(AZ::u32 index) const
    {
        return m_softBody.GetVertexPosition(index);
    }
} // namespace JoltPhysics
