#include <Editor/Components/EditorJoltVehicleComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltVehicleComponent.h>
#include <Editor/Components/EditorJoltDebugDrawUtils.h>
#include <Editor/Components/JoltVehicleComponentMode.h>
#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void EditorJoltVehicleComponent::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<AzToolsFramework::ComponentModeFramework::ComponentModeDelegate>(context);
        Internal::ReflectOnce<JoltVehicleComponentMode>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltVehicleComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(2)
                ->Field("VehicleConfiguration", &EditorJoltVehicleComponent::m_configuration)
                ->Field("ComponentMode", &EditorJoltVehicleComponent::m_componentModeDelegate)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                // The JoltVehicleConfiguration field-level edit context is registered by the
                // runtime JoltVehicleComponent::Reflect, which also runs in this dll.
                editContext->Class<EditorJoltVehicleComponent>(
                    "Jolt Vehicle", "Four-wheeled vehicle simulated by the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltVehicleComponent::m_configuration,
                        "Vehicle Configuration", "Vehicle chassis, wheel and controller settings")
                    // Renders the Edit button that enters component mode.
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltVehicleComponent::m_componentModeDelegate,
                        "Component Mode", "Wheel placement component mode")
                    ;
            }
        }
    }

    void EditorJoltVehicleComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltVehicleService"));
    }

    void EditorJoltVehicleComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltVehicleService"));
    }

    void EditorJoltVehicleComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        required.push_back(AZ_CRC_CE("JoltRigidBodyService"));
    }

    void EditorJoltVehicleComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltVehicleComponent>())
        {
            component->GetConfiguration() = m_configuration;
        }
    }

    void EditorJoltVehicleComponent::Activate()
    {
        AzToolsFramework::Components::EditorComponentBase::Activate();

        const AZ::EntityComponentIdPair entityComponentIdPair(GetEntityId(), GetId());
        AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(GetEntityId());
        JoltVehicleWheelRequestBus::Handler::BusConnect(entityComponentIdPair);
        // Addressed by entity, not by entity+component, unlike the wheel bus.
        AzToolsFramework::EditorComponentSelectionRequestsBus::Handler::BusConnect(GetEntityId());

        m_componentModeDelegate.ConnectWithSingleComponentMode<EditorJoltVehicleComponent, JoltVehicleComponentMode>(
            entityComponentIdPair, this);
    }

    void EditorJoltVehicleComponent::Deactivate()
    {
        m_componentModeDelegate.Disconnect();

        AzToolsFramework::EditorComponentSelectionRequestsBus::Handler::BusDisconnect();
        JoltVehicleWheelRequestBus::Handler::BusDisconnect();
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();

        AzToolsFramework::Components::EditorComponentBase::Deactivate();
    }

    AZ::u32 EditorJoltVehicleComponent::GetWheelCount() const
    {
        return static_cast<AZ::u32>(m_configuration.m_wheels.size());
    }

    AZ::Vector3 EditorJoltVehicleComponent::GetWheelPosition(AZ::u32 wheelIndex) const
    {
        if (wheelIndex >= m_configuration.m_wheels.size())
        {
            return AZ::Vector3::CreateZero();
        }
        return m_configuration.m_wheels[wheelIndex].m_position;
    }

    void EditorJoltVehicleComponent::SetWheelPosition(AZ::u32 wheelIndex, const AZ::Vector3& position)
    {
        if (wheelIndex < m_configuration.m_wheels.size())
        {
            m_configuration.m_wheels[wheelIndex].m_position = position;
        }
    }

    AZ::Transform EditorJoltVehicleComponent::GetChassisSpace() const
    {
        AZ::Transform chassisTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(chassisTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        // Wheel positions are plain chassis-space offsets, so scale would move the
        // handles somewhere the wheels are not.
        chassisTransform.ExtractUniformScale();
        return chassisTransform;
    }

    AZ::Aabb EditorJoltVehicleComponent::GetEditorSelectionBoundsViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo)
    {
        // Around the wheels rather than the chassis: the chassis has its own collider
        // component answering for its own bounds, and this is what the vehicle draws.
        AZ::Aabb localBounds = AZ::Aabb::CreateNull();
        for (const JoltWheelConfiguration& wheel : m_configuration.m_wheels)
        {
            const AZ::Vector3 lowest = wheel.m_position - AZ::Vector3::CreateAxisZ(wheel.m_suspensionMaxLength);
            localBounds.AddAabb(AZ::Aabb::CreateCenterRadius(wheel.m_position, wheel.m_radius));
            localBounds.AddAabb(AZ::Aabb::CreateCenterRadius(lowest, wheel.m_radius));
        }
        if (!localBounds.IsValid())
        {
            return AZ::Aabb::CreateNull();
        }
        return localBounds.GetTransformedAabb(GetChassisSpace());
    }

    bool EditorJoltVehicleComponent::SupportsEditorRayIntersect()
    {
        return false;
    }

    void EditorJoltVehicleComponent::DisplayEntityViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo,
        AzFramework::DebugDisplayRequests& debugDisplay)
    {
        AZ::Transform chassisTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(chassisTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        chassisTransform.ExtractUniformScale();

        for (const JoltWheelConfiguration& wheel : m_configuration.m_wheels)
        {
            // JoltVehicle drops the suspension along -Z, spins the wheel about Y and
            // points it forward along X; m_position is the attachment point, so the
            // wheel centre hangs below it by the current suspension length.
            const AZ::Vector3 fullyRaised = wheel.m_position - AZ::Vector3::CreateAxisZ(wheel.m_suspensionMinLength);
            const AZ::Vector3 fullyDropped = wheel.m_position - AZ::Vector3::CreateAxisZ(wheel.m_suspensionMaxLength);

            // Suspension travel, from the attachment point through the whole range.
            EditorDebugDraw::DrawLine(
                debugDisplay, chassisTransform.TransformPoint(wheel.m_position),
                chassisTransform.TransformPoint(fullyDropped), EditorDebugDraw::LinkColor);
            EditorDebugDraw::DrawLine(
                debugDisplay, chassisTransform.TransformPoint(fullyRaised),
                chassisTransform.TransformPoint(fullyDropped), EditorDebugDraw::LimitColor);

            // The wheel is drawn mid-travel, which is roughly where it sits at rest.
            const AZ::Vector3 centre = (fullyRaised + fullyDropped) * 0.5f;
            const AZ::Transform wheelTransform = chassisTransform * AZ::Transform::CreateTranslation(centre);
            for (const float side : { -0.5f, 0.5f })
            {
                EditorColliderDraw::DrawWireCircle(
                    debugDisplay, wheelTransform, wheel.m_radius, /*axis*/ 1, side * wheel.m_width);
            }
        }
    }

} // namespace JoltPhysics
