#include <Editor/Components/EditorJoltVehicleComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltVehicleComponent.h>
#include <Editor/Components/EditorJoltDebugDrawUtils.h>

namespace JoltPhysics
{
    void EditorJoltVehicleComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltVehicleComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(1)
                ->Field("VehicleConfiguration", &EditorJoltVehicleComponent::m_configuration)
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
        AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(GetEntityId());
    }

    void EditorJoltVehicleComponent::Deactivate()
    {
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();
        AzToolsFramework::Components::EditorComponentBase::Deactivate();
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
