#include <Editor/Components/EditorJoltRigidBodyComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltRigidBodyComponent.h>
#include <Editor/Components/EditorJoltDebugDrawUtils.h>

namespace JoltPhysics
{
    void EditorJoltRigidBodyComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltRigidBodyComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(1)
                ->Field("RigidBodyConfiguration", &EditorJoltRigidBodyComponent::m_configuration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                // The per-field edit context for AzPhysics::RigidBodyConfiguration is
                // registered by the runtime JoltRigidBodyComponent::Reflect, which also
                // runs in the editor module (same dll); only the wrapper element is added here.
                editContext->Class<EditorJoltRigidBodyComponent>(
                    "Jolt Rigid Body", "Dynamic rigid body simulated by the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltRigidBodyComponent::m_configuration,
                        "Configuration", "Rigid body configuration")
                    ;
            }
        }
    }

    void EditorJoltRigidBodyComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltRigidBodyService"));
    }

    void EditorJoltRigidBodyComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltRigidBodyService"));
        incompatible.push_back(AZ_CRC_CE("JoltStaticRigidBodyService"));
    }

    void EditorJoltRigidBodyComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        required.push_back(AZ_CRC_CE("JoltColliderService"));
    }

    void EditorJoltRigidBodyComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltRigidBodyComponent>())
        {
            component->GetConfiguration() = m_configuration;
        }
    }

    void EditorJoltRigidBodyComponent::Activate()
    {
        AzToolsFramework::Components::EditorComponentBase::Activate();
        AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(GetEntityId());
    }

    void EditorJoltRigidBodyComponent::Deactivate()
    {
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();
        AzToolsFramework::Components::EditorComponentBase::Deactivate();
    }

    void EditorJoltRigidBodyComponent::DisplayEntityViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo,
        AzFramework::DebugDisplayRequests& debugDisplay)
    {
        // With ComputeCenterOfMass set the offset is ignored and the real centre of
        // mass comes from the shapes, which is not known at edit time. Drawing the
        // stale offset then would be actively misleading, so draw nothing.
        if (m_configuration.m_computeCenterOfMass)
        {
            return;
        }

        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        worldTransform.ExtractUniformScale();

        constexpr float MarkerRadius = 0.1f;
        const AZ::Transform centreOfMass =
            worldTransform * AZ::Transform::CreateTranslation(m_configuration.m_centerOfMassOffset);

        EditorDebugDraw::DrawWireSphere(
            debugDisplay, centreOfMass, MarkerRadius, EditorDebugDraw::LimitColor);
        // Crosshair through the marker, so it stays locatable when the sphere is
        // small on screen or buried inside a collider.
        for (AZ::u32 axis = 0; axis < 3; ++axis)
        {
            AZ::Vector3 arm = AZ::Vector3::CreateZero();
            arm.SetElement(axis, MarkerRadius * 2.0f);
            EditorDebugDraw::DrawLine(
                debugDisplay, centreOfMass.TransformPoint(-arm), centreOfMass.TransformPoint(arm),
                EditorDebugDraw::LimitColor);
        }
    }

} // namespace JoltPhysics
