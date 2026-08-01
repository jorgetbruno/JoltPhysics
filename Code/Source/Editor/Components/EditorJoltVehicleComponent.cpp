#include <Editor/Components/EditorJoltVehicleComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <AzFramework/Physics/Configuration/RigidBodyConfiguration.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Physics/SystemBus.h>

#include <Clients/Components/JoltVehicleComponent.h>
#include <Editor/Components/EditorJoltDebugDrawUtils.h>
#include <Editor/Components/JoltVehicleComponentMode.h>
#include <Scene/JoltScene.h>
#include <System/JoltSystem.h>
#include <Utils/ReflectionUtils.h>
#include <Vehicle/JoltVehicle.h>

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
                    "Jolt Vehicle", "Wheeled, motorcycle or tracked vehicle simulated by the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltVehicleComponent::m_configuration,
                        "Vehicle Configuration", "Vehicle chassis, wheel and controller settings")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltVehicleComponent::OnConfigurationChanged)
                    ->UIElement(AZ::Edit::UIHandlers::Button, "",
                        "Simulates the vehicle for a few seconds on flat ground at the height found under it in "
                        "the editor world, and shows the settled chassis and wheel poses as a ghost - suspension "
                        "rest pose without entering game mode. Any configuration edit clears the ghost.")
                        ->Attribute(AZ::Edit::Attributes::ButtonText, "Preview suspension settle")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &EditorJoltVehicleComponent::OnSettlePreviewPressed)
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

    AZ::u32 EditorJoltVehicleComponent::OnSettlePreviewPressed()
    {
        RunSettlePreview();
        return AZ::Edit::PropertyRefreshLevels::None;
    }

    AZ::u32 EditorJoltVehicleComponent::OnConfigurationChanged()
    {
        m_hasSettlePreview = false;
        m_settledWheels.clear();
        return AZ::Edit::PropertyRefreshLevels::None;
    }

    void EditorJoltVehicleComponent::RunSettlePreview()
    {
        m_hasSettlePreview = false;
        m_settledWheels.clear();

        auto* joltSystem = GetJoltSystem();
        if (joltSystem == nullptr)
        {
            AZ_Warning("JoltPhysics", false, "Settle preview: no physics system is active.");
            return;
        }

        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        worldTransform.ExtractUniformScale();

        JoltVehicleConfiguration configuration = m_configuration;
        configuration.m_debugName = GetEntity() ? GetEntity()->GetName() : AZStd::string();
        // The settle needs wheels; an empty list means the type's default layout, the
        // same substitution JoltVehicle itself makes.
        const AZStd::vector<JoltWheelConfiguration>& wheels =
            !configuration.m_wheels.empty() ? configuration.m_wheels
                                            : JoltVehicleConfiguration().m_wheels; // empty stays empty; handled below

        // Extent of the wheel attachments, for the stand-in chassis and the fallback
        // ground height.
        AZ::Aabb wheelBounds = AZ::Aabb::CreateNull();
        float maxWheelDrop = 0.0f;
        for (const JoltWheelConfiguration& wheel : wheels)
        {
            wheelBounds.AddPoint(wheel.m_position);
            maxWheelDrop = AZStd::max(maxWheelDrop, wheel.m_suspensionMaxLength + wheel.m_radius);
        }
        if (!wheelBounds.IsValid())
        {
            // No authored wheels: a default layout will be used by the vehicle; span a
            // car-sized stand-in.
            wheelBounds = AZ::Aabb::CreateCenterHalfExtents(AZ::Vector3(0.0f, 0.0f, -0.2f), AZ::Vector3(1.0f, 0.7f, 0.05f));
            maxWheelDrop = 0.8f;
        }

        // Ground height: what the editor world has under the vehicle (the edit-mode
        // collider bodies), or a plane below full droop when nothing is there.
        float groundZ = worldTransform.GetTranslation().GetZ() + wheelBounds.GetMin().GetZ() - maxWheelDrop;
        AzPhysics::SceneHandle editorSceneHandle = AzPhysics::InvalidSceneHandle;
        Physics::EditorWorldBus::BroadcastResult(
            editorSceneHandle, &Physics::EditorWorldRequests::GetEditorSceneHandle);
        if (AzPhysics::Scene* editorScene = joltSystem->GetScene(editorSceneHandle))
        {
            AzPhysics::RayCastRequest groundRay;
            groundRay.m_start = worldTransform.GetTranslation();
            groundRay.m_direction = AZ::Vector3(0.0f, 0.0f, -1.0f);
            groundRay.m_distance = 200.0f;
            const AzPhysics::SceneQueryHits hits = editorScene->QueryScene(&groundRay);
            if (!hits.m_hits.empty())
            {
                groundZ = hits.m_hits[0].m_position.GetZ();
            }
        }

        // A private throwaway scene: nothing here can disturb the editor world, and the
        // editor world cannot disturb the settle.
        AzPhysics::SceneConfiguration previewSceneConfig = joltSystem->GetDefaultSceneConfiguration();
        previewSceneConfig.m_sceneName = "VehicleSettlePreview";
        const AzPhysics::SceneHandle previewSceneHandle = joltSystem->AddScene(previewSceneConfig);
        AzPhysics::Scene* previewScene = joltSystem->GetScene(previewSceneHandle);
        if (previewScene == nullptr)
        {
            AZ_Warning("JoltPhysics", false, "Settle preview: could not create the preview scene.");
            return;
        }

        {
            auto groundCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
            auto groundShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            groundShape->m_dimensions = AZ::Vector3(400.0f, 400.0f, 1.0f);
            AzPhysics::StaticRigidBodyConfiguration groundConfig;
            groundConfig.m_position = AZ::Vector3(
                worldTransform.GetTranslation().GetX(), worldTransform.GetTranslation().GetY(), groundZ - 0.5f);
            groundConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(groundCollider, groundShape);
            previewScene->AddSimulatedBody(&groundConfig);
        }

        // Stand-in chassis: a thin slab spanning the wheel attachments, sitting above
        // them so it cannot ground out where the real chassis colliders would not.
        AzPhysics::SimulatedBodyHandle chassisHandle = AzPhysics::InvalidSimulatedBodyHandle;
        {
            auto chassisCollider = AZStd::make_shared<Physics::ColliderConfiguration>();
            chassisCollider->m_position = AZ::Vector3(
                wheelBounds.GetCenter().GetX(), wheelBounds.GetCenter().GetY(), wheelBounds.GetMax().GetZ() + 0.15f);
            auto chassisShape = AZStd::make_shared<Physics::BoxShapeConfiguration>();
            chassisShape->m_dimensions = AZ::Vector3(
                wheelBounds.GetExtents().GetX() + 0.6f, wheelBounds.GetExtents().GetY() + 0.6f, 0.2f);
            AzPhysics::RigidBodyConfiguration chassisConfig;
            chassisConfig.m_position = worldTransform.GetTranslation();
            chassisConfig.m_orientation = worldTransform.GetRotation();
            chassisConfig.m_mass = configuration.m_chassisMass > 0.0f ? configuration.m_chassisMass : 1200.0f;
            chassisConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(chassisCollider, chassisShape);
            chassisHandle = previewScene->AddSimulatedBody(&chassisConfig);
        }

        JPH::Body* chassisBody = static_cast<JoltScene*>(previewScene)->GetJoltBody(chassisHandle);
        if (chassisBody == nullptr)
        {
            joltSystem->RemoveScene(previewSceneHandle);
            return;
        }

        {
            JoltVehicle vehicle(configuration, static_cast<JoltScene*>(previewScene), chassisBody);
            if (!vehicle.IsValid())
            {
                AZ_Warning("JoltPhysics", false, "Settle preview: the vehicle could not be created; check the "
                    "configuration (wheel and differential indices).");
            }
            else
            {
                // Four simulated seconds settles any sane suspension.
                constexpr float fixedDeltaTime = 1.0f / 60.0f;
                for (int step = 0; step < 240; ++step)
                {
                    previewScene->StartSimulation(fixedDeltaTime);
                    previewScene->FinishSimulation();
                }

                const AZ::Transform inverseWorld = worldTransform.GetInverse();
                if (AzPhysics::SimulatedBody* settledChassis = previewScene->GetSimulatedBodyFromHandle(chassisHandle))
                {
                    m_settledChassisLocal = inverseWorld *
                        AZ::Transform::CreateFromQuaternionAndTranslation(
                            settledChassis->GetOrientation(), settledChassis->GetPosition());
                }

                AZStd::string summary;
                for (AZ::u32 wheelIndex = 0; wheelIndex < vehicle.GetWheelCount(); ++wheelIndex)
                {
                    SettledWheel settled;
                    AZ::Transform wheelWorld = AZ::Transform::CreateIdentity();
                    vehicle.GetWheelTransform(wheelIndex, wheelWorld);
                    settled.m_localTransform = inverseWorld * wheelWorld;
                    settled.m_suspensionLength = vehicle.GetSuspensionLength(wheelIndex);
                    settled.m_onGround = vehicle.IsWheelOnGround(wheelIndex);
                    m_settledWheels.push_back(settled);

                    summary += AZStd::string::format("  wheel %u: suspension %.3f m, %s\n",
                        wheelIndex, settled.m_suspensionLength, settled.m_onGround ? "on ground" : "IN THE AIR");
                }
                m_hasSettlePreview = !m_settledWheels.empty();

                AZ_Printf("JoltPhysics", "Settle preview for '%s' (ground at z=%.2f):\n%s",
                    configuration.m_debugName.c_str(), groundZ, summary.c_str());
            }
            // The vehicle's destructor must run while the preview scene still exists.
        }

        joltSystem->RemoveScene(previewSceneHandle);
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

        // The settle-preview ghost: the simulated rest pose, following the entity.
        if (m_hasSettlePreview)
        {
            for (size_t wheelIndex = 0; wheelIndex < m_settledWheels.size(); ++wheelIndex)
            {
                const SettledWheel& settled = m_settledWheels[wheelIndex];
                const float radius = wheelIndex < m_configuration.m_wheels.size()
                    ? m_configuration.m_wheels[wheelIndex].m_radius
                    : 0.35f;
                const float width = wheelIndex < m_configuration.m_wheels.size()
                    ? m_configuration.m_wheels[wheelIndex].m_width
                    : 0.25f;

                // A wheel that never found ground is the thing this preview exists to
                // catch; paint it with the limit color instead of the ghost color.
                debugDisplay.SetColor(settled.m_onGround ? AZ::Color(0.3f, 0.9f, 1.0f, 1.0f)
                                                         : AZ::Color(1.0f, 0.3f, 0.2f, 1.0f));
                const AZ::Transform ghostTransform = chassisTransform * settled.m_localTransform;
                for (const float side : { -0.5f, 0.5f })
                {
                    EditorColliderDraw::DrawWireCircle(debugDisplay, ghostTransform, radius, /*axis*/ 1, side * width);
                }
            }

            // The settled chassis frame, as a small axis cross.
            const AZ::Transform settledChassis = chassisTransform * m_settledChassisLocal;
            debugDisplay.SetColor(AZ::Color(0.3f, 0.9f, 1.0f, 1.0f));
            debugDisplay.DrawLine(
                settledChassis.TransformPoint(AZ::Vector3(-0.4f, 0.0f, 0.0f)),
                settledChassis.TransformPoint(AZ::Vector3(0.4f, 0.0f, 0.0f)));
            debugDisplay.DrawLine(
                settledChassis.TransformPoint(AZ::Vector3(0.0f, -0.4f, 0.0f)),
                settledChassis.TransformPoint(AZ::Vector3(0.0f, 0.4f, 0.0f)));
        }
    }

} // namespace JoltPhysics
