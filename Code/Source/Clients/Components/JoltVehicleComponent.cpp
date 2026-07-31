#include <Utils/JoltComponentUtils.h>
#include <Clients/Components/JoltVehicleComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/SystemBus.h>

#include <Scene/JoltScene.h>
#include <Utils/ReflectionUtils.h>
#include <Vehicle/JoltVehicle.h>

#include <Utils/JoltDiagnostics.h>

namespace JoltPhysics
{
    JoltVehicleComponent::~JoltVehicleComponent()
    {
        DestroyVehicle();
    }

    void JoltVehicleComponent::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<JoltWheelConfiguration>(context);
        Internal::ReflectOnce<JoltVehicleConfiguration>(context);

        Internal::ReflectEBusOnce(context, "JoltVehicleRequestBus",
            [](AZ::BehaviorContext* behaviorContext)
            {
                behaviorContext->EBus<JoltVehicleRequestBus>("JoltVehicleRequestBus")
                    ->Attribute(AZ::Script::Attributes::Category, "Jolt Physics")
                    ->Event("SetDriverInput", &JoltVehicleRequests::SetDriverInput)
                    ->Event("SetForwardInput", &JoltVehicleRequests::SetForwardInput)
                    ->Event("SetSteeringInput", &JoltVehicleRequests::SetSteeringInput)
                    ->Event("SetBrakeInput", &JoltVehicleRequests::SetBrakeInput)
                    ->Event("SetHandBrakeInput", &JoltVehicleRequests::SetHandBrakeInput)
                    ->Event("GetSpeed", &JoltVehicleRequests::GetSpeed)
                    ->Event("GetEngineRpm", &JoltVehicleRequests::GetEngineRpm)
                    ->Event("GetCurrentGear", &JoltVehicleRequests::GetCurrentGear)
                    ->Event("SetGear", &JoltVehicleRequests::SetGear)
                    ->Event("SetTransmissionAutomatic", &JoltVehicleRequests::SetTransmissionAutomatic)
                    ->Event("IsTransmissionAutomatic", &JoltVehicleRequests::IsTransmissionAutomatic)
                    ->Event("GetLeanAngle", &JoltVehicleRequests::GetLeanAngle)
                    ->Event("SetLeanControllerEnabled", &JoltVehicleRequests::SetLeanControllerEnabled)
                    ->Event("SetLeanSteeringLimitEnabled", &JoltVehicleRequests::SetLeanSteeringLimitEnabled)
                    // The wheel accessors are what drives the visual wheels, which is
                    // exactly the job a script is likely to be doing here.
                    ->Event("GetWheelCount", &JoltVehicleRequests::GetWheelCount)
                    ->Event("GetWheelTransform", &JoltVehicleRequests::GetWheelTransform)
                    ->Event("GetSuspensionLength", &JoltVehicleRequests::GetSuspensionLength)
                    ->Event("IsWheelOnGround", &JoltVehicleRequests::IsWheelOnGround)
                    // The slip/contact readouts are the tire-smoke and skid-audio signals.
                    ->Event("GetWheelAngularVelocity", &JoltVehicleRequests::GetWheelAngularVelocity)
                    ->Event("GetWheelSteerAngle", &JoltVehicleRequests::GetWheelSteerAngle)
                    ->Event("GetWheelLongitudinalSlip", &JoltVehicleRequests::GetWheelLongitudinalSlip)
                    ->Event("GetWheelLateralSlip", &JoltVehicleRequests::GetWheelLateralSlip)
                    ->Event("GetWheelContactPoint", &JoltVehicleRequests::GetWheelContactPoint)
                    ->Event("GetWheelContactNormal", &JoltVehicleRequests::GetWheelContactNormal)
                    ->Event("IsWheelSuspensionBottomedOut", &JoltVehicleRequests::IsWheelSuspensionBottomedOut)
                    ->Event("RecreateVehicle", &JoltVehicleRequests::RecreateVehicle)
                    ;
            });

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltVehicleComponent, AZ::Component>()
                ->Version(1)
                ->Field("VehicleConfiguration", &JoltVehicleComponent::m_configuration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltVehicleComponent>(
                    "Jolt Vehicle",
                    "Wheeled, motorcycle or tracked vehicle simulated by the Jolt physics backend")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        // No AppearsInAddComponentMenu: EditorJoltVehicleComponent owns the
                        // menu entry (PhysX-style editor/runtime split). The runtime component
                        // stays registered for old prefabs and BuildGameEntity.
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::RemoveableByUser, true)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltVehicleComponent::m_configuration,
                        "Vehicle Configuration", "Wheels, engine and transmission settings")
                    ;
            }
        }
    }

    void JoltVehicleComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltVehicleService"));
    }

    void JoltVehicleComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltVehicleService"));
    }

    void JoltVehicleComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        required.push_back(AZ_CRC_CE("JoltRigidBodyService"));
    }

    void JoltVehicleComponent::Activate()
    {
        JoltVehicleRequestBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();
    }

    void JoltVehicleComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        JoltVehicleRequestBus::Handler::BusDisconnect();

        DestroyVehicle();

        m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
    }

    void JoltVehicleComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (!m_vehicle)
        {
            CreateVehicle();
        }
        else
        {
            AZ::TickBus::Handler::BusDisconnect();
        }
    }

    void JoltVehicleComponent::CreateVehicle()
    {
        if (m_attachedSceneHandle == AzPhysics::InvalidSceneHandle)
        {
            Physics::DefaultWorldBus::BroadcastResult(m_attachedSceneHandle, &Physics::DefaultWorldRequests::GetDefaultSceneHandle);
        }

        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AzPhysics::Scene* scene = physicsSystem ? physicsSystem->GetScene(m_attachedSceneHandle) : nullptr;
        if (!scene)
        {
            return;
        }

        // The chassis rigid body must exist first (it is created after entity activation).
        AzPhysics::SimulatedBodyHandle chassisHandle;
        AzPhysics::SimulatedBodyComponentRequestsBus::EventResult(
            chassisHandle, GetEntityId(), &AzPhysics::SimulatedBodyComponentRequestsBus::Events::GetSimulatedBodyHandle);
        if (chassisHandle == AzPhysics::InvalidSimulatedBodyHandle)
        {
            return;
        }

        auto* joltScene = static_cast<JoltScene*>(scene);
        JPH::Body* chassisBody = joltScene->GetJoltBody(chassisHandle);
        if (!chassisBody)
        {
            return;
        }

        // Name the vehicle for its diagnostics; the configuration carries it no further.
        m_configuration.m_debugName = GetEntity() ? GetEntity()->GetName() : AZStd::string();

        m_vehicle = aznew JoltVehicle(m_configuration, joltScene, chassisBody);
        if (!m_vehicle->IsValid())
        {
            DestroyVehicle();
        }
    }

    void JoltVehicleComponent::DestroyVehicle()
    {
        delete m_vehicle;
        m_vehicle = nullptr;
    }

    void JoltVehicleComponent::SetDriverInput(float forward, float right, float brake, float handbrake)
    {
        if (m_vehicle)
        {
            m_vehicle->SetDriverInput(forward, right, brake, handbrake);
        }
    }

    void JoltVehicleComponent::SetForwardInput(float forward)
    {
        if (m_vehicle)
        {
            m_vehicle->SetForwardInput(forward);
        }
    }

    void JoltVehicleComponent::SetSteeringInput(float right)
    {
        if (m_vehicle)
        {
            m_vehicle->SetSteeringInput(right);
        }
    }

    void JoltVehicleComponent::SetBrakeInput(float brake)
    {
        if (m_vehicle)
        {
            m_vehicle->SetBrakeInput(brake);
        }
    }

    void JoltVehicleComponent::SetHandBrakeInput(float handbrake)
    {
        if (m_vehicle)
        {
            m_vehicle->SetHandBrakeInput(handbrake);
        }
    }

    void JoltVehicleComponent::SetGear(int gear)
    {
        if (m_vehicle)
        {
            m_vehicle->SetGear(gear);
        }
    }

    void JoltVehicleComponent::SetTransmissionAutomatic(bool automatic)
    {
        if (m_vehicle)
        {
            m_vehicle->SetTransmissionAutomatic(automatic);
        }
    }

    bool JoltVehicleComponent::IsTransmissionAutomatic() const
    {
        return m_vehicle ? m_vehicle->IsTransmissionAutomatic() : true;
    }

    void JoltVehicleComponent::SetLeanControllerEnabled(bool enabled)
    {
        if (m_vehicle)
        {
            m_vehicle->SetLeanControllerEnabled(enabled);
        }
    }

    void JoltVehicleComponent::SetLeanSteeringLimitEnabled(bool enabled)
    {
        if (m_vehicle)
        {
            m_vehicle->SetLeanSteeringLimitEnabled(enabled);
        }
    }

    float JoltVehicleComponent::GetWheelAngularVelocity(AZ::u32 wheelIndex) const
    {
        return m_vehicle ? m_vehicle->GetWheelAngularVelocity(wheelIndex) : 0.0f;
    }

    float JoltVehicleComponent::GetWheelSteerAngle(AZ::u32 wheelIndex) const
    {
        return m_vehicle ? m_vehicle->GetWheelSteerAngle(wheelIndex) : 0.0f;
    }

    float JoltVehicleComponent::GetWheelLongitudinalSlip(AZ::u32 wheelIndex) const
    {
        return m_vehicle ? m_vehicle->GetWheelLongitudinalSlip(wheelIndex) : 0.0f;
    }

    float JoltVehicleComponent::GetWheelLateralSlip(AZ::u32 wheelIndex) const
    {
        return m_vehicle ? m_vehicle->GetWheelLateralSlip(wheelIndex) : 0.0f;
    }

    AZ::Vector3 JoltVehicleComponent::GetWheelContactPoint(AZ::u32 wheelIndex) const
    {
        AZ::Vector3 point = AZ::Vector3::CreateZero();
        if (m_vehicle)
        {
            m_vehicle->GetWheelContactPoint(wheelIndex, point);
        }
        return point;
    }

    AZ::Vector3 JoltVehicleComponent::GetWheelContactNormal(AZ::u32 wheelIndex) const
    {
        AZ::Vector3 normal = AZ::Vector3::CreateZero();
        if (m_vehicle)
        {
            m_vehicle->GetWheelContactNormal(wheelIndex, normal);
        }
        return normal;
    }

    bool JoltVehicleComponent::IsWheelSuspensionBottomedOut(AZ::u32 wheelIndex) const
    {
        return m_vehicle ? m_vehicle->IsWheelSuspensionBottomedOut(wheelIndex) : false;
    }

    void JoltVehicleComponent::RecreateVehicle()
    {
        // Jolt bakes the configuration into the constraint at creation, so runtime
        // config edits only land through a rebuild. Destroy now and let the tick
        // recreate exactly like first activation (the chassis body must still exist).
        DestroyVehicle();
        if (!AZ::TickBus::Handler::BusIsConnected())
        {
            AZ::TickBus::Handler::BusConnect();
        }
    }

    float JoltVehicleComponent::GetSpeed() const
    {
        return m_vehicle ? m_vehicle->GetSpeed() : 0.0f;
    }

    float JoltVehicleComponent::GetEngineRpm() const
    {
        return m_vehicle ? m_vehicle->GetEngineRpm() : 0.0f;
    }

    int JoltVehicleComponent::GetCurrentGear() const
    {
        return m_vehicle ? m_vehicle->GetCurrentGear() : 0;
    }

    float JoltVehicleComponent::GetLeanAngle() const
    {
        return m_vehicle ? m_vehicle->GetLeanAngle() : 0.0f;
    }

    AZ::u32 JoltVehicleComponent::GetWheelCount() const
    {
        return m_vehicle ? m_vehicle->GetWheelCount() : 0;
    }

    AZ::Transform JoltVehicleComponent::GetWheelTransform(AZ::u32 wheelIndex) const
    {
        AZ::Transform wheelTransform = AZ::Transform::CreateIdentity();
        if (m_vehicle)
        {
            m_vehicle->GetWheelTransform(wheelIndex, wheelTransform);
        }
        return wheelTransform;
    }

    float JoltVehicleComponent::GetSuspensionLength(AZ::u32 wheelIndex) const
    {
        return m_vehicle ? m_vehicle->GetSuspensionLength(wheelIndex) : 0.0f;
    }

    bool JoltVehicleComponent::IsWheelOnGround(AZ::u32 wheelIndex) const
    {
        return m_vehicle && m_vehicle->IsWheelOnGround(wheelIndex);
    }


    void JoltVehicleComponent::OnAfterEntitySet()
    {
        if (m_serializedIdentifier.empty())
        {
            m_serializedIdentifier = Internal::GenerateSerializedIdentifier(this);
        }
    }

    AZStd::string JoltVehicleComponent::GetSerializedIdentifier() const
    {
        return m_serializedIdentifier;
    }
} // namespace JoltPhysics
