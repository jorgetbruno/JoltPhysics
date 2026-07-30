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
                    ->Event("GetSpeed", &JoltVehicleRequests::GetSpeed)
                    ->Event("GetEngineRpm", &JoltVehicleRequests::GetEngineRpm)
                    ->Event("GetCurrentGear", &JoltVehicleRequests::GetCurrentGear)
                    ->Event("GetLeanAngle", &JoltVehicleRequests::GetLeanAngle)
                    // The wheel accessors are what drives the visual wheels, which is
                    // exactly the job a script is likely to be doing here.
                    ->Event("GetWheelCount", &JoltVehicleRequests::GetWheelCount)
                    ->Event("GetWheelTransform", &JoltVehicleRequests::GetWheelTransform)
                    ->Event("GetSuspensionLength", &JoltVehicleRequests::GetSuspensionLength)
                    ->Event("IsWheelOnGround", &JoltVehicleRequests::IsWheelOnGround)
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
                    "Wheeled vehicle simulated by the Jolt physics backend")
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
