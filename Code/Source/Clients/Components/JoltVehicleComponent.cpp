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
                    ->Event("OverrideVehicleGravity", &JoltVehicleRequests::OverrideVehicleGravity)
                    ->Event("ResetVehicleGravityOverride", &JoltVehicleRequests::ResetVehicleGravityOverride)
                    ->Event("RecreateVehicle", &JoltVehicleRequests::RecreateVehicle)
                    // The configuration round-trips by value: read, edit fields, write
                    // back, then RecreateVehicle to apply.
                    ->Event("GetVehicleConfiguration", &JoltVehicleRequests::GetVehicleConfiguration)
                    ->Event("SetVehicleConfiguration", &JoltVehicleRequests::SetVehicleConfiguration)
                    ;

                // The configuration classes, so script can author a drivetrain and not
                // just drive it. Guarded by the bus registration above: bus and classes
                // always register together, exactly once.
                behaviorContext->Class<JoltWheelConfiguration>("JoltWheelConfiguration")
                    ->Attribute(AZ::Script::Attributes::Category, "Jolt Physics")
                    ->Property("Position", BehaviorValueProperty(&JoltWheelConfiguration::m_position))
                    ->Property("Radius", BehaviorValueProperty(&JoltWheelConfiguration::m_radius))
                    ->Property("Width", BehaviorValueProperty(&JoltWheelConfiguration::m_width))
                    ->Property("SuspensionMinLength", BehaviorValueProperty(&JoltWheelConfiguration::m_suspensionMinLength))
                    ->Property("SuspensionMaxLength", BehaviorValueProperty(&JoltWheelConfiguration::m_suspensionMaxLength))
                    ->Property("SuspensionPreloadLength", BehaviorValueProperty(&JoltWheelConfiguration::m_suspensionPreloadLength))
                    ->Property("SuspensionFrequency", BehaviorValueProperty(&JoltWheelConfiguration::m_suspensionFrequency))
                    ->Property("SuspensionDamping", BehaviorValueProperty(&JoltWheelConfiguration::m_suspensionDamping))
                    ->Property("Inertia", BehaviorValueProperty(&JoltWheelConfiguration::m_inertia))
                    ->Property("AngularDamping", BehaviorValueProperty(&JoltWheelConfiguration::m_angularDamping))
                    ->Property("MaxSteerAngleDegrees", BehaviorValueProperty(&JoltWheelConfiguration::m_maxSteerAngleDegrees))
                    ->Property("MaxBrakeTorque", BehaviorValueProperty(&JoltWheelConfiguration::m_maxBrakeTorque))
                    ->Property("MaxHandBrakeTorque", BehaviorValueProperty(&JoltWheelConfiguration::m_maxHandBrakeTorque))
                    ->Property("TrackedLongitudinalFriction", BehaviorValueProperty(&JoltWheelConfiguration::m_trackedLongitudinalFriction))
                    ->Property("TrackedLateralFriction", BehaviorValueProperty(&JoltWheelConfiguration::m_trackedLateralFriction))
                    // The tyre curves: the gem's own header calls these the single biggest
                    // handling knob a vehicle has, and they were the one part of the
                    // configuration a script could read back but never author.
                    ->Property("LongitudinalFrictionCurve", BehaviorValueProperty(&JoltWheelConfiguration::m_longitudinalFrictionCurve))
                    ->Property("LateralFrictionCurve", BehaviorValueProperty(&JoltWheelConfiguration::m_lateralFrictionCurve))
                    ->Property("SuspensionSpringMode",
                        [](const JoltWheelConfiguration* wheel) { return static_cast<int>(wheel->m_suspensionSpringMode); },
                        [](JoltWheelConfiguration* wheel, int value) { wheel->m_suspensionSpringMode = static_cast<JoltSuspensionSpringMode>(value); })
                    ->Property("SuspensionForcePoint", BehaviorValueProperty(&JoltWheelConfiguration::m_suspensionForcePoint))
                    ->Property("EnableSuspensionForcePoint", BehaviorValueProperty(&JoltWheelConfiguration::m_enableSuspensionForcePoint))
                    ;

                behaviorContext->Class<JoltVehicleAntiRollBar>("JoltVehicleAntiRollBar")
                    ->Attribute(AZ::Script::Attributes::Category, "Jolt Physics")
                    ->Property("LeftWheel", BehaviorValueProperty(&JoltVehicleAntiRollBar::m_leftWheel))
                    ->Property("RightWheel", BehaviorValueProperty(&JoltVehicleAntiRollBar::m_rightWheel))
                    ->Property("Stiffness", BehaviorValueProperty(&JoltVehicleAntiRollBar::m_stiffness))
                    ;

                behaviorContext->Class<JoltVehicleDifferential>("JoltVehicleDifferential")
                    ->Attribute(AZ::Script::Attributes::Category, "Jolt Physics")
                    ->Property("LeftWheel", BehaviorValueProperty(&JoltVehicleDifferential::m_leftWheel))
                    ->Property("RightWheel", BehaviorValueProperty(&JoltVehicleDifferential::m_rightWheel))
                    ->Property("DifferentialRatio", BehaviorValueProperty(&JoltVehicleDifferential::m_differentialRatio))
                    ->Property("LeftRightSplit", BehaviorValueProperty(&JoltVehicleDifferential::m_leftRightSplit))
                    ->Property("LimitedSlipRatio", BehaviorValueProperty(&JoltVehicleDifferential::m_limitedSlipRatio))
                    ->Property("EngineTorqueRatio", BehaviorValueProperty(&JoltVehicleDifferential::m_engineTorqueRatio))
                    ;

                behaviorContext->Class<JoltVehicleConfiguration>("JoltVehicleConfiguration")
                    ->Attribute(AZ::Script::Attributes::Category, "Jolt Physics")
                    // The enums cross into script as plain numbers (0 = Wheeled/Automatic...),
                    // matching how the serialized data stores them.
                    ->Property("VehicleType",
                        [](const JoltVehicleConfiguration* config) { return static_cast<int>(config->m_vehicleType); },
                        [](JoltVehicleConfiguration* config, int value) { config->m_vehicleType = static_cast<JoltVehicleType>(value); })
                    ->Property("CollisionTester",
                        [](const JoltVehicleConfiguration* config) { return static_cast<int>(config->m_collisionTester); },
                        [](JoltVehicleConfiguration* config, int value) { config->m_collisionTester = static_cast<JoltVehicleCollisionTester>(value); })
                    ->Property("TransmissionMode",
                        [](const JoltVehicleConfiguration* config) { return static_cast<int>(config->m_transmissionMode); },
                        [](JoltVehicleConfiguration* config, int value) { config->m_transmissionMode = static_cast<JoltVehicleTransmissionMode>(value); })
                    ->Property("EngineTorqueCurve", BehaviorValueProperty(&JoltVehicleConfiguration::m_engineTorqueCurve))
                    ->Property("Wheels", BehaviorValueProperty(&JoltVehicleConfiguration::m_wheels))
                    ->Property("AntiRollBars", BehaviorValueProperty(&JoltVehicleConfiguration::m_antiRollBars))
                    ->Property("Differentials", BehaviorValueProperty(&JoltVehicleConfiguration::m_differentials))
                    ->Property("DifferentialLimitedSlipRatio", BehaviorValueProperty(&JoltVehicleConfiguration::m_differentialLimitedSlipRatio))
                    ->Property("MaxPitchRollAngleDegrees", BehaviorValueProperty(&JoltVehicleConfiguration::m_maxPitchRollAngleDegrees))
                    ->Property("ChassisMass", BehaviorValueProperty(&JoltVehicleConfiguration::m_chassisMass))
                    ->Property("MaxEngineTorque", BehaviorValueProperty(&JoltVehicleConfiguration::m_maxEngineTorque))
                    ->Property("MaxEngineRpm", BehaviorValueProperty(&JoltVehicleConfiguration::m_maxEngineRpm))
                    ->Property("MinEngineRpm", BehaviorValueProperty(&JoltVehicleConfiguration::m_minEngineRpm))
                    ->Property("EngineInertia", BehaviorValueProperty(&JoltVehicleConfiguration::m_engineInertia))
                    ->Property("EngineAngularDamping", BehaviorValueProperty(&JoltVehicleConfiguration::m_engineAngularDamping))
                    ->Property("GearRatios", BehaviorValueProperty(&JoltVehicleConfiguration::m_gearRatios))
                    ->Property("ReverseGearRatio", BehaviorValueProperty(&JoltVehicleConfiguration::m_reverseGearRatio))
                    ->Property("GearSwitchTime", BehaviorValueProperty(&JoltVehicleConfiguration::m_gearSwitchTime))
                    ->Property("ClutchReleaseTime", BehaviorValueProperty(&JoltVehicleConfiguration::m_clutchReleaseTime))
                    ->Property("GearSwitchLatency", BehaviorValueProperty(&JoltVehicleConfiguration::m_gearSwitchLatency))
                    ->Property("ShiftUpRpm", BehaviorValueProperty(&JoltVehicleConfiguration::m_shiftUpRpm))
                    ->Property("ShiftDownRpm", BehaviorValueProperty(&JoltVehicleConfiguration::m_shiftDownRpm))
                    ->Property("ClutchStrength", BehaviorValueProperty(&JoltVehicleConfiguration::m_clutchStrength))
                    ->Property("MaxLeanAngleDegrees", BehaviorValueProperty(&JoltVehicleConfiguration::m_maxLeanAngleDegrees))
                    ->Property("LeanSpringConstant", BehaviorValueProperty(&JoltVehicleConfiguration::m_leanSpringConstant))
                    ->Property("LeanSpringDamping", BehaviorValueProperty(&JoltVehicleConfiguration::m_leanSpringDamping))
                    ->Property("LeanSpringIntegrationCoefficient", BehaviorValueProperty(&JoltVehicleConfiguration::m_leanSpringIntegrationCoefficient))
                    ->Property("LeanSpringIntegrationCoefficientDecay", BehaviorValueProperty(&JoltVehicleConfiguration::m_leanSpringIntegrationCoefficientDecay))
                    ->Property("LeanSmoothingFactor", BehaviorValueProperty(&JoltVehicleConfiguration::m_leanSmoothingFactor))
                    ->Property("TrackInertia", BehaviorValueProperty(&JoltVehicleConfiguration::m_trackInertia))
                    ->Property("TrackAngularDamping", BehaviorValueProperty(&JoltVehicleConfiguration::m_trackAngularDamping))
                    ->Property("TrackMaxBrakeTorque", BehaviorValueProperty(&JoltVehicleConfiguration::m_trackMaxBrakeTorque))
                    ->Property("TrackDifferentialRatio", BehaviorValueProperty(&JoltVehicleConfiguration::m_trackDifferentialRatio))
                    ->Property("LeftTrackDrivenWheel", BehaviorValueProperty(&JoltVehicleConfiguration::m_leftTrackDrivenWheel))
                    ->Property("RightTrackDrivenWheel", BehaviorValueProperty(&JoltVehicleConfiguration::m_rightTrackDrivenWheel))
                    ->Property("NumVelocityStepsOverride", BehaviorValueProperty(&JoltVehicleConfiguration::m_numVelocityStepsOverride))
                    ->Property("NumPositionStepsOverride", BehaviorValueProperty(&JoltVehicleConfiguration::m_numPositionStepsOverride))
                    ->Property("CollisionTestStepsActive", BehaviorValueProperty(&JoltVehicleConfiguration::m_collisionTestStepsActive))
                    ->Property("CollisionTestStepsInactive", BehaviorValueProperty(&JoltVehicleConfiguration::m_collisionTestStepsInactive))
                    ;
            });

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            // The friction and torque curves are AZStd::vector<AZ::Vector2>; the generic
            // type has to be registered for script to marshal one across.
            serializeContext->RegisterGenericType<AZStd::vector<AZ::Vector2>>();

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

    JoltVehicleConfiguration JoltVehicleComponent::GetVehicleConfiguration() const
    {
        return m_configuration;
    }

    void JoltVehicleComponent::SetVehicleConfiguration(const JoltVehicleConfiguration& configuration)
    {
        // The debug name is derived, not authored; keep it through a script write.
        const AZStd::string debugName = m_configuration.m_debugName;
        m_configuration = configuration;
        m_configuration.m_debugName = debugName;
    }

    void JoltVehicleComponent::OverrideVehicleGravity(const AZ::Vector3& gravity)
    {
        if (m_vehicle)
        {
            m_vehicle->OverrideGravity(gravity);
        }
    }

    void JoltVehicleComponent::ResetVehicleGravityOverride()
    {
        if (m_vehicle)
        {
            m_vehicle->ResetGravityOverride();
        }
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
