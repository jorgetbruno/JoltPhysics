#include <Utils/JoltComponentUtils.h>
#include <Clients/Components/JoltJointComponentBase.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/SystemBus.h>

#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void JoltJointComponentConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltJointComponentConfiguration>()
                ->Version(1)
                ->Field("LeadEntity", &JoltJointComponentConfiguration::m_leadEntity)
                ->Field("FollowerEntity", &JoltJointComponentConfiguration::m_followerEntity)
                ->Field("LocalTransformFromFollower", &JoltJointComponentConfiguration::m_localTransformFromFollower)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltJointComponentConfiguration>("Jolt Joint Configuration", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltJointComponentConfiguration::m_leadEntity,
                        "Lead entity", "Entity holding the parent body of the joint.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltJointComponentConfiguration::m_followerEntity,
                        "Follower entity", "Entity holding the child body (this entity when left invalid).")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltJointComponentConfiguration::m_localTransformFromFollower,
                        "Joint frame", "Joint frame in the follower entity's local space.")
                    ;
            }
        }
    }

    void JoltJointComponentBase::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<JoltJointComponentConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            if (serializeContext->FindClassData(azrtti_typeid<JoltJointComponentBase>()) != nullptr)
            {
                return;
            }

            serializeContext->Class<JoltJointComponentBase, AZ::Component>()
                ->Version(1)
                ->Field("Configuration", &JoltJointComponentBase::m_configuration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltJointComponentBase>("Jolt Joint Base", "Base configuration for Jolt joints")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltJointComponentBase::m_configuration,
                        "Joint Configuration", "Lead/follower entities and joint frame")
                    ;
            }
        }
    }

    void JoltJointComponentBase::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltJointService"));
    }

    void JoltJointComponentBase::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltJointService"));
    }

    void JoltJointComponentBase::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void JoltJointComponentBase::Activate()
    {
        JoltJointRequestBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();
    }

    void JoltJointComponentBase::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        JoltJointRequestBus::Handler::BusDisconnect();

        DestroyJoint();

        m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
    }

    void JoltJointComponentBase::WarnSingleAxisUnsupported(const char* requestName) const
    {
        AZ_WarningOnce("JoltPhysics", false,
            "JoltJointRequestBus::%s is only supported on the hinge and prismatic joints, which have a single "
            "driven axis. This joint type ignores it.",
            requestName);
    }

    void JoltJointComponentBase::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_jointHandle == AzPhysics::InvalidJointHandle)
        {
            // Bodies on either entity may not exist yet (activation order); keep retrying.
            CreateJoint();
        }
        else
        {
            AZ::TickBus::Handler::BusDisconnect();
        }
    }

    AZ::Transform JoltJointComponentBase::GetJointWorldTransform() const
    {
        AZ::Transform followerWorld = AZ::Transform::CreateIdentity();
        const AZ::EntityId followerEntity = m_configuration.m_followerEntity.IsValid()
            ? m_configuration.m_followerEntity
            : GetEntityId();
        AZ::TransformBus::EventResult(followerWorld, followerEntity, &AZ::TransformBus::Events::GetWorldTM);
        return followerWorld * m_configuration.m_localTransformFromFollower;
    }

    void JoltJointComponentBase::CreateJoint()
    {
        const AZ::EntityId followerEntity = m_configuration.m_followerEntity.IsValid()
            ? m_configuration.m_followerEntity
            : GetEntityId();

        if (!m_configuration.m_leadEntity.IsValid())
        {
            return;
        }

        AzPhysics::SimulatedBodyHandle parentHandle;
        AzPhysics::SimulatedBodyComponentRequestsBus::EventResult(
            parentHandle, m_configuration.m_leadEntity, &AzPhysics::SimulatedBodyComponentRequestsBus::Events::GetSimulatedBodyHandle);
        AzPhysics::SimulatedBodyHandle childHandle;
        AzPhysics::SimulatedBodyComponentRequestsBus::EventResult(
            childHandle, followerEntity, &AzPhysics::SimulatedBodyComponentRequestsBus::Events::GetSimulatedBodyHandle);

        if (parentHandle == AzPhysics::InvalidSimulatedBodyHandle || childHandle == AzPhysics::InvalidSimulatedBodyHandle)
        {
            return;
        }

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

        // Compute the joint frames (mirrors the PhysX gem's JointComponent math).
        const AZ::Transform jointWorld = GetJointWorldTransform();
        const AZ::Transform childLocal = m_configuration.m_localTransformFromFollower;

        AZ::Transform leadWorld = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(leadWorld, m_configuration.m_leadEntity, &AZ::TransformBus::Events::GetWorldTM);
        const AZ::Transform parentLocal = leadWorld.GetInverse() * jointWorld;

        AZStd::unique_ptr<AzPhysics::JointConfiguration> jointConfiguration = BuildJointConfiguration();
        if (!jointConfiguration)
        {
            AZ::TickBus::Handler::BusDisconnect();
            return;
        }

        jointConfiguration->m_parentLocalPosition = parentLocal.GetTranslation();
        jointConfiguration->m_parentLocalRotation = parentLocal.GetRotation();
        jointConfiguration->m_childLocalPosition = childLocal.GetTranslation();
        jointConfiguration->m_childLocalRotation = childLocal.GetRotation();
        jointConfiguration->m_debugName = GetEntity()->GetName();

        m_jointHandle = scene->AddJoint(jointConfiguration.get(), parentHandle, childHandle);
    }

    void JoltJointComponentBase::DestroyJoint()
    {
        if (m_jointHandle != AzPhysics::InvalidJointHandle)
        {
            if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
            {
                if (AzPhysics::Scene* scene = physicsSystem->GetScene(m_attachedSceneHandle))
                {
                    scene->RemoveJoint(m_jointHandle);
                }
            }
            m_jointHandle = AzPhysics::InvalidJointHandle;
        }
    }

    JPH::Constraint* JoltJointComponentBase::GetNativeConstraint() const
    {
        if (m_jointHandle == AzPhysics::InvalidJointHandle)
        {
            return nullptr;
        }
        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            if (AzPhysics::Scene* scene = physicsSystem->GetScene(m_attachedSceneHandle))
            {
                if (AzPhysics::Joint* joint = scene->GetJointFromHandle(m_jointHandle))
                {
                    return static_cast<JPH::Constraint*>(joint->GetNativePointer());
                }
            }
        }
        return nullptr;
    }

    AZ::Transform JoltJointComponentBase::GetTransform() const
    {
        return GetJointWorldTransform();
    }


    void JoltJointComponentBase::OnAfterEntitySet()
    {
        if (m_serializedIdentifier.empty())
        {
            m_serializedIdentifier = Internal::GenerateSerializedIdentifier(this);
        }
    }

    AZStd::string JoltJointComponentBase::GetSerializedIdentifier() const
    {
        return m_serializedIdentifier;
    }
} // namespace JoltPhysics
