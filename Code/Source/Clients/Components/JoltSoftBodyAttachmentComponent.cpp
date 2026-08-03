#include <Clients/Components/JoltSoftBodyAttachmentComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Debug/Profiler.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/limits.h>

#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SystemBus.h>

#include <JoltPhysics/JoltSoftBodyBus.h>

namespace JoltPhysics
{
    namespace
    {
        //! Distance from a point to the shape a target fastens along: its physics body's
        //! bounds if it has one, else the authored attach line, else its position.
        float DistanceToAttachShape(
            const AZ::Vector3& point,
            const AZ::Aabb& targetBounds,
            const AZ::Transform& targetTransform,
            float attachExtent)
        {
            if (targetBounds.IsValid())
            {
                return targetBounds.GetDistance(point);
            }

            if (attachExtent <= 0.0f)
            {
                return point.GetDistance(targetTransform.GetTranslation());
            }

            const AZ::Vector3 lineStart = targetTransform.TransformPoint(AZ::Vector3(-attachExtent, 0.0f, 0.0f));
            const AZ::Vector3 lineEnd = targetTransform.TransformPoint(AZ::Vector3(attachExtent, 0.0f, 0.0f));
            const AZ::Vector3 line = lineEnd - lineStart;
            const float lengthSq = line.GetLengthSq();
            const float t = lengthSq > 0.0f
                ? AZ::GetClamp(line.Dot(point - lineStart) / lengthSq, 0.0f, 1.0f)
                : 0.0f;
            return point.GetDistance(lineStart + line * t);
        }
    } // namespace

    void JoltSoftBodyAttachTarget::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltSoftBodyAttachTarget>()
                ->Version(1)
                ->Field("Entity", &JoltSoftBodyAttachTarget::m_entity)
                ->Field("AttachDistance", &JoltSoftBodyAttachTarget::m_attachDistance)
                ->Field("AttachExtent", &JoltSoftBodyAttachTarget::m_attachExtent)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltSoftBodyAttachTarget>("Attachment", "One place the cloth is fastened")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodyAttachTarget::m_entity,
                        "Entity", "What the cloth is fastened to - a yard, a boom, a pole, a hook. "
                        "Particles near it are welded to it and follow it; the rest stay free cloth.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodyAttachTarget::m_attachDistance,
                        "Distance", "Particles within this of the fastening attach.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.01f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodyAttachTarget::m_attachExtent,
                        "Extent", "Half-length of the fastening along the target's local x axis - the yard a "
                        "sail is bent to, the rail a curtain rides. 0 fastens at a point.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ;
            }
        }
    }

    void JoltSoftBodyAttachmentComponent::Reflect(AZ::ReflectContext* context)
    {
        JoltSoftBodyAttachTarget::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltSoftBodyAttachmentComponent, AZ::Component>()
                ->Version(1)
                ->Field("Targets", &JoltSoftBodyAttachmentComponent::m_targets)
                ->Field("PushEntity", &JoltSoftBodyAttachmentComponent::m_pushEntity)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltSoftBodyAttachmentComponent>(
                    "Jolt Soft Body Attachment", "Fastens this entity's soft body to other entities, so cloth "
                    "can hang off things that move")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodyAttachmentComponent::m_targets,
                        "Fastenings", "Where the cloth is held. Cloth held in only one place flogs - a sail "
                        "wants its head and its foot.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltSoftBodyAttachmentComponent::m_pushEntity,
                        "Push", "Optional: the rigid body that carries what the cloth catches. For a sail, "
                        "the hull - the wind force on the canvas is applied to this body, so the boat is "
                        "driven by the same gust its sail fills with.")
                    ;
            }
        }
    }

    void JoltSoftBodyAttachmentComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltSoftBodyAttachmentService"));
    }

    void JoltSoftBodyAttachmentComponent::GetIncompatibleServices(
        AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        // One per body, because binding replaces the soft body's skinning data wholesale.
        // Several fastenings go in the list rather than in several components.
        incompatible.push_back(AZ_CRC_CE("JoltSoftBodyAttachmentService"));
    }

    void JoltSoftBodyAttachmentComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        // The soft body this rigs. Requiring it means the component cannot sit on an
        // entity where it would do nothing.
        required.push_back(AZ_CRC_CE("JoltSoftBodyService"));
    }

    void JoltSoftBodyAttachmentComponent::Activate()
    {
        Physics::DefaultWorldBus::BroadcastResult(
            m_sceneHandle, &Physics::DefaultWorldRequests::GetDefaultSceneHandle);

        m_sceneStartHandler = AzPhysics::SceneEvents::OnSceneSimulationStartHandler(
            [this]([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, [[maybe_unused]] float fixedDeltaTime)
            {
                DriveAttachment();
            });
        if (auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
            sceneInterface && m_sceneHandle != AzPhysics::InvalidSceneHandle)
        {
            sceneInterface->RegisterSceneSimulationStartHandler(m_sceneHandle, m_sceneStartHandler);
        }

        AZ::TickBus::Handler::BusConnect();
    }

    void JoltSoftBodyAttachmentComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        m_sceneStartHandler.Disconnect();

        // Release the cloth rather than leaving it welded to transforms nothing will
        // update again.
        JoltSoftBodyRequestBus::Event(
            GetEntityId(), &JoltSoftBodyRequests::SetSkinningData, AZStd::vector<AZ::Transform>(),
            AZStd::vector<JoltSoftBodySkinnedVertex>());

        m_attached = false;
        m_sceneHandle = AzPhysics::InvalidSceneHandle;
    }

    void JoltSoftBodyAttachmentComponent::Reattach()
    {
        m_attached = false;
        m_needsHardSkin = false;
    }

    void JoltSoftBodyAttachmentComponent::OnTick(
        [[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (!m_attached)
        {
            // The soft body's particles and the targets' transforms become ready in no
            // fixed order, so this retries rather than failing once at activation.
            TryAttach();
        }
    }

    bool JoltSoftBodyAttachmentComponent::TryAttach()
    {
        AZ_PROFILE_FUNCTION(Physics);

        if (m_targets.empty())
        {
            return false;
        }

        AZStd::vector<AZ::Vector3> particlePositions;
        JoltSoftBodyRequestBus::EventResult(
            particlePositions, GetEntityId(), &JoltSoftBodyRequests::GetVertexPositions);
        if (particlePositions.empty())
        {
            return false; // the soft body has not been built yet
        }

        // One bone per target, in the order they were authored, so the per-step update can
        // hand over transforms without a lookup.
        AZStd::vector<AZ::Transform> bindTransforms;
        AZStd::vector<AZ::Aabb> targetBounds;
        bindTransforms.reserve(m_targets.size());
        targetBounds.reserve(m_targets.size());

        for (const JoltSoftBodyAttachTarget& target : m_targets)
        {
            AZ::Transform transform = AZ::Transform::CreateIdentity();
            AZ::Aabb bounds = AZ::Aabb::CreateNull();
            if (target.m_entity.IsValid())
            {
                AZ::TransformBus::EventResult(transform, target.m_entity, &AZ::TransformBus::Events::GetWorldTM);
                AzPhysics::SimulatedBodyComponentRequestsBus::EventResult(
                    bounds, target.m_entity, &AzPhysics::SimulatedBodyComponentRequests::GetAabb);
            }
            bindTransforms.push_back(transform);
            targetBounds.push_back(bounds);
        }

        AZStd::vector<JoltSoftBodySkinnedVertex> skinnedVertices;
        for (AZ::u32 index = 0; index < particlePositions.size(); ++index)
        {
            // Nearest fastening wins, so a particle within reach of both a yard and a
            // boom follows the one it actually sits on rather than whichever happened to
            // be authored first.
            size_t bestTarget = m_targets.size();
            float bestDistance = AZStd::numeric_limits<float>::max();

            for (size_t targetIndex = 0; targetIndex < m_targets.size(); ++targetIndex)
            {
                if (!m_targets[targetIndex].m_entity.IsValid())
                {
                    continue;
                }

                const float distance = DistanceToAttachShape(
                    particlePositions[index], targetBounds[targetIndex], bindTransforms[targetIndex],
                    m_targets[targetIndex].m_attachExtent);
                if (distance <= m_targets[targetIndex].m_attachDistance && distance < bestDistance)
                {
                    bestDistance = distance;
                    bestTarget = targetIndex;
                }
            }

            if (bestTarget == m_targets.size())
            {
                continue; // free cloth
            }

            JoltSoftBodySkinnedVertex vertex;
            vertex.m_vertexIndex = index;
            vertex.m_influences.push_back({ aznumeric_cast<AZ::u32>(bestTarget), 1.0f });
            // Welded: a sail is laced to its yard, not dangling near it.
            vertex.m_maxDistance = 0.0f;
            skinnedVertices.push_back(AZStd::move(vertex));
        }

        if (skinnedVertices.empty())
        {
            AZ_Warning("JoltPhysics", false,
                "No particle of the soft body on entity %s is within reach of any of its %zu attachment "
                "targets, so nothing attached. Move the cloth onto a target, or raise an attach distance.",
                GetEntityId().ToString().c_str(), m_targets.size());
            // Answered, not retried: the geometry is where it is, and warning once a
            // frame forever helps nobody. Reattach() is the way back in.
            m_attached = true;
            return true;
        }

        JoltSoftBodyRequestBus::Event(
            GetEntityId(), &JoltSoftBodyRequests::SetSkinningData, bindTransforms, skinnedVertices);

        m_attached = true;
        m_needsHardSkin = true;
        return true;
    }

    void JoltSoftBodyAttachmentComponent::DriveAttachment()
    {
        if (!m_attached || m_targets.empty())
        {
            return;
        }

        AZ_PROFILE_FUNCTION(Physics);

        // Where the targets are now, in the order the binding recorded them.
        m_targetTransforms.clear();
        m_targetTransforms.reserve(m_targets.size());
        for (const JoltSoftBodyAttachTarget& target : m_targets)
        {
            AZ::Transform transform = AZ::Transform::CreateIdentity();
            if (target.m_entity.IsValid())
            {
                AZ::TransformBus::EventResult(transform, target.m_entity, &AZ::TransformBus::Events::GetWorldTM);
            }
            m_targetTransforms.push_back(transform);
        }

        bool updated = false;
        JoltSoftBodyRequestBus::EventResult(
            updated, GetEntityId(), &JoltSoftBodyRequests::UpdateSkinnedJoints, m_targetTransforms,
            m_needsHardSkin);
        if (updated)
        {
            m_needsHardSkin = false;
        }

        // The pull back: what the wind put into the canvas this step goes into the hull.
        // Same wind, seen on the sail and felt on the boat.
        if (m_pushEntity.IsValid())
        {
            AZ::Vector3 windImpulse = AZ::Vector3::CreateZero();
            JoltSoftBodyRequestBus::EventResult(
                windImpulse, GetEntityId(), &JoltSoftBodyRequests::GetLastWindImpulse);

            if (!windImpulse.IsClose(AZ::Vector3::CreateZero()))
            {
                Physics::RigidBodyRequestBus::Event(
                    m_pushEntity, &Physics::RigidBodyRequests::ApplyLinearImpulse, windImpulse);
            }
        }
    }
} // namespace JoltPhysics
