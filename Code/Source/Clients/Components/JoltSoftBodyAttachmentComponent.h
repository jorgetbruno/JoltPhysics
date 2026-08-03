#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/RTTI/TypeInfo.h>
#include <AzCore/std/containers/vector.h>

#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>

namespace JoltPhysics
{
    //! One place a soft body is fastened to something.
    struct JoltSoftBodyAttachTarget
    {
        AZ_TYPE_INFO(JoltSoftBodyAttachTarget, "{1F6E9A34-8C25-4B70-9D48-6A3B0E5C7F92}");
        static void Reflect(AZ::ReflectContext* context);

        //! The entity the cloth is fastened to. Its world transform is the one this
        //! target's particles follow; it needs no physics body of its own, so a plain
        //! child entity of a vehicle works.
        AZ::EntityId m_entity;

        //! Particles within this of the fastening weld; the rest stay free cloth.
        float m_attachDistance = 0.3f;

        //! Half-length of the fastening, along the target's local x axis. Cloth is
        //! usually fastened along a *line* - a yard, a rail, a boom - and a target's
        //! position alone welds only whatever sits at its centre point. Zero means a
        //! point, which is right for a hook or a clew. Ignored when the target has a
        //! physics body of its own, whose bounds are used instead.
        float m_attachExtent = 0.0f;
    };

    //! Fastens the soft body on this entity to one or more other entities, so cloth can
    //! hang off things that move: a sail bent to a yard and sheeted to a boom, a banner
    //! between two poles, a tarp roped to a truck bed.
    //!
    //! The pinning presets cannot do this - a pinned particle has infinite mass and
    //! anchors where it was built, in world space, which is right for a curtain rail and
    //! wrong for anything that sails away. This rides the skinning path instead: each
    //! target is one bone, particles near it at bind time are welded to that bone, and
    //! every step feeds each bone its target's current transform. Rig cloth to a skeleton
    //! and the same machinery is JoltCloth; rig it to entities and it is this.
    //!
    //! **Cloth fastened in one place flogs.** A sail bent only along its head thrashes
    //! and averages no thrust at all, exactly as it would in a real gale - which is why
    //! this takes a list rather than a single target. Fasten the head *and* the foot and
    //! the canvas fills. That was not a guess: the demo's first rigged sail flogged for
    //! ten seconds and moved its boat half a metre.
    //!
    //! The other half is the pull back. The wind force the cloth catches each step can be
    //! applied to a rigid body of your choosing, so a boat is driven by the same gust its
    //! sail visibly fills with - one wind, seen and felt - rather than by a second,
    //! invented force that would drift out of agreement with what the canvas is doing.
    class JoltSoftBodyAttachmentComponent
        : public AZ::Component
        , private AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(JoltSoftBodyAttachmentComponent, "{5D2B8F41-9E67-4C30-A18D-7F52E9B4C6A3}");

        static void Reflect(AZ::ReflectContext* context);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        AZStd::vector<JoltSoftBodyAttachTarget>& GetTargets()
        {
            return m_targets;
        }
        AZ::EntityId& GetPushEntity()
        {
            return m_pushEntity;
        }

        bool IsAttached() const
        {
            return m_attached;
        }

        //! Drops the rig so the next tick takes it again - after teleporting a target, or
        //! rebuilding the cloth.
        void Reattach();

    protected:
        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus - only the bind retries here; driving is per physics step.
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

    private:
        //! Welds every particle within reach of a target to it, at the pose everything is
        //! in right now. The soft body and the targets become ready in no fixed order, so
        //! this is retried until it works.
        bool TryAttach();

        //! Per physics step: hands the cloth each target's current transform, and hands
        //! the push body the wind impulse the cloth just caught.
        void DriveAttachment();

        AZStd::vector<JoltSoftBodyAttachTarget> m_targets;

        //! Optional: the rigid body that carries what the cloth catches. For a sail this
        //! is the hull - distinct from the targets on purpose, since the thing the cloth
        //! is laced to (the yard) is rarely the thing the force should move (the boat).
        AZ::EntityId m_pushEntity;

        bool m_attached = false;

        //! First drive after attaching snaps the welded particles exactly onto their
        //! targets - which is also what primes Jolt's skin state. Driving softly from an
        //! unprimed state reads uninitialised positions and the solver NaNs.
        bool m_needsHardSkin = false;

        //! Drives the rig once per fixed step - following is physics, so it must track
        //! steps, not render frames.
        AzPhysics::SceneEvents::OnSceneSimulationStartHandler m_sceneStartHandler;
        AzPhysics::SceneHandle m_sceneHandle = AzPhysics::InvalidSceneHandle;

        //! Scratch reused every step: one transform per target, in target order.
        AZStd::vector<AZ::Transform> m_targetTransforms;
    };
} // namespace JoltPhysics
