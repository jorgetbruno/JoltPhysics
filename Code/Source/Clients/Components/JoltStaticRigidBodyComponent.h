#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/TransformBus.h>

#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>
#include <AzFramework/Physics/Configuration/StaticRigidBodyConfiguration.h>

namespace JoltPhysics
{
    //! Component used to register an entity as a static rigid body in the Jolt simulation.
    class JoltStaticRigidBodyComponent final
        : public AZ::Component
        , public AZ::EntityBus::Handler
        , public AzPhysics::SimulatedBodyComponentRequestsBus::Handler
        , public AZ::TickBus::Handler
        , private AZ::TransformNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(JoltStaticRigidBodyComponent, "{A6B5FF06-CD9F-4A7B-EC8D-AE1F2A3B4C5D}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // AzPhysics::SimulatedBodyComponentRequestsBus
        void EnablePhysics() override;
        void DisablePhysics() override;
        bool IsPhysicsEnabled() const override;

        AzPhysics::SceneQueryHit RayCast(const AzPhysics::RayCastRequest& request) override;
        AZ::Aabb GetAabb() const override;

        AzPhysics::SimulatedBody* GetSimulatedBody() override;
        AzPhysics::SimulatedBodyHandle GetSimulatedBodyHandle() const override;

    protected:
        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // AZ::EntityBus
        void OnEntityActivated(const AZ::EntityId& entityId) override;

        // AZ::TickBus
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        int GetTickOrder() override;

        // AZ::TransformNotificationBus
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

    private:
        void TryCreateRigidBody();
        void CreateRigidBody();
        void DestroyRigidBody();

        AzPhysics::SimulatedBodyHandle m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        AzPhysics::SceneHandle m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
    };
} // namespace JoltPhysics
