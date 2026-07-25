#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/std/containers/vector.h>

#include <AzFramework/Physics/Common/PhysicsTypes.h>

#include <SoftBody/JoltSoftBody.h>
#include <JoltPhysics/JoltSoftBodyBus.h>

namespace JoltPhysics
{
    //! Turns the entity into a Jolt soft body: cloth, a wobbling cube or a pressurised
    //! balloon. The body is built at the entity's transform when the component activates.
    class JoltSoftBodyComponent
        : public AZ::Component
        , private AZ::TransformNotificationBus::Handler
        , private AZ::TickBus::Handler
        , private JoltSoftBodyRequestBus::Handler
    {
    public:
        AZ_COMPONENT(JoltSoftBodyComponent, "{3F8A1C5D-7B2E-4A69-8D1F-5C3A7B2E4A69}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        JoltSoftBodySettings& GetSettings()
        {
            return m_settings;
        }
        bool& GetVisible()
        {
            return m_visible;
        }

    protected:
        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // AZ::TransformNotificationBus
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        // AZ::TickBus
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // JoltSoftBodyRequestBus
        void SetPressure(float pressure) override;
        float GetPressure() const override;
        void SetLinearDamping(float damping) override;
        float GetLinearDamping() const override;
        void SetGravityFactor(float factor) override;
        float GetGravityFactor() const override;
        void SetNumIterations(AZ::u32 iterations) override;
        AZ::u32 GetNumIterations() const override;
        void SetCollisionLayer(const AzPhysics::CollisionLayer& layer) override;
        AzPhysics::CollisionLayer GetCollisionLayer() const override;
        void SetCollisionGroupId(const AzPhysics::CollisionGroups::Id& groupId) override;
        AzPhysics::CollisionGroups::Id GetCollisionGroupId() const override;
        void SetEnabled(bool enabled) override;
        bool IsEnabled() const override;
        AZ::u32 GetVertexCount() const override;
        AZ::Aabb GetWorldBounds() const override;
        AZ::Vector3 GetVertexPosition(AZ::u32 index) const override;

    private:
        JoltSoftBodySettings m_settings;
        bool m_visible = true;

        JoltSoftBody m_softBody;
        AzPhysics::SceneHandle m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
        bool m_enabled = true;

        //! Reused every frame so drawing a soft body does not allocate per tick.
        AZStd::vector<AZ::Vector3> m_vertexPositionCache;
    };
} // namespace JoltPhysics
