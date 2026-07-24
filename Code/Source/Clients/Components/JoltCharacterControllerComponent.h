#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/TransformBus.h>

#include <AzFramework/Physics/Character.h>
#include <AzFramework/Physics/CharacterBus.h>
#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>

#include <JoltPhysics/JoltCharacterGameplayBus.h>

namespace JoltPhysics
{
    class JoltCharacter;

    //! Character controller component: physically represents a character in the Jolt
    //! scene (capsule/box shape via JPH::CharacterVirtual). Mirrors the PhysX gem's
    //! CharacterControllerComponent buses so gameplay code works unchanged.
    class JoltCharacterControllerComponent
        : public AZ::Component
        , private AZ::EntityBus::Handler
        , private AZ::TickBus::Handler
        , private AZ::TransformNotificationBus::Handler
        , private Physics::CharacterRequestBus::Handler
        , private AzPhysics::SimulatedBodyComponentRequestsBus::Handler
        , private JoltCharacterGameplayRequestBus::Handler
    {
    public:
        AZ_COMPONENT(JoltCharacterControllerComponent, "{F3A5C810-7E2A-4B1C-9D4E-A6B7C8D9E0F1}");

        static void Reflect(AZ::ReflectContext* context);

        //! Serialized identifier for the DPE inspector (empty on plain AZ::Component).
        AZStd::string GetSerializedIdentifier() const override;

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        Physics::CharacterConfiguration& GetCharacterConfiguration()
        {
            return m_characterConfig;
        }
        const Physics::CharacterConfiguration& GetCharacterConfiguration() const
        {
            return m_characterConfig;
        }
        AZStd::shared_ptr<Physics::ShapeConfiguration>& GetShapeConfiguration()
        {
            return m_shapeConfig;
        }
        const AZStd::shared_ptr<Physics::ShapeConfiguration>& GetShapeConfiguration() const
        {
            return m_shapeConfig;
        }
        float& GetHeight() { return m_height; }
        float GetHeight() const { return m_height; }
        float& GetRadius() { return m_radius; }
        float GetRadius() const { return m_radius; }

    protected:
        // AZ::Component
        void OnAfterEntitySet() override;
        void Activate() override;
        void Deactivate() override;

        // AZ::EntityBus
        void OnEntityActivated(const AZ::EntityId& entityId) override;

        // AZ::TickBus
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        int GetTickOrder() override;

        // AZ::TransformNotificationBus
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        // Physics::CharacterRequestBus
        AZ::Vector3 GetBasePosition() const override;
        void SetBasePosition(const AZ::Vector3& position) override;
        AZ::Vector3 GetCenterPosition() const override;
        float GetStepHeight() const override;
        void SetStepHeight(float stepHeight) override;
        AZ::Vector3 GetUpDirection() const override;
        void SetUpDirection(const AZ::Vector3& upDirection) override;
        float GetSlopeLimitDegrees() const override;
        void SetSlopeLimitDegrees(float slopeLimitDegrees) override;
        float GetMaximumSpeed() const override;
        void SetMaximumSpeed(float maximumSpeed) override;
        AZ::Vector3 GetVelocity() const override;
        void AddVelocityForTick(const AZ::Vector3& velocity) override;
        void AddVelocityForPhysicsTimestep(const AZ::Vector3& velocity) override;
        bool IsPresent() const override;
        Physics::Character* GetCharacter() override;

        // JoltCharacterGameplayRequestBus
        bool IsOnGround() const override;
        AZ::Vector3 GetGroundNormal() const override;

        // AzPhysics::SimulatedBodyComponentRequestsBus
        void EnablePhysics() override;
        void DisablePhysics() override;
        bool IsPhysicsEnabled() const override;
        AZ::Aabb GetAabb() const override;
        AzPhysics::SimulatedBody* GetSimulatedBody() override;
        AzPhysics::SimulatedBodyHandle GetSimulatedBodyHandle() const override;
        AzPhysics::SceneQueryHit RayCast(const AzPhysics::RayCastRequest& request) override;

    private:
        void CreateCharacter();
        void DestroyCharacter();
        void TryCreateCharacter();

        Physics::CharacterConfiguration m_characterConfig;
        AZStd::shared_ptr<Physics::ShapeConfiguration> m_shapeConfig;

        // Capsule dimensions edited in the inspector. Defaults match JoltCharacter's
        // fallback capsule (1.8 m tall, 0.3 m radius). m_shapeConfig, if set
        // programmatically, takes precedence over these.
        float m_height = 1.8f;
        float m_radius = 0.3f;

        AZStd::string m_serializedIdentifier;

        AzPhysics::SimulatedBodyHandle m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        AzPhysics::SceneHandle m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
        bool m_syncingTransformFromCharacter = false;
    };
} // namespace JoltPhysics
