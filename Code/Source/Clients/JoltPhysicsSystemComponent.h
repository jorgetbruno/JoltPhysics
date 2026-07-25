#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <AzCore/Interface/Interface.h>

#include <AzFramework/Physics/SystemBus.h>
#include <AzFramework/Physics/CollisionBus.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Physics/Configuration/CollisionConfiguration.h>

#include <JoltPhysics/JoltPhysicsBus.h>
#include <JoltPhysics/Configuration/JoltConfiguration.h>

#include <Clients/DefaultWorldComponent.h>

namespace JoltPhysics
{
    class JoltSystem;

    class JoltPhysicsSystemComponent
        : public AZ::Component
        , protected Physics::SystemRequestBus::Handler
        , protected Physics::CollisionRequestBus::Handler
        , protected Physics::SystemDebugRequestBus::Handler
        , protected JoltPhysicsSystemRequestBus::Handler
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(JoltPhysicsSystemComponent, "{3E5F7A9B-1C3D-5E7F-9A1B-3C5D7E9F1A2B}");
        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        JoltPhysicsSystemComponent() = default;
        ~JoltPhysicsSystemComponent() override = default;

    protected:
        void Init() override;
        void Activate() override;
        void Deactivate() override;

        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        int GetTickOrder() override;

        AZStd::shared_ptr<Physics::Shape> CreateShape(
            const Physics::ColliderConfiguration& colliderConfiguration,
            const Physics::ShapeConfiguration& configuration) override;

        void ReleaseNativeMeshObject(void* nativeMeshObject) override;
        void ReleaseNativeHeightfieldObject(void* nativeHeightfieldObject) override;

        bool CookConvexMeshToFile(
            const AZStd::string& filePath,
            const AZ::Vector3* vertices,
            AZ::u32 vertexCount) override;

        bool CookConvexMeshToMemory(
            const AZ::Vector3* vertices,
            AZ::u32 vertexCount,
            AZStd::vector<AZ::u8>& result) override;

        bool CookTriangleMeshToFile(
            const AZStd::string& filePath,
            const AZ::Vector3* vertices,
            AZ::u32 vertexCount,
            const AZ::u32* indices,
            AZ::u32 indexCount) override;

        bool CookTriangleMeshToMemory(
            const AZ::Vector3* vertices,
            AZ::u32 vertexCount,
            const AZ::u32* indices,
            AZ::u32 indexCount,
            AZStd::vector<AZ::u8>& result) override;

        AzPhysics::CollisionLayer GetCollisionLayerByName(const AZStd::string& layerName) override;
        AZStd::string GetCollisionLayerName(const AzPhysics::CollisionLayer& layer) override;
        bool TryGetCollisionLayerByName(const AZStd::string& layerName, AzPhysics::CollisionLayer& layer) override;

        AzPhysics::CollisionGroup GetCollisionGroupByName(const AZStd::string& groupName) override;
        bool TryGetCollisionGroupByName(const AZStd::string& groupName, AzPhysics::CollisionGroup& group) override;
        AZStd::string GetCollisionGroupName(const AzPhysics::CollisionGroup& group) override;

        AzPhysics::CollisionGroup GetCollisionGroupById(const AzPhysics::CollisionGroups::Id& groupId) override;

        void SetCollisionLayerName(int index, const AZStd::string& layerName) override;
        void CreateCollisionGroup(const AZStd::string& groupName, const AzPhysics::CollisionGroup& group) override;

        bool ShouldCollide(
            const Physics::ColliderConfiguration& colliderConfigurationA,
            const Physics::ColliderConfiguration& colliderConfigurationB) override;

        // Physics::SystemDebugRequestBus
        void DebugDrawPhysics(const Physics::DebugDrawSettings& settings) override;

        // JoltPhysicsSystemRequestBus
        JPH::PhysicsSystem* GetNativePhysicsSystem(AzPhysics::SceneHandle sceneHandle) override;

    private:
        void EnablePhysics();
        void DisablePhysics();
        void ActivatePhysicsSimulation();

        //! Draws all collider shapes via AzFramework's debug display (jolt_Debug cvar).
        void DrawColliderShapes();

        bool m_enabled = false;
        JoltSystem* m_physicsSystem = nullptr;
        JoltDefaultWorldComponent m_defaultWorldComponent;
    };

} // namespace JoltPhysics
