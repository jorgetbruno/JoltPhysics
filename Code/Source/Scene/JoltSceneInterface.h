#pragma once

#include <AzCore/Interface/Interface.h>
#include <AzFramework/Physics/PhysicsScene.h>

namespace JoltPhysics
{
    class JoltSystem;

    //! Implements the engine-wide AzPhysics::SceneInterface by forwarding every call to the
    //! AzPhysics::Scene already looked up from JoltSystem via the given SceneHandle.
    //! JoltScene (see Scene/JoltScene.h) already implements the full AzPhysics::Scene
    //! interface and is what every call here ultimately delegates to - this class exists
    //! only so that engine/gem code written against the generic multi-scene
    //! AzPhysics::SceneInterface (e.g. the WhiteBox gem) works without needing to know
    //! about Jolt specifically.
    class JoltSceneInterface
        : public AZ::Interface<AzPhysics::SceneInterface>::Registrar
    {
    public:
        AZ_RTTI(JoltSceneInterface, "{6B4B8E2A-3F1D-4E9C-8A5B-2D7C6E1F9A3B}", AzPhysics::SceneInterface);

        void Initialize(JoltSystem* system);
        void Shutdown();

        // AzPhysics::SceneInterface overrides
        AzPhysics::SceneHandle GetSceneHandle(const AZStd::string& sceneName) override;
        AzPhysics::Scene* GetScene(AzPhysics::SceneHandle handle) override;

        void StartSimulation(AzPhysics::SceneHandle sceneHandle, float deltatime) override;
        void FinishSimulation(AzPhysics::SceneHandle sceneHandle) override;
        void SetEnabled(AzPhysics::SceneHandle sceneHandle, bool enable) override;
        bool IsEnabled(AzPhysics::SceneHandle sceneHandle) const override;

        AzPhysics::SimulatedBodyHandle AddSimulatedBody(
            AzPhysics::SceneHandle sceneHandle, const AzPhysics::SimulatedBodyConfiguration* simulatedBodyConfig) override;
        AzPhysics::SimulatedBodyHandleList AddSimulatedBodies(
            AzPhysics::SceneHandle sceneHandle, const AzPhysics::SimulatedBodyConfigurationList& simulatedBodyConfigs) override;
        AzPhysics::SimulatedBody* GetSimulatedBodyFromHandle(
            AzPhysics::SceneHandle sceneHandle, AzPhysics::SimulatedBodyHandle bodyHandle) override;
        AzPhysics::SimulatedBodyList GetSimulatedBodiesFromHandle(
            AzPhysics::SceneHandle sceneHandle, const AzPhysics::SimulatedBodyHandleList& bodyHandles) override;
        void RemoveSimulatedBody(AzPhysics::SceneHandle sceneHandle, AzPhysics::SimulatedBodyHandle& bodyHandle) override;
        void RemoveSimulatedBodies(AzPhysics::SceneHandle sceneHandle, AzPhysics::SimulatedBodyHandleList& bodyHandles) override;
        void EnableSimulationOfBody(AzPhysics::SceneHandle sceneHandle, AzPhysics::SimulatedBodyHandle bodyHandle) override;
        void DisableSimulationOfBody(AzPhysics::SceneHandle sceneHandle, AzPhysics::SimulatedBodyHandle bodyHandle) override;

        AzPhysics::JointHandle AddJoint(
            AzPhysics::SceneHandle sceneHandle,
            const AzPhysics::JointConfiguration* jointConfig,
            AzPhysics::SimulatedBodyHandle parentBody,
            AzPhysics::SimulatedBodyHandle childBody) override;
        AzPhysics::Joint* GetJointFromHandle(AzPhysics::SceneHandle sceneHandle, AzPhysics::JointHandle jointHandle) override;
        void RemoveJoint(AzPhysics::SceneHandle sceneHandle, AzPhysics::JointHandle jointHandle) override;

        AzPhysics::SceneQueryHits QueryScene(AzPhysics::SceneHandle sceneHandle, const AzPhysics::SceneQueryRequest* request) override;
        bool QueryScene(
            AzPhysics::SceneHandle sceneHandle, const AzPhysics::SceneQueryRequest* request, AzPhysics::SceneQueryHits& result) override;
        AzPhysics::SceneQueryHitsList QuerySceneBatch(
            AzPhysics::SceneHandle sceneHandle, const AzPhysics::SceneQueryRequests& requests) override;
        bool QuerySceneAsync(
            AzPhysics::SceneHandle sceneHandle,
            AzPhysics::SceneQuery::AsyncRequestId requestId,
            const AzPhysics::SceneQueryRequest* request,
            AzPhysics::SceneQuery::AsyncCallback callback) override;
        bool QuerySceneAsyncBatch(
            AzPhysics::SceneHandle sceneHandle,
            AzPhysics::SceneQuery::AsyncRequestId requestId,
            const AzPhysics::SceneQueryRequests& requests,
            AzPhysics::SceneQuery::AsyncBatchCallback callback) override;

        void SuppressCollisionEvents(
            AzPhysics::SceneHandle sceneHandle,
            const AzPhysics::SimulatedBodyHandle& bodyHandleA,
            const AzPhysics::SimulatedBodyHandle& bodyHandleB) override;
        void UnsuppressCollisionEvents(
            AzPhysics::SceneHandle sceneHandle,
            const AzPhysics::SimulatedBodyHandle& bodyHandleA,
            const AzPhysics::SimulatedBodyHandle& bodyHandleB) override;

        void SetGravity(AzPhysics::SceneHandle sceneHandle, const AZ::Vector3& gravity) override;
        AZ::Vector3 GetGravity(AzPhysics::SceneHandle sceneHandle) const override;

        void RegisterSceneConfigurationChangedEventHandler(
            AzPhysics::SceneHandle sceneHandle, AzPhysics::SceneEvents::OnSceneConfigurationChanged::Handler& handler) override;
        void RegisterSimulationBodyAddedHandler(
            AzPhysics::SceneHandle sceneHandle, AzPhysics::SceneEvents::OnSimulationBodyAdded::Handler& handler) override;
        void RegisterSimulationBodyRemovedHandler(
            AzPhysics::SceneHandle sceneHandle, AzPhysics::SceneEvents::OnSimulationBodyRemoved::Handler& handler) override;
        void RegisterSimulationBodySimulationEnabledHandler(
            AzPhysics::SceneHandle sceneHandle, AzPhysics::SceneEvents::OnSimulationBodySimulationEnabled::Handler& handler) override;
        void RegisterSimulationBodySimulationDisabledHandler(
            AzPhysics::SceneHandle sceneHandle, AzPhysics::SceneEvents::OnSimulationBodySimulationDisabled::Handler& handler) override;
        void RegisterSceneSimulationStartHandler(
            AzPhysics::SceneHandle sceneHandle, AzPhysics::SceneEvents::OnSceneSimulationStartHandler& handler) override;
        void RegisterSceneSimulationFinishHandler(
            AzPhysics::SceneHandle sceneHandle, AzPhysics::SceneEvents::OnSceneSimulationFinishHandler& handler) override;
        void RegisterSceneActiveSimulatedBodiesHandler(
            AzPhysics::SceneHandle sceneHandle, AzPhysics::SceneEvents::OnSceneActiveSimulatedBodiesEvent::Handler& handler) override;
        void RegisterSceneCollisionEventHandler(
            AzPhysics::SceneHandle sceneHandle, AzPhysics::SceneEvents::OnSceneCollisionsEvent::Handler& handler) override;
        void RegisterSceneTriggersEventHandler(
            AzPhysics::SceneHandle sceneHandle, AzPhysics::SceneEvents::OnSceneTriggersEvent::Handler& handler) override;
        void RegisterSceneGravityChangedEvent(
            AzPhysics::SceneHandle sceneHandle, AzPhysics::SceneEvents::OnSceneGravityChangedEvent::Handler& handler) override;

    private:
        JoltSystem* m_system = nullptr;
    };

} // namespace JoltPhysics
