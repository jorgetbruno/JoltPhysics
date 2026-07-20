#pragma once

#include <AzFramework/Physics/PhysicsScene.h>

namespace JPH
{
    class PhysicsSystem;
}

namespace JoltPhysics
{
    class JoltSceneQueryHelpers
    {
    public:
        static bool QueryScene(
            JPH::PhysicsSystem* physicsSystem,
            const AzPhysics::SceneQueryRequest* request,
            AzPhysics::SceneQueryHits& result);

        static bool Raycast(
            JPH::PhysicsSystem* physicsSystem,
            const AzPhysics::RayCastRequest& request,
            AzPhysics::SceneQueryHits& result);

        static bool ShapeCast(
            JPH::PhysicsSystem* physicsSystem,
            const AzPhysics::ShapeCastRequest& request,
            AzPhysics::SceneQueryHits& result);

        static bool Overlap(
            JPH::PhysicsSystem* physicsSystem,
            const AzPhysics::OverlapRequest& request,
            AzPhysics::SceneQueryHits& result);
    };

} // namespace JoltPhysics
