#pragma once

#include <AzFramework/Physics/PhysicsScene.h>

namespace JPH
{
    class PhysicsSystem;
}

namespace JoltPhysics
{
    class JoltScene;

    class JoltSceneQueryHelpers
    {
    public:
        static bool QueryScene(
            JoltScene* scene,
            const AzPhysics::SceneQueryRequest* request,
            AzPhysics::SceneQueryHits& result);

        static bool Raycast(
            JoltScene* scene,
            const AzPhysics::RayCastRequest& request,
            AzPhysics::SceneQueryHits& result);

        static bool ShapeCast(
            JoltScene* scene,
            const AzPhysics::ShapeCastRequest& request,
            AzPhysics::SceneQueryHits& result);

        static bool Overlap(
            JoltScene* scene,
            const AzPhysics::OverlapRequest& request,
            AzPhysics::SceneQueryHits& result);
    };

} // namespace JoltPhysics
