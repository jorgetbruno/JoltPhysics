/*
 * A measurement, not a feature: how much main-thread time is recoverable by adding a batch of
 * static bodies the way Jolt asks to be asked, rather than one at a time.
 *
 * The gem currently creates every static body inside entity activation, one call each -
 * JoltStaticRigidBody::CreateInScene -> BodyInterface::CreateAndAddBody. Jolt's own header says
 * not to: "Note that if you need to add multiple bodies, use the AddBodiesPrepare/AddBodiesFinalize
 * function." (BodyInterface.h:96). AddBodiesPrepare builds the broadphase sub-tree for the whole
 * batch and is documented as safe to run on a background thread without touching the
 * PhysicsSystem; AddBodiesFinalize splices that prepared tree in and is the only part that must
 * happen on the simulation thread.
 *
 * Measured on the Phoenix city: bringing a live entity into the world costs 0.371 ms, of which
 * removing physics entirely accounts for 0.111 ms - about 930 ms of main thread for one dense
 * tile. This tells us how much of that 0.111 the batch API can actually move off the frame, which
 * is the difference between a worthwhile change and a rewrite that buys nothing.
 *
 * (JoltScene exposes no OptimizeBroadPhase at all, so the post-burst rebuild cannot even be
 * timed from outside the scene - which is itself the answer to whether it is ever called.)
 *
 *   jolt_BenchBodyAdd <count> [mode]
 *     mode 0  one at a time, CreateAndAddBody          (what the gem does today)
 *     mode 1  CreateBody loop, then Prepare + Finalize (what it could do)
 *
 * Bodies are trivial unit boxes: the point is the cost of insertion, not of shape building, which
 * is a separate saving the shape-sharing work already covers.
 *
 * ---------------------------------------------------------------------------------------------
 * FINDINGS (2026-09-07, profile build, run in game mode on the SmokeBox level)
 *
 *   bodies   one at a time     batched: create   prepare   finalize
 *    2,000        0.6 ms              0.2 ms     0.2 ms     0.0 ms
 *   10,000        3.1 ms              1.5 ms     1.2 ms     0.1 ms
 *
 * The batch API does what its header promises: for 10,000 bodies it leaves 0.1 ms on the
 * simulation thread against 3.1 ms, so ~97% of insertion becomes movable. The total barely
 * changes - this is about *where* the time is spent, not how much there is.
 *
 * And there is not much. Insertion costs 0.3 us per body. That is the whole prize, and it is
 * far too small to be the 0.111 ms per entity measured on the city. So the premise this file
 * was written to test - that the batch API is where the activation cost hides - is wrong.
 *
 * Where it actually goes, measured over 4,000 static bodies through the gem's own creation path
 * (JoltScene::AddSimulatedBody, which is what entity activation drives):
 *
 *   whole AddSimulatedBody      21.4 us/body
 *     Physics::Shape wrapper     4.0      one per collider
 *     native Jolt shape          0.8
 *     Jolt insertion             0.3      <- all the batch API can recover
 *     object layer lookup       0.03
 *     material lookup           0.03
 *     unattributed             ~13        inside body construction and scene bookkeeping
 *
 * So the batch rewrite buys ~1.5% of body creation, and body creation is itself under a fifth
 * of the 0.111 ms. Roughly 0.3% of what was measured on the city. Not worth doing.
 *
 * The 4 us Physics::Shape wrapper and the ~13 us unattributed are where the next measurement
 * should go, and the remaining ~90 us of the 0.111 ms is not physics body creation at all -
 * it is component activation, transform plumbing and asset work.
 *
 * (One real saving did come out of running this: resolving every material slot at body creation
 * was allocating for single-slot colliders, 0.8 us each, and now does not.)
 * ---------------------------------------------------------------------------------------------
 */

#include <AzCore/Console/IConsole.h>
#include <AzCore/Console/ILogger.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/std/chrono/chrono.h>
#include <AzCore/std/containers/vector.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/PhysicsSystem.h>

#include <Scene/JoltScene.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

namespace JoltPhysics
{
    namespace
    {
        double MillisecondsSince(AZStd::chrono::steady_clock::time_point from)
        {
            return AZStd::chrono::duration<double, AZStd::milli>(AZStd::chrono::steady_clock::now() - from).count();
        }

        JoltScene* FindDefaultScene()
        {
            auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
            auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
            if (physicsSystem == nullptr || sceneInterface == nullptr)
            {
                return nullptr;
            }
            const AzPhysics::SceneHandle handle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
            return handle != AzPhysics::InvalidSceneHandle
                ? azdynamic_cast<JoltScene*>(sceneInterface->GetScene(handle))
                : nullptr;
        }

        void BenchBodyAdd(const AZ::ConsoleCommandContainer& arguments)
        {
            AZLOG_INFO("JoltBench: entered, %zu args", arguments.size());
            int count = 2000;
            int mode = 0;
            if (!arguments.empty())
            {
                count = atoi(AZStd::string(arguments[0]).c_str());
            }
            if (arguments.size() > 1)
            {
                mode = atoi(AZStd::string(arguments[1]).c_str());
            }
            if (count <= 0)
            {
                count = 2000;
            }

            JoltScene* scene = FindDefaultScene();
            if (scene == nullptr || scene->GetBodyInterface() == nullptr)
            {
                AZLOG_INFO("no default physics scene\n");
                return;
            }
            JPH::BodyInterface& bodyInterface = *scene->GetBodyInterface();

            // One shape, shared by every body: shape construction is not what is being measured,
            // and sharing it here mirrors what the gem already does per mesh asset.
            JPH::RefConst<JPH::Shape> shape = new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f));

            auto settingsFor = [&shape](int index)
            {
                // Spread them out so the broadphase has real work to do rather than a
                // degenerate pile of coincident boxes.
                const float x = static_cast<float>(index % 100) * 3.0f;
                const float y = static_cast<float>((index / 100) % 100) * 3.0f;
                const float z = -500.0f - static_cast<float>(index / 10000);
                JPH::BodyCreationSettings settings(
                    shape, JPH::RVec3(x, y, z), JPH::Quat::sIdentity(), JPH::EMotionType::Static, 0);
                return settings;
            };

            if (mode == 0)
            {
                AZStd::vector<JPH::BodyID> created;
                created.reserve(count);
                const auto start = AZStd::chrono::steady_clock::now();
                for (int index = 0; index < count; ++index)
                {
                    created.push_back(bodyInterface.CreateAndAddBody(settingsFor(index), JPH::EActivation::DontActivate));
                }
                const double totalMs = MillisecondsSince(start);

                AZLOG_INFO("=== one at a time (CreateAndAddBody), %d bodies ===\n", count);
                AZLOG_INFO("  total %.1f ms   per body %.4f ms   ALL on the calling thread\n",
                    totalMs, totalMs / count);

                const auto cleanupStart = AZStd::chrono::steady_clock::now();
                bodyInterface.RemoveBodies(created.data(), static_cast<int>(created.size()));
                bodyInterface.DestroyBodies(created.data(), static_cast<int>(created.size()));
                AZLOG_INFO("  (cleanup %.1f ms)\n", MillisecondsSince(cleanupStart));
                return;
            }

            // Split path. Create and Prepare are the parts a worker thread could own; only
            // Finalize has to happen where the simulation lives.
            AZStd::vector<JPH::BodyID> ids;
            ids.reserve(count);

            const auto createStart = AZStd::chrono::steady_clock::now();
            for (int index = 0; index < count; ++index)
            {
                JPH::Body* body = bodyInterface.CreateBody(settingsFor(index));
                if (body != nullptr)
                {
                    ids.push_back(body->GetID());
                }
            }
            const double createMs = MillisecondsSince(createStart);

            const auto prepareStart = AZStd::chrono::steady_clock::now();
            JPH::BodyInterface::AddState addState =
                bodyInterface.AddBodiesPrepare(ids.data(), static_cast<int>(ids.size()));
            const double prepareMs = MillisecondsSince(prepareStart);

            const auto finalizeStart = AZStd::chrono::steady_clock::now();
            bodyInterface.AddBodiesFinalize(
                ids.data(), static_cast<int>(ids.size()), addState, JPH::EActivation::DontActivate);
            const double finalizeMs = MillisecondsSince(finalizeStart);

            AZLOG_INFO("=== batched (CreateBody + Prepare + Finalize), %d bodies ===\n", count);
            AZLOG_INFO("  create   %8.1f ms  (%.4f/body)  <- worker thread\n", createMs, createMs / count);
            AZLOG_INFO("  prepare  %8.1f ms  (%.4f/body)  <- worker thread\n", prepareMs, prepareMs / count);
            AZLOG_INFO("  FINALIZE %8.1f ms  (%.4f/body)  <- main thread, the only unavoidable part\n",
                finalizeMs, finalizeMs / count);
            AZLOG_INFO("  total    %8.1f ms\n", createMs + prepareMs + finalizeMs);

            const auto cleanupStart = AZStd::chrono::steady_clock::now();
            bodyInterface.RemoveBodies(ids.data(), static_cast<int>(ids.size()));
            bodyInterface.DestroyBodies(ids.data(), static_cast<int>(ids.size()));
            AZLOG_INFO("  (cleanup %.1f ms)\n", MillisecondsSince(cleanupStart));
        }

    } // namespace

    AZ_CONSOLEFREEFUNC("jolt_BenchBodyAdd", BenchBodyAdd, AZ::ConsoleFunctorFlags::DontReplicate,
        "Compare per-body adds against Prepare/Finalize batching: jolt_BenchBodyAdd <count> [mode 0|1]");

    void EnsureBodyBatchBenchmarkLinked()
    {
        // Anchor. The console functors above register through static initialisation, and this
        // file lives in a static library, so without a referenced symbol the linker discards the
        // whole object file and the commands quietly do not exist.
    }
} // namespace JoltPhysics
