# Known Issues

Tracked per milestone process: anything broken, stubbed, or knowingly incomplete,
with the milestone expected to address it. See DIVERGENCES.md for intentional
deviations from PhysX behavior.

## Remaining gaps (scheduled)

- **`AzPhysics::SceneInterface` is not implemented.** The high-level scene-management
  API and scene-level events (`OnSceneTriggersEvent`, `OnSceneCollisionsEvent`,
  simulation start/finish notifications) are unavailable. Body-level events work.
  → M3 or dedicated follow-up.
- **Mesh colliders / convex hulls unavailable.** `CookConvexMeshToFile/Memory`,
  `CookTriangleMeshToFile/Memory` are stubs returning false; `SystemRequestBus::CreateShape`
  returns `nullptr`; `AddShape`/`RemoveShape` on rigid bodies are no-ops (needs the
  `Physics::Shape` wrapper). → M3 (compound) / later.
- **Multiple colliders per entity** are disallowed at the component level
  (`JoltColliderService` self-incompatible). Compound collider components arrive in M3.
- **No vehicles, soft bodies, water** — scheduled M7–M8.
- **Joints do not disable collision between the connected bodies** (PhysX disables it
  by default): jointed bodies whose shapes overlap will fight the constraint — use
  collision layers/groups or keep the shapes apart. `AzPhysics::JointHelpersInterface`
  (editor joint-limit visualization/auto-configuration) is not implemented.
- **Character controller gaps**: no collision layer/group filtering on the character's
  own movement (defaults are used), `AttachShape` is a no-op, body-level `RayCast`
  returns empty, and there is no `CharacterGameplayComponent` equivalent (gameplay
  drives via `CharacterRequestBus`).
- **No editor (edit-mode) world.** Only the game default world exists
  (`JoltDefaultWorldComponent`); edit-mode simulation and edit-mode scene queries
  are unavailable. PhysX implements this via `EditorWorldBus` in its editor gem.
- **`Physics::Shape`-based collider debug draw and edit-time collider visualization
  are not implemented** (no component modes, no viewport collider rendering yet;
  `DebugDrawPhysics` works for the simulation backend).

## Build / Tooling

- Gem registers via `external_subdirectories` (O3DE 26.05 manifest behavior), not the
  legacy `gems` list — expected, not a bug.
- Stale engine entries in `C:\Users\jorge\.o3de\o3de_manifest.json` (25.05, 24.09.2,
  25.10.x) print `Invalid engine json` warnings from `o3de.bat`; harmless noise.

## Resolved in M2 (kept for reference)

- Physics materials: `JoltMaterial`/`JoltMaterialManager` registered on
  `AZ::Interface<Physics::MaterialManager>`; friction/restitution resolved from
  collider material slots at body creation.
- Collision filtering: per-collider layer/group honored via `AzPhysicsGroupFilter`
  (group masks) + layer index in the Jolt collision subgroup id. Query-side
  filtering by collision group and static/dynamic type works.
- Triggers: `m_isTrigger` produces Jolt sensors; enter/exit events delivered via
  `SimulatedBody::ProcessTriggerEvent`.
- Scene queries: raycast (complete hits: handle, entity, normal), shapecast
  (with MTD recovery), overlap (one hit per body).
- Rigid body: kinematic targets, CCD toggle, mass/inertia getters and overrides,
  center-of-mass offset (Jolt semantics — see DIVERGENCES.md), simulation
  enable/disable.
- `FindAttachedBodyHandleFromEntityId` implemented.
- Debug draw via `Physics::SystemDebugRequestBus::DebugDrawPhysics`.
- Cylinder collider removal (engine dropped `CylinderShapeConfiguration` in 26.05).
