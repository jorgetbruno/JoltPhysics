# Known Issues

Tracked per milestone process: anything broken, stubbed, or knowingly incomplete,
with the milestone expected to address it. See DIVERGENCES.md for intentional
deviations from PhysX behavior.

## Remaining gaps (scheduled)

- **`AzPhysics::JointHelpersInterface` is not implemented** (editor joint-limit
  visualization/auto-configuration).
- **Character controller gaps**: `AttachShape` is a no-op (a Jolt character's shape is
  fixed at creation) and there is no `CharacterGameplayComponent` equivalent (gameplay
  drives via `CharacterRequestBus`).
- **Compound and heightfield colliders have no edit-mode bodies.** The primitive and
  mesh editor colliders create static bodies in the editor scene (see resolved
  entries); the compound colliders deliberately do not (their children are separate
  entities with colliders of their own), and the heightfield's geometry lives with the
  terrain provider.
- **Vehicle gaps**: O3DE 26.05 has no AzPhysics vehicle interfaces (the PhysXVehicle
  gem is not part of this engine), so vehicles are exposed only through this gem's own
  component/bus.

## Build / Tooling

- Gem registers via `external_subdirectories` (O3DE 26.05 manifest behavior), not the
  legacy `gems` list — expected, not a bug.
- Stale engine entries in `%USERPROFILE%\.o3de\o3de_manifest.json` (25.05, 24.09.2,
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

## Resolved in later milestones (kept for reference)

- **`AzPhysics::SceneInterface`** is implemented (`JoltSceneInterface`), including
  scene-level trigger/collision events, so engine-wide consumers (e.g. the WhiteBox
  gem) work against it.
- **Editor world**: `EditorWorldBus::GetEditorSceneHandle` returns a real editor
  scene (named `"EditorScene"`, mirroring PhysX): edit-mode scene queries work, and
  the scene is disabled during play-in-editor and re-enabled on stop. Like PhysX, the
  system tick does step it while enabled (`Simulate` steps every enabled scene), but
  it hosts nothing dynamic - it is a query/body host for editor tools. A dynamic body
  added to it would simulate in edit mode.
- **Edit-mode collider bodies**: the primitive (box, sphere, capsule, cylinder), baked
  mesh and mesh asset editor colliders create static bodies in the editor scene -
  PhysX's `CreateStaticEditorCollider` equivalent - so editor-time physics queries hit
  what the viewport shows. Bodies follow entity moves, rebuild on scale or property
  changes and on re-bakes/asset loads, and are removed on deactivate.
- **Joints disable collision between connected bodies** (PhysX default): `AddJoint`
  registers the body pair and `RemoveJoint` drops it; the contact listener rejects
  contact generation for registered pairs (`JoltScene::AreBodiesJointed`, guarded for
  narrow-phase worker access).
- **Mesh colliders / convex hulls**: `CookConvexMeshToFile/Memory` and
  `CookTriangleMeshToFile/Memory` pack the geometry blob (Jolt needs no cooking pass),
  `SystemRequestBus::CreateShape` returns a `JoltShape` wrapper, and
  `JoltRigidBody::AddShape`/`RemoveShape` manage runtime compound shapes. The editor
  mesh collider bakes triangle-mesh or convex-hull collision from the render mesh
  (single hull, hull per mesh node, or VHACD decomposition), and source scenes cook
  into shared `.joltmesh` product assets in the Asset Processor via the Scene Builder
  (see DIVERGENCES.md "Asset pipeline mesh colliders").
- **Multiple colliders per entity** (M3): colliders no longer declare
  `JoltColliderService` self-incompatible; static and mutable compound collider
  components group child colliders into one body.
- **Soft bodies** (M8): `JoltSoftBodyComponent` + `JoltSoftBodyRequestBus` with
  procedural geometry and `AzPhysics::SimulatedBody` integration. Water/buoyancy
  lives in the separate JoltBuoyancy gem.
- **Editor collider visualization and component modes**: viewport wireframes with
  selection bounds for all collider types (box, sphere, capsule, cylinder,
  mesh/convex, heightfield); Box/Capsule component modes plus joint, vehicle, and
  character controller modes.
- **Character controller filtering and queries**: the character's collision
  layer/group is honored for its own movement (Jolt default filters over the
  character's object layer), and body-level `RayCast` casts against the shape at its
  current pose.
- **Vehicle wheel state for rendering**: exposed through `JoltVehicleRequestBus`
  (`GetWheelCount`, `GetWheelTransform` in world space, carrying suspension);
  driving the wheel meshes from it remains the user's side.
- **Vehicle tuning surface**: per-wheel tire friction curves, engine torque curve and
  idle, automatic/manual transmission with shift tuning and `SetGear`, a differential
  list with limited slip (AWD), suspension preload/force point/spring modes, per-axis
  driver input setters, slip/contact wheel readouts, motorcycle lean tuning and
  runtime toggles, authorable tracked driven wheels, `RecreateVehicle` for runtime
  config edits, a C++ combine-friction hook for terrain-dependent grip, and vehicle
  constraint debug draw under `jolt_Debug` (see DIVERGENCES.md, M7/vehicles).
