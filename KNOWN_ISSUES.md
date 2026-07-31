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
- **No edit-mode collider bodies.** The editor world exists (see resolved entries), but
  editor collider components only draw wireframes — they do not create collider bodies
  in it (PhysX's `CreateStaticEditorCollider` path), so editor-time physics queries
  cannot hit them directly.
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
  the scene is disabled during play-in-editor and re-enabled on stop. Like PhysX,
  nothing ticks it by default.
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
