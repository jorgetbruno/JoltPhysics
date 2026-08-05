# Known Issues

Tracked per milestone process: anything broken, stubbed, or knowingly incomplete,
with the milestone expected to address it. See DIVERGENCES.md for intentional
deviations from PhysX behavior.

## Remaining gaps (scheduled)

- **A collapsed soft body face is still unusable to Jolt, and a shape-overlap query against
  one reports an assertion.** Colliding a convex shape against a soft body walks its faces
  and seeds GJK with each triangle's raw cross product, which Jolt asserts on when that is
  near zero (`JPH_ASSERT(!ioV.IsNearZero())` in `EPAPenetrationDepth.h`, tolerance 1e-12 on
  the squared length). Cloth with little or nothing pinned genuinely does collapse a face
  when something crushes it - measured on an unpinned crest, the smallest face fell from
  1.97e-4 to 8.1e-7 in five frames.
  The two paths that used to reach it are closed: characters no longer collide with soft
  bodies at all (see DIVERGENCES.md, "Characters do not collide with cloth"), and every face
  this gem builds is checked against Jolt's own tolerance before it is handed over. Ordinary
  rigid-body simulation does not walk faces either - the soft body collides its own
  particles against rigid shapes rather than the other way round - which is why crates and
  balls rest on cloth without trouble.
  What remains is a **shape overlap** aimed at a soft body, which goes through the same
  face-walking code. A shape *cast* is safe by contrast: `EPAPenetrationDepth::CastShape`
  seeds GJK with the cast direction and handles a degenerate contact normal explicitly.
  Since the gem's own scene queries pass a default body filter, a caller cannot currently
  exclude soft bodies from an overlap before narrow phase runs.
  Since assertions now report rather than break (see below), the consequence is a logged
  `AZ_Error` and one contact resolved along a meaningless direction, not a dead process.
- **Jolt assertions are reported, not fatal, and are compiled out of release.** Jolt's
  assert callback contract is that returning true triggers a breakpoint. This gem returned
  true, which turned every assertion - including ones about geometry it could recover from -
  into a dead process, in release builds as well, since `USE_ASSERTS` was on for every
  configuration. The callback now returns false and the define is per-configuration. An
  assertion is a real defect worth fixing and it is logged as an error with Jolt's own file
  and line, but it no longer takes the level down with it.
- **Nothing interpolates a body's transform between fixed physics steps.** The simulation
  runs on a fixed-timestep accumulator (`JoltSystem::Simulate`, 1/60 s by default) and a
  rigid body writes its entity transform only on the steps that actually run
  (`JoltRigidBodyComponent`, at `TICK_PHYSICS`). Above the physics rate the frames and the
  steps do not line up, so a moving body advances in a staircase: at 102 FPS against 60 Hz
  physics, about four frames in ten redraw a body at exactly the pose it already had, and
  the rest jump a whole step.
  This is invisible with a fixed camera and obvious the moment anything moves smoothly
  *relative* to the body. A chase camera that eases toward its target - interpolating its
  own position every render frame while the car it follows moves in steps - puts the
  difference dead centre in frame, where it reads as the car shaking. The camera is the
  amplifier, not the cause; a rigid, uninterpolated follow camera moves in the same
  staircase and shows nothing.
  PhysX has the missing piece and this gem does not: `TransformForwardTimeInterpolator`
  (`PhysX/Core/Code/Source/RigidBodyComponent.h`), driven by
  `AzPhysics::RigidBodyConfiguration::m_interpolateMotion`. Note that flag defaults to
  **false**, so interpolation is opt-in there rather than standard. This gem does not read
  the field at all, and does not offer it in the editor - so nothing here claims to
  interpolate and then fails to.
  Until it is implemented, the ways round it are: follow the body without smoothing, so
  the viewer moves in the same steps it does; raise the physics rate (a 1/120 fixed
  timestep halves the step and costs twice the steps); or cap the frame rate to the
  physics rate so the two line up. Implementing it means keeping each body's previous and
  current pose and blending them by the leftover accumulator time when writing the entity
  transform.
- **Compound and heightfield colliders have no edit-mode bodies.** The primitive and
  mesh editor colliders create static bodies in the editor scene (see resolved
  entries); the compound colliders deliberately do not (their children are separate
  entities with colliders of their own), and the heightfield's geometry lives with the
  terrain provider.
- **Vehicle gaps**: O3DE 26.05 has no AzPhysics vehicle interfaces (the PhysXVehicle
  gem is not part of this engine), so vehicles are exposed only through this gem's own
  component/bus.
- **Soft body gaps**: this gem still draws nothing itself — debug-draw plus the bus's
  bulk vertex/triangle reads are all it offers, which is correct for a gem that depends on
  no renderer. A character's cloth is now simulated, skinned and drawn by the **JoltCloth**
  sibling gem, which reads the actor's cloth mesh (weights and painted `CLOTH_DATA` alike),
  hands the geometry back through `SetCustomGeometry`, and writes the simulation into the
  render mesh. What is left here is anything that is *not* a character: a soft body built
  from a `.joltmesh` or a procedural shape has no render mesh to write into, so drawing one
  still means reading its vertices and drawing them yourself.

- **Force region gaps**: the spline-follow force PhysX offers is not wrapped (it needs a
  spline component to follow), there is no editor force-region component (the runtime one
  composes on editor entities because the editor colliders provide `JoltColliderService`,
  which is how the sail demo level authors its wind), and force regions have no editor
  viewport preview of the
  forces they apply.

- **The shape collider does not wrap Quad**, and a **concave** Polygon Prism becomes its
  convex hull rather than its true outline. Box, Sphere, Capsule, Cylinder and convex
  prisms are handled; a concave prism needs the decomposition path the baked mesh collider
  already uses, which is an editor-time bake rather than the live read this does.

- **`jolt_Debug` still re-tessellates every shape each frame.** Primitives are now
  batched by colour into a handful of broadcasts per frame rather than one per triangle,
  which was the dominant cost; the remaining work is Jolt's full `DebugRenderer`
  interface, which caches a shape's geometry once and redraws it by handle.

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

- **`AzPhysics::JointHelpersInterface` and `EditorJointHelpersInterface`** are
  implemented (`JoltJointHelpers`, `JoltEditorJointHelpers`), so the Animation Editor's
  ragdoll joint tools work in a project running Jolt: joint types to author, initial
  limits computed from a bone's rest pose and example rotations, swing-cone and
  twist-arc visualization data, and the limit auto-fit. Only the PhysX gem registers
  these in the shipped engine, and both `EMotionFX.dll` and `AzFramework`'s
  `CharacterPhysicsDebugDraw` reach for them — so a ragdoll this gem could simulate
  could not be authored.
- **Characters fall on their own** — the character controller applies the scene's
  gravity, tracks the falling velocity between frames, and sheds it on landing (see
  DIVERGENCES.md "Character gravity"). PhysX puts this in a separate example
  `CharacterGameplayComponent`; here it is on the controller.
- **`AttachShape` works**, and collider components on a character's entity are gathered
  into `CharacterConfiguration::m_colliders` and attached the same way — both end up on
  a kinematic body that follows the character rather than on the shape it moves with
  (see DIVERGENCES.md "Character attachments").

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
  lives in the separate JoltBuoyancy gem. Later extended with the full creation
  surface (friction, restitution, vertex radius, max particle velocity,
  update-position, double-sided faces, opt-in LRA tethers), bulk vertex/triangle
  reads on the bus, runtime per-particle pinning and velocity, an edit-mode live
  preview simulating in the editor scene, mesh-sourced bodies from `.joltmesh`
  assets (welded), a per-particle contact notification bus, Jolt collision
  group/filter exposure, and wrapped skinned constraints (see DIVERGENCES.md
  "Soft bodies").
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
  constraint debug draw under `jolt_Debug` (see DIVERGENCES.md, M7/vehicles). The
  configuration is also scriptable (behavior-context reflected, carried by the bus),
  per-vehicle gravity override and solver/collision-test knobs are exposed, and the
  editor component previews the suspension rest pose without entering game mode.
