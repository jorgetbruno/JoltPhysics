# Known Issues

Anything broken, stubbed or knowingly incomplete in this gem. Intentional deviations from
PhysX behaviour are in DIVERGENCES.md, not here.

- **Open** - still true today.
- **Behaviour worth knowing** - not a defect, but surprising enough to write down.
- **Resolved** - fixed, and kept because how it was found is worth remembering.

## Open

- **The default tracked vehicle flips itself onto its back at full throttle.**

  At full throttle from standstill on flat ground, the default eight-wheel layout under a
  3.0 x 1.6 x 0.6 m box chassis pops a wheelie and lands on its back. It was masked until
  2026-08-26 by the pitch/roll limit defaulting to 60 degrees.

  The cause is the chassis's centre of mass, not the engine. This entry used to say
  sizing the engine torque per vehicle type was the fix; measured on 2026-09-13 (three
  seconds of full throttle, limit off), it is not:

  | setup | peak pitch | result |
  | --- | --- | --- |
  | default: 1200 kg, 500 Nm | 89.9 deg | flips |
  | 1200 kg, 250 Nm | 90.0 deg | flips |
  | 1200 kg, 150 Nm | 13.9 deg | upright |
  | 4000 kg, 500 Nm | 18.6 deg | upright |
  | 1200 kg, 500 Nm, centre of mass 0.3 m lower | 9.8 deg | upright, and fastest (19.9 m/s) |

  Halving the torque still flips it; stopping it by torque alone takes a 70% cut, and a
  taller hull would need more. Lowering the centre of mass keeps the stock engine and the
  tank stays flat - and goes more than twice as fast, since the drive stops going into a
  wheelie. That is what Jolt's own tank sample does (`OffsetCenterOfMassShapeSettings` to
  the bottom of the hull), and it is the same cause as a wheeled car standing on its nose
  under braking.

  **Workaround:** on the chassis rigid body, turn off `Compute COM` and set
  `Centre of mass offset` down toward the hull floor. The pitch/roll limit still works as a
  backstop.

  **Why the gem does not fix it:** it cannot choose the default, because the chassis
  collider - and so its centroid - is the author's. A creation-time warning would not be
  reliable either: the textbook front-lift threshold for this geometry is 13.1 m/s², yet
  it flips at a computed 6.3 and holds at 3.8. Suspension and drive-shock dynamics add a
  factor of two to three, so a warning built on the static rule would stay silent in
  exactly the cases that flip.

- **A shape-overlap query aimed at a soft body can hit a Jolt assertion on a collapsed
  face.**

  Colliding a convex shape against a soft body walks its faces and seeds GJK with each
  triangle's raw cross product, which Jolt asserts on when that is near zero
  (`JPH_ASSERT(!ioV.IsNearZero())` in `EPAPenetrationDepth.h`, tolerance 1e-12 on the
  squared length). Cloth with little or nothing pinned genuinely does collapse a face
  when something crushes it: measured on an unpinned crest, the smallest face fell from
  1.97e-4 to 8.1e-7 in five frames.

  Most paths to it are closed:
  - Characters no longer collide with soft bodies at all (see DIVERGENCES.md,
    "Characters do not collide with cloth").
  - Every face this gem builds is checked against Jolt's own tolerance before it is
    handed over.
  - Ordinary rigid-body simulation does not walk faces: the soft body collides its own
    particles against rigid shapes, which is why crates and balls rest on cloth without
    trouble.
  - A shape *cast* is safe: `EPAPenetrationDepth::CastShape` seeds GJK with the cast
    direction and handles a degenerate contact normal explicitly.

  What remains is a shape **overlap** that does not filter soft bodies out.

  **Workaround:** reject soft bodies in the request's filter callback. The overlap path
  runs it before narrow phase for soft bodies (`SceneQueryPreNarrowPhaseBodyFilter`), so
  their faces are never walked. Unfiltered, the consequence is a logged `AZ_Error` and one
  contact resolved along a meaningless direction - not a dead process, since assertions
  report rather than break (see below).

- **The spline-follow force PhysX offers is not wrapped.** It needs a spline component to
  follow and an authoring story of its own. The other five force types are wrapped.

- **The shape collider does not wrap Quad.** Box, Sphere, Capsule, Cylinder and Polygon
  Prism are handled; a quad has no volume for Jolt to collide with.

- **Compound and heightfield colliders have no edit-mode bodies.** The primitive and mesh
  editor colliders create static bodies in the editor scene; the compound colliders
  deliberately do not, since their children are separate entities with colliders of their
  own, and a heightfield's geometry lives with the terrain provider.

- **Vehicles are exposed only through this gem's own component and bus.** O3DE 26.05 has
  no AzPhysics vehicle interfaces - the PhysXVehicle gem is not part of this engine.

- **A soft body that is not a character draws nothing.** This gem offers debug draw and
  the bus's bulk vertex and triangle reads, which is correct for a gem that depends on no
  renderer. A character's cloth is simulated, skinned and drawn by the **JoltCloth**
  sibling gem, which reads the actor's cloth mesh (weights and painted `CLOTH_DATA` alike),
  hands the geometry back through `SetCustomGeometry`, and writes the simulation into the
  render mesh. A soft body built from a `.joltmesh` or
  a procedural shape has no render mesh to write into, so drawing one still means reading
  its vertices and drawing them yourself.

## Behaviour worth knowing

- **Jolt assertions are reported, not fatal, and are compiled out of release.** Jolt's
  assert callback contract is that returning true triggers a breakpoint. This gem used to
  return true, which turned every assertion - including ones about geometry it could
  recover from - into a dead process, in release builds as well, since `USE_ASSERTS` was on
  for every configuration. The callback now returns false and the define is
  per-configuration. An assertion is still a real defect worth fixing, and it is logged as
  an error with Jolt's own file and line, but it no longer takes the level down with it.

## Build / Tooling

- **The gem registers via `external_subdirectories`**, not the legacy `gems` list. That is
  O3DE 26.05 manifest behaviour, not a bug.

- **`o3de.bat` prints `Invalid engine json` warnings** for stale engine entries (25.05,
  24.09.2, 25.10.x) in `%USERPROFILE%\.o3de\o3de_manifest.json`. Harmless noise.

## Resolved (kept for reference)

### 2026-09

- **`jolt_Debug` unpacked every shape each frame** (resolved 2026-09-13).

  Measured on a city level of 648 shapes and 411,000 triangles, the toggle cost 50 ms a
  frame - 130 fps down to 17 - which made it unusable on exactly the scenes it exists to
  inspect. This list used to call it polish.

  `JoltDebugRenderer` now implements Jolt's full `DebugRenderer`: each shape's unique
  edges are built once when Jolt creates the batch and only transformed per frame; the
  renderer lives with the system component instead of being rebuilt (and re-tessellating
  the unit primitives) on every draw; and lines go straight into per-colour buffers
  rather than through a callback and a hash lookup each. Same level, after: 12 ms a frame,
  51 fps. Lines fell from 1,232,002 to 640,986, since every interior edge had been drawn
  twice, and the walk from 27 ns a line to 9.7.

  What is left is mostly the flush - 7 ms of handing 1.3 million points to the renderer -
  which scales with the line count and is not this gem's code.
  `jolt_DebugDrawProfile <frames>` prints the split, so the next change can be measured
  the same way.

- **A concave polygon prism collided as its convex hull** (resolved 2026-09-13). It now
  collides as its outline: the footprint is ear-clipped and each triangle extruded to a
  hull, which is exact and still a live read. This list was wrong that it needed the mesh
  collider's decomposition bake - a prism's concavity lives in a plane. See DIVERGENCES.md,
  "Shape collider".

- **Force regions had no editor component and no viewport preview** (resolved
  2026-09-13). `EditorJoltForceRegionComponent` now owns the Add Component entry and draws
  each force as an arrow from the region's origin: world-space along its axis, local-space
  turned with the entity, a point force as six radial arrows outward or inward by sign.
  Drag and damping draw nothing, since they have no direction until there is a body. The
  runtime component still loads from prefabs that carry it directly.

- **A scene query against a heightfield reported the collider's material, not the
  square's** (resolved 2026-09-08). The per-square lookup existed only on the contact path,
  so a ray cast at terrain reported collider 0's material however the surface was painted,
  and footstep audio or impact effects keyed off a raycast heard one surface for the whole
  heightfield. The square-index arithmetic now lives in one helper
  (`HeightfieldColliderIndexForSubShape`) that both paths call, which is what stops them
  drifting apart again - that drift was the whole defect. Two raycast tests cover it, split
  along x and along y, because a row/column transposition or a wrong stride survives a
  split in one axis only.

- **The CCD toggle was listed as resolved in M2 and did not work until 2026-09-07.**
  `m_ccdEnabled` was exposed by the inspector and read by nothing, so every body ran
  Discrete however the checkbox was set. Kept because this list certified a feature for
  months on the strength of the field existing rather than of anything reading it - which
  is the failure this list is meant to catch.

### Later milestones

- **`AzPhysics::JointHelpersInterface` and `EditorJointHelpersInterface`** are implemented
  (`JoltJointHelpers`, `JoltEditorJointHelpers`), so the Animation Editor's ragdoll joint
  tools work in a project running Jolt: joint types to author, initial limits computed from
  a bone's rest pose and example rotations, swing-cone and twist-arc visualization data,
  and the limit auto-fit. Only the PhysX gem registers these in the shipped engine, and
  both `EMotionFX.dll` and `AzFramework`'s `CharacterPhysicsDebugDraw` reach for them - so
  a ragdoll this gem could simulate could not be authored.

- **Characters fall on their own.** The character controller applies the scene's gravity,
  tracks the falling velocity between frames, and sheds it on landing (see DIVERGENCES.md,
  "Character gravity"). PhysX puts this in a separate example
  `CharacterGameplayComponent`; here it is on the controller.

- **`AttachShape` works**, and collider components on a character's entity are gathered
  into `CharacterConfiguration::m_colliders` and attached the same way. Both end up on a
  kinematic body that follows the character rather than on the shape it moves with (see
  DIVERGENCES.md, "Character attachments").

- **`AzPhysics::SceneInterface`** is implemented (`JoltSceneInterface`), including
  scene-level trigger and collision events, so engine-wide consumers such as the WhiteBox
  gem work against it.

- **Editor world.** `EditorWorldBus::GetEditorSceneHandle` returns a real editor scene
  (named `"EditorScene"`, mirroring PhysX): edit-mode scene queries work, and the scene is
  disabled during play-in-editor and re-enabled on stop. Like PhysX, the system tick does
  step it while enabled (`Simulate` steps every enabled scene), but it hosts nothing
  dynamic - it is a query and body host for editor tools. A dynamic body added to it would
  simulate in edit mode.

- **Edit-mode collider bodies.** The primitive (box, sphere, capsule, cylinder), baked mesh
  and mesh asset editor colliders create static bodies in the editor scene - PhysX's
  `CreateStaticEditorCollider` equivalent - so editor-time physics queries hit what the
  viewport shows. Bodies follow entity moves, rebuild on scale or property changes and on
  re-bakes and asset loads, and are removed on deactivate.

- **Joints disable collision between connected bodies** (PhysX default). `AddJoint`
  registers the body pair and `RemoveJoint` drops it; the contact listener rejects contact
  generation for registered pairs (`JoltScene::AreBodiesJointed`, guarded for narrow-phase
  worker access).

- **Mesh colliders and convex hulls.** `CookConvexMeshToFile/Memory` and
  `CookTriangleMeshToFile/Memory` pack the geometry blob (Jolt needs no cooking pass),
  `SystemRequestBus::CreateShape` returns a `JoltShape` wrapper, and
  `JoltRigidBody::AddShape`/`RemoveShape` manage runtime compound shapes. The editor mesh
  collider bakes triangle-mesh or convex-hull collision from the render mesh (single hull,
  hull per mesh node, or V-HACD decomposition), and source scenes cook into shared
  `.joltmesh` product assets in the Asset Processor via the Scene Builder (see
  DIVERGENCES.md, "Asset pipeline mesh colliders").

- **Multiple colliders per entity** (M3). Colliders no longer declare `JoltColliderService`
  self-incompatible; static and mutable compound collider components group child colliders
  into one body.

- **Soft bodies** (M8). `JoltSoftBodyComponent` and `JoltSoftBodyRequestBus`, with
  procedural geometry and `AzPhysics::SimulatedBody` integration; water and buoyancy live
  in the separate JoltBuoyancy gem. Later extended with the full creation surface
  (friction, restitution, vertex radius, max particle velocity, update-position,
  double-sided faces, opt-in LRA tethers), bulk vertex and triangle reads on the bus,
  runtime per-particle pinning and velocity, an edit-mode live preview simulating in the
  editor scene, mesh-sourced bodies from `.joltmesh` assets (welded), a per-particle
  contact notification bus, Jolt collision group and filter exposure, and wrapped skinned
  constraints (see DIVERGENCES.md, "Soft bodies").

- **Editor collider visualization and component modes.** Viewport wireframes with
  selection bounds for all collider types (box, sphere, capsule, cylinder, mesh/convex,
  heightfield); Box and Capsule component modes, plus joint, vehicle and character
  controller modes.

- **Character controller filtering and queries.** The character's collision layer and
  group are honoured for its own movement (Jolt's default filters over the character's
  object layer), and body-level `RayCast` casts against the shape at its current pose.

- **Vehicle wheel state for rendering.** Exposed through `JoltVehicleRequestBus`
  (`GetWheelCount`, and `GetWheelTransform` in world space, carrying suspension); driving
  the wheel meshes from it remains the project's side.

- **Vehicle tuning surface.** Per-wheel tyre friction curves; engine torque curve and idle;
  automatic and manual transmission with shift tuning and `SetGear`; a differential list
  with limited slip (AWD); suspension preload, force point and spring modes; per-axis
  driver input setters; slip and contact wheel readouts; motorcycle lean tuning and runtime
  toggles; authorable tracked driven wheels; `RecreateVehicle` for runtime configuration
  edits; and vehicle constraint debug draw under `jolt_Debug` (see DIVERGENCES.md, M7).
  Both of Jolt's C++ per-wheel hooks are on the bus - combine-friction for
  terrain-dependent grip, and tire-max-impulse for the tyre model itself, since Jolt's
  default clamps longitudinal and lateral independently and a friction circle has to be
  supplied - and both are re-applied across `RecreateVehicle`. The configuration is
  scriptable (behaviour-context reflected, carried by the bus), per-vehicle gravity
  override and solver and collision-test knobs are exposed, and the editor component
  previews the suspension rest pose without entering game mode.

### M2

- **Physics materials.** `JoltMaterial` and `JoltMaterialManager` registered on
  `AZ::Interface<Physics::MaterialManager>`; friction and restitution resolved from
  collider material slots at body creation.

- **Collision filtering.** Per-collider layer and group honoured via
  `AzPhysicsGroupFilter` (group masks) plus the layer index in the Jolt collision subgroup
  id. Query-side filtering by collision group and static/dynamic type works.

- **Triggers.** `m_isTrigger` produces Jolt sensors; enter and exit events are delivered
  via `SimulatedBody::ProcessTriggerEvent`.

- **Scene queries.** Raycast (complete hits: handle, entity, normal), shape cast (with MTD
  recovery), and overlap (one hit per body).

- **Rigid body.** Kinematic targets, mass and inertia getters and overrides, centre-of-mass
  offset (Jolt semantics - see DIVERGENCES.md), and simulation enable/disable.

- **`FindAttachedBodyHandleFromEntityId`** is implemented.

- **Debug draw** via `Physics::SystemDebugRequestBus::DebugDrawPhysics`.

- **Cylinder colliders survived the engine dropping `CylinderShapeConfiguration` in
  26.05.** The gem carries its own `JoltCylinderShapeConfiguration` and ships both runtime
  and editor cylinder collider components; the M2 work was replacing the engine type, not
  removing the feature.
