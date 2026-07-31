# Divergences from PhysX API Compatibility

Per project rule 6, component names / serialize contexts / reflected properties should
match the PhysX gem equivalents. Every intentional divergence is logged here.

The `M0`–`M7` sections are a milestone-by-milestone record and some of their entries
describe limitations that later milestones removed. Those are marked
**[superseded]** with a pointer to the section that now governs, rather than deleted,
so the reasoning behind each decision stays readable. **For the current state of a
feature, trust the topic sections below the milestones.**

## M0 (baseline)

- **Component class and display names use a `Jolt` prefix instead of `PhysX`.**
  e.g. `JoltRigidBodyComponent` ("Jolt Rigid Body") vs `RigidBodyComponent`
  ("PhysX Rigid Body"), `JoltBoxColliderComponent` vs `BoxColliderComponent`.
  Reason: the gem is being built out milestone by milestone; claiming the PhysX names
  now would silently hijack existing PhysX-authored levels with an incomplete
  implementation. Property layout mirrors the PhysX equivalents so a rename/aliasing
  pass later is mechanical. Target: full PhysX-named compatibility components once
  the feature set is complete (M3+).
- **Editor Add-Component menu category is "Jolt Physics"** instead of "PhysX".
  Same reason as above.
- **[superseded — see "Editor components"]** *Runtime components are used directly in
  the editor, with no editor wrapper components.* Every component family now has an
  `EditorJolt*` variant.
- **[superseded — see M3]** *Only one collider component per entity is allowed.*
  Colliders no longer declare `JoltColliderService` as incompatible with itself;
  multi-collider (compound) bodies work.
- **[superseded — see "Shapes"]** *No cylinder collider.* The gem now supplies
  `JoltCylinderShapeConfiguration` and maps it onto a native `JPH::CylinderShape`.
- **`GetNativeType` values differ.** PhysX bodies report constants from its
  `NativeTypeIdentifiers.h`; Jolt bodies report `AZ_CRC_CE("JoltRigidBody")` /
  `AZ_CRC_CE("JoltStaticRigidBody")`. These values are backend-identifying by design.

## M2 (stabilization)

- **`SetCenterOfMassOffset` semantics follow Jolt's model, not PhysX's.** PhysX moves
  the mass frame by +offset while keeping collision geometry fixed relative to the
  actor frame. Jolt cannot express that (its body frame and mass frame are coupled to
  the shape's geometric center), so the gem shifts the collision geometry by -offset
  around the actor frame instead: the actor/entity frame stays put, geometry and the
  mass frame move by -offset. Rotation behavior about the offset point is equivalent;
  what differs is that the collision geometry physically moves relative to the entity
  (a raycast will find the shape displaced by -offset).
- **Friction/restitution combine uses Jolt's built-in rules** (friction: geometric
  mean, restitution: max) rather than PhysX's average/min/max/multiply combine-mode
  properties. The `FrictionCombineMode`/`RestitutionCombineMode` material properties
  are accepted but ignored.
- **Per-body sleep threshold is not configurable** — see "Scene queries and rigid
  body details" for the current, more precise description.
- **[superseded — see "Scene queries and rigid body details"]** *Scene-query filter
  callbacks receive `nullptr` for the `Physics::Shape*` argument.* Hits now carry
  the collider they came from, and the callbacks receive it.
- **Query collision-group filtering is single-directional** (query group mask must
  contain the body's collision layer). PhysX additionally applies the symmetric
  body-group check against the query's layer; queries have no layer in practice.
- **Per-collider settings on multi-collider bodies** (collision layer/group,
  trigger flag, material) are taken from the first collider only. Compound support
  landed in M3 and the restriction survived it for the reasons given there — it is
  a Jolt structural limit, not unfinished work.
- **[superseded]** *`AzPhysics::SceneInterface` is not implemented yet, so
  scene-level events are unavailable.* `JoltSceneInterface` registers the interface
  and forwards `OnSceneCollisionsEvent`, `OnSceneTriggersEvent` and the
  simulation start/finish events to the scene. Note that *registration* was
  forwarded from the start but nothing ever signalled the collision and trigger
  events, so handlers registered through them silently never fired until that was
  fixed — see "Scene-level collision and trigger events" below.
- **[superseded — see "Shapes"]** *Mesh colliders and `Physics::Shape` objects
  remain unimplemented.* `CreateShape` returns a real `JoltShape`, mesh cooking
  works, and `AddShape`/`RemoveShape` mutate live bodies through
  `JPH::MutableCompoundShape`.

## M3 (compound colliders)

- **Per-collider trigger flag inside compound bodies is ignored** (Jolt sensors
  are per-body); the first collider's trigger flag decides for the whole body.
- **Per-collider collision layer/group inside compound bodies uses the first
  collider's values** (Jolt's GroupFilter is per-body). Per-sub-shape
  friction/restitution ARE honored (via contact-listener override).
- **Friction/restitution combine** uses Jolt's defaults (geometric mean / max),
  including for compound sub-shapes.
- **Body rebuilds on collider-set changes are deferred to the next tick** (PhysX
  rebuilds synchronously); removal of a simulated body takes effect immediately
  in the simulation while the object deletion is deferred to the next step.

## M4 (heightfield collider)

- **The heightfield collider is provider-driven only**: it builds its shape from a
  `Physics::HeightfieldProviderRequestsBus` implementation on the same entity (e.g. a
  terrain gem), like PhysX's heightfield component. There is no hand-authored height
  grid in the component itself.
- **Grid axes map to world (x, -y)**: Jolt heightfields are Y-up, so the shape is
  wrapped in a `RotatedTranslatedShape` that maps grid column → +X, grid row → -Y,
  height → +Z (the rotation necessarily mirrors one horizontal axis). The grid's
  corner sits at the entity origin.
- **The Jolt grid is square-padded with no-collision samples** up to the next power
  of two (Jolt requirement); provider grids of any rectangular shape are accepted.
- **Runtime height updates** apply via `HeightFieldShape::SetHeights` + a broadphase
  refresh, and overlapping dynamic bodies are force-woken. Updates are driven by
  `HeightfieldProviderNotificationBus` plus a per-tick poll as a safety net.
  Large raises must be sent in small increments: Jolt's heightfield narrowphase only
  visits blocks overlapping a body's AABB, so a surface raised *past* a resting body
  in one step leaves it stranded underneath (no contact is generated).
- **Per-triangle friction/restitution** are resolved at contact time from the
  provider's material list (contact-listener override), not baked into the shape.

## M5 (character controllers)

- **A character is positioned by its base, not its shape centre** — matching PhysX and
  the rest of O3DE. `Physics::Character` makes `SetBasePosition` writable while
  `GetCenterPosition` is read-only, and the PhysX backend maps the entity transform
  onto `PxController`'s foot position. Jolt has no such notion (a `CharacterVirtual`
  sits at its shape centre), so `JoltCharacter::BaseToCenter` converts on every
  entity-facing path: creation, `GetTransform`/`SetTransform` (which is what the
  component syncs to and from the entity), and the `GetCenterPosition` fallback.
  `CharacterConfiguration::m_position` is therefore a base position too. Until this
  landed the component treated the entity origin as the capsule centre, so a
  character stood half a capsule below where PhysX would put it, and
  `SetBasePosition` disagreed with the entity transform by the same amount.
- **The offset is taken from the shape's own bounds** (`GetBottomOffset`), not from
  the capsule height, so it stays correct for whatever shape drives the character.
- **Requested velocities are applied by the scene at simulation start**, not via an
  `OnSceneSimulationStart` event handler like the PhysX gem; per-tick and per-physics-
  step velocity requests coincide in this backend (both are applied and flushed on the
  next simulation step).
- **The character is visible to the simulation through a kinematic inner body**
  (`CharacterVirtualSettings::mInnerBodyShape`): dynamic bodies collide with and are
  pushed by the character, and sensors fire trigger events for it. PhysX instead uses
  its CCT obstacle/shadow-body machinery.
- **[superseded — see "Collision filtering"]** *The character's collision layer/group
  is stored but not enforced.* Both character backends now resolve their layer and
  group through `AcquireObjectLayer`, so the shared object-layer rule applies to
  them like any other body. Sensors still never block movement.
- **A virtual character's layer has to be applied twice.** `CharacterVirtual` sweeps
  its own movement instead of going through the simulation, so the object layer
  reaches its inner body through `mInnerBodyLayer` but reaches the sweep only via the
  filters handed to `ExtendedUpdate`. Passing default-constructed filters there — as
  the gem did until this was found — accepts every layer, so the character walked
  through everything while other bodies still saw it correctly. The rigid backend has
  no such split; it collides through the simulation like any body.
- **`GetPosition` and `GetTransform` both report the base.** They are the same concept
  on `SimulatedBody`, so they must agree; reading one and writing the other would
  otherwise shift a character by half a capsule.
- **Body-level `RayCast` on a character returns an empty hit** (use scene queries).
- **`AttachShape` on a character is unsupported.** A `Physics::Shape` wrapper exists
  now; what is missing is a way to recombine a character's shape at runtime, since
  Jolt characters take a single shape at construction.
- **No `CharacterGameplayComponent` equivalent**: gameplay drives the character via
  `CharacterRequestBus::AddVelocityForTick` (gravity, jumps), as the smoke test's
  `CharacterDriverTestComponent` demonstrates.

## M6 (joints)

- **Jointed bodies still collide with each other** (PhysX disables the pair by
  default). Jolt has no per-constraint collision toggle; layers/groups or non-
  overlapping shapes are the workaround.
- **Limit semantics are made explicit in the Jolt configs**: hinge `LimitFirst`/`LimitSecond`
  are the lower/upper angle in degrees, prismatic min/max slide in meters, ball cone
  half-angles about joint Y/Z in degrees (PhysX's field semantics are backend-specific).
- **Soft limits exist on hinge and prismatic joints only.** `Stiffness`/`Damping` map
  straight onto Jolt's limit spring in `StiffnessAndDamping` mode (the same k and c a
  PhysX soft limit takes: N m/rad and N m s/rad for the hinge, N/m and N s/m for the
  prismatic). Jolt's swing-twist constraint has no limit spring, so a ball joint asking
  for a soft cone limit warns at creation and keeps the limit hard; the inspector notes
  the restriction on the field.
- **The `Tolerance` limit field is parsed but unused, and hidden from the inspector.**
  PhysX treats it as the distance before a hard limit at which enforcement begins;
  Jolt hard limits engage exactly at the limit and expose no such band. The field
  stays serialized so PhysX-era data loads.
- **Runtime joint control is exposed through `JoltJointRequestBus`** (this gem's own
  bus) mirroring the PhysX gem's `JointRequestBus` surface, since AzPhysics defines no
  joint control bus and the PhysX bus lives in the PhysX gem.
- **`AzPhysics::JointHelpersInterface` is not implemented** (joint auto-configuration);
  joints are configured explicitly. Editor joint-limit visualization and frame
  editing both exist, but through this gem's own `DrawJointLimits` and
  `JoltJointComponentMode` rather than that interface — see "Editor viewport debug
  draw" and "Component modes".
- **Rebinding a joint to different bodies requires recreating it**
  (`SetParentBody`/`SetChildBody` only update bookkeeping).
- **Breakable joints are implemented scene-side** (Jolt has no native breakable
  constraints, unlike PhysX). After every simulation step the scene reads each
  breakable constraint's accumulated solver impulses, divides by the step duration to
  get the average reaction force/torque, and removes the joint when either exceeds
  `Max force`/`Max torque`. A threshold of 0 disables that axis of the test; motor
  impulses are excluded (a joint should not break from its own capped drive); limit
  impulses count. Gear and rack-and-pinion reactions are tested against `Max torque`
  only (the coupling impulse is angular). D6 joints have no generic properties
  (mirroring PhysX) and cannot break. Gameplay hooks: `JoltScene::RegisterJointBreakHandler`
  fires with the removed joint's handle, and joint components forward it as
  `JoltJointNotificationBus::OnJointBroken` on their entity (a broken joint's component
  drops its handle and does not recreate the joint).
- **Connected bodies do not collide with each other** (PhysX's default, so this
  removes a divergence rather than adding one). Jolt has no such flag on constraints,
  so `AddJoint` registers the body pair with the scene and `RemoveJoint` drops it;
  the contact listener's `OnContactValidate` rejects contact generation for
  registered pairs (`JoltScene::AreBodiesJointed`, shared-mutex guarded for
  narrow-phase worker access). The ids ride on the joint so removal needs no body
  lookup. Ragdoll-internal links are a separate construction path and are not
  filtered yet.

## M7 (vehicles)

- **No AzPhysics vehicle interfaces exist in O3DE 26.05** (the PhysXVehicle gem is not
  part of this engine release), so the vehicle system is exposed through the gem's own
  `JoltVehicleComponent` and `JoltVehicleRequestBus` (rule 5 of the project brief).
  There is no PhysXVehicle-API compatibility layer.
- **Chassis mass is set via `JoltVehicleConfiguration::m_chassisMass`** (applied with
  `ScaleToMass`, default 1200 kg) instead of relying on the rigid body's mass, because
  the gem's rigid bodies default to 1 kg which is unusable for a car.
- **Wheel/ground detection is selectable, and defaults to a cylinder cast.**
  `JoltVehicleConfiguration::m_collisionTester` picks Ray, Sphere or Cylinder. A ray
  only tests the wheel's centre line, so a ray-tested wheel drops into any gap and
  catches on any edge narrower than the tyre - there is a test driving a car over a
  trench a third the width of its wheel that the ray falls into and the cylinder
  bridges. The ray and sphere testers need the up axis passed explicitly (their
  defaults are Y-up, which would read O3DE's ground as a wall); the cylinder tester
  takes none, reading the orientation off the constraint. An earlier note here said
  the cylinder tester found no contacts - that was wrong.
- **The pitch/roll limit defaults to 60 degrees**, not Jolt's unlimited 180.
  Without it a vehicle powers itself over: the default tank has enough drive torque
  to pop a wheelie, and with the suspension still pushing from the vertical it lands
  on its back and drives on there. This only became visible once the wheels had real
  grip - with the ray tester they slipped enough to hide it. Both cases are pinned by
  tests.
- **Anti-roll bars are wrapped** (`JoltVehicleConfiguration::m_antiRollBars`), each
  naming the two wheels it couples and a stiffness, validated against the wheel count
  at creation. They are opt-in and empty by default: they cure suspension roll, which
  is worth having on soft springs, but they cannot prevent a vehicle whose lateral
  grip out-levers its track width from tipping over bodily, so switching them on by
  default would have bought nothing measurable at the default spring rates.
- **The chassis is force-woken on driver input**: Jolt's vehicle anti-sleep only resets
  the sleep timer, so a body that fell asleep while parked would never wake up again
  (deadlock that leaves the tire constraints inactive).
- **Wheel state is exposed for rendering** through `JoltVehicleRequestBus`:
  `GetWheelCount`, `GetWheelTransform` (world space, carrying the suspension position,
  the steer angle and the rolling of the tyre), `GetSuspensionLength` and
  `IsWheelOnGround`. The gem still does not move any render entity itself - the
  chassis is a normal rigid body and wheels are not entities - so driving visual
  wheels stays the caller's job, but it no longer needs the native constraint to do
  it. Out-of-range indices report nothing rather than reading past Jolt's array.
- **All three Jolt controllers are available** through `JoltVehicleConfiguration::m_vehicleType`:
  wheeled (car), motorcycle and tracked (tank). PhysXVehicle only models wheeled
  vehicles, so the other two have no PhysX counterpart to be compatible with.
- **A tracked vehicle steers by track speed, not wheel angle.** The shared
  `SetDriverInput(forward, right, brake, handbrake)` maps steering onto a left/right
  track ratio (full lock reverses the inner track, pivoting the vehicle on the spot)
  and folds the handbrake into the brake, since a tank has neither steered wheels nor
  a separate handbrake. Per-wheel steer angles and brake torques are ignored on this
  type; braking comes from the track's own brake torque. Wheels are assigned to the
  left or right track by the sign of their Y position, and the first wheel of each
  track is its driven wheel.
- **Motorcycle lean gains must be sized to the chassis.** Jolt's defaults
  (`mLeanSpringConstant` 5000, `mLeanSpringDamping` 1000) suit a particular bike; on a
  lighter one the balance correction is violent enough to throw the vehicle into the
  air. The gem warns at creation when the gains imply a lean response above 3 Hz for
  the chassis roll inertia and suggests workable values (roughly
  `constant = I * (2*pi*f)^2` and `damping = 2 * I * (2*pi*f)` for `f` around 1-2 Hz).
- **The motorcycle only balances while every wheel is on the ground** (a Jolt
  restriction: the lean impulse is skipped unless all wheels have contact). A bike
  with a car-sized engine wheelies, loses front contact and falls over, so the engine
  needs sizing for the vehicle too.

## Gear and rack-and-pinion joints

- **No PhysX counterpart:** the PhysX gem wraps neither, so these are exposed
  through this gem's own `JoltGearJointConfiguration` and
  `JoltRackAndPinionJointConfiguration` with no compatibility layer to match.
- **They couple motion and nothing else.** Neither constraint holds its bodies in
  place, so each body still needs its own joint - two hinges for a gear, a hinge
  and a prismatic for a rack and pinion - or the bodies simply drift apart. This
  is a Jolt requirement, not a gem limitation.
- **There is no gear geometry.** Nothing meshes and no interpenetration is
  checked; the tooth counts only express a ratio. Gear ratio is
  `parentRotation = -(childTeeth / parentTeeth) * childRotation` (so the driven
  wheel turns the opposite way, as real gears do); rack and pinion is
  `pinionRotation = 2*pi * rackTeeth / (rackLength * pinionTeeth) * rackTranslation`.
  Tooth counts are exposed rather than the raw ratio because that is what an
  author knows, and it is the form Jolt's own `SetRatio` helpers take.
- **The accompanying joints are referenced by `AzPhysics::JointHandle`, and those
  handles are deliberately not serialized.** A joint handle is a runtime index into
  a scene's joint list and means nothing once written to disk, so whatever creates
  the gear fills them in. Jolt uses the references only to measure accumulated
  rotation error and correct drift - the coupling itself works without them, but
  two gears left running slowly lose phase. Both must resolve; with only one there
  is nothing to compare against. They are resolved in `JoltScene::AddJoint` rather
  than in `CreateJoltConstraint`, which has no access to the scene's joint list.
- **No editor or runtime components yet.** Unlike the other eight joint types,
  these are reachable only through `SceneInterface::AddJoint` from code. A
  component would need to reference two *other joint components* rather than two
  entities, which the existing `JoltJointComponentBase` lead/follower model does
  not express.

## Shapes

- **Cylinder colliders are native.** AzFramework declares `Physics::ShapeType::Cylinder`
  but ships no configuration for it (PhysX has no native cylinder and approximates one
  with a cooked convex hull of `DefaultCylinderSubdivisionCount` sides), so the gem
  supplies `JoltCylinderShapeConfiguration` and maps it onto `JPH::CylinderShape`. The
  cylinder is Z-aligned to match the O3DE capsule convention, and is exact rather than
  faceted — colliders authored for PhysX will not transfer, and a rolling PhysX
  "cylinder" behaves subtly differently from a real one.
- **Jolt's tapered capsule, tapered cylinder, plane and triangle shapes are not
  exposed.** They have no `Physics::ShapeType` counterpart, so they would each need a
  gem-specific configuration in the same way as the cylinder.
- **Convex hulls are authored through the mesh collider, not a component of their
  own.** `JoltMeshUtils` builds a `JPH::ConvexHullShape`, and the mesh collider's
  Mesh Type field chooses between "Triangle Mesh" and "Convex Hull" — but there is no
  standalone convex-hull collider component or shape configuration, so a hull always
  comes from cooked geometry rather than from authored points.

## Mesh colliders

- **Jolt needs no cooking pass, so "cooked" is packed geometry on every path.** Jolt's
  `MeshShape` and `ConvexHullShape` build from raw vertices at `Create()` time, so
  everywhere this gem stores cooked data — the blob baked into a prefab by the "Jolt
  Baked Mesh Collider" and the `.joltmesh` product cooked by the Scene Builder (see
  "Asset pipeline mesh colliders" below) — the bytes are just `JoltMeshUtils::Pack*`
  output that the gem writes and reads itself. PhysX's cooking is a real offline pass;
  ours is a memcpy with a header.
- **The baked component's geometry comes from the entity's render mesh, at edit time.**
  The editor component asks `AzFramework::VisibleGeometryRequestBus` — the bus the Mesh
  component answers — and bakes the result into the component. Consequences worth
  knowing before building content on it: the blob is serialized into the **prefab**,
  not into a shared asset, so two entities using the same model carry two copies and
  the level file grows with the mesh; changing the model does not update the collider
  until it is re-baked; and a collider on an entity with no Mesh component has nothing
  to bake from. All three are what the asset pipeline solves; the bake stays the
  quick path for one-off props.
- **Baking is automatic, and retried, because the asset load is a race.** The bake
  attempted on activation almost always loses to the model's asynchronous load, so the
  component stays on the tick bus and retries until the mesh can answer, then reports
  what it baked. Without the retry a freshly added collider stayed silently empty until
  someone found the button — which is what "the bake button does nothing" actually was.
- **A bake is an undoable edit that dirties the entity.** It changes serialized data
  from code rather than from the property grid, so it is wrapped in a
  `ScopedUndoBatch` and marks the entity dirty; otherwise saving skips the entity and
  the baked mesh dies with the session, leaving the level to warn again on every play.
- **"Bake from render mesh" is a button in the property grid**, which PhysX has no
  counterpart for — its equivalent step happens in the asset pipeline. It exists for
  the cases automation cannot cover: the model changed, or the mesh type changed.
- **Convex bakes come in three modes.** *Single Hull* wraps everything in one hull.
  *Hull per Mesh Node* bakes one hull per `VisibleGeometry` entry (per render node —
  wheels separate from the wagon body) and packs them as a hull group: a v2 blob the
  runtime decodes into a `StaticCompoundShape` of hulls. *Decomposed (VHACD)* runs
  approximate convex decomposition over the merged geometry at bake time. The gem
  fetches v-hacd sources (same upstream commit the engine's 3rdParty package was built
  from) and links them **editor-only** — the runtime just decodes the same hull-group
  blob and never links the decomposer. Decomposition runs on a worker thread (it takes
  seconds on dense meshes) and the component collects the result on tick. The blob
  format is versioned: v1 (one point cloud) still decodes, v2 is a counted list of
  hulls; a one-hull group decodes to a bare hull, never a one-child compound.
- **Hulls and compounds are centroid-relative in Jolt, and everything that reads
  their geometry must account for it.** Jolt stores convex-hull vertices relative to
  the hull's centroid, and compound children relative to the compound's center of
  mass — so `GetLocalBounds()` on either is in the centroid frame, `GetTrianglesStart`
  positions shapes by `inPositionCOM`, and `CollectTransformedShapes` culls children
  against the box in that same frame. The wireframe/`GetGeometry` extraction walks
  leaves with a `sBiggest()` box and lets each leaf re-apply its own centroid offset;
  a single hull group under one collider still maps all contact material lookups to
  that collider (`JoltScene::GetMaterialForSubShape`).

## Asset pipeline mesh colliders

- **`.joltmesh` mirrors PhysX's `.pxmesh`, and it is built the same way: no builder of
  our own.** The gem plugs two SceneAPI components into the engine's Scene Builder
  job — `Pipeline::JoltMeshBehavior` (the Scene Settings "Jolt Physics" tab and
  manifest defaults) and `Pipeline::JoltMeshExporter` (packs geometry during the scene
  compilation job) — discovered by reflection in the builder process, exactly like
  PhysX's `MeshBehavior`/`MeshExporter`. The `JoltPhysics.Builders` alias (the editor
  module) already existed, so no new builder registration was needed.
- **"Cooked" is our packed blob, written per shape into the product.** The exporter
  turns each selected mesh node into a `Physics::CookedMeshShapeConfiguration` holding
  a `JoltMeshUtils` blob (triangle soup, one convex hull per node, or one entry per
  VHACD hull when decomposing), collected into `JoltMeshAssetData` with material slots
  and per-shape material indices. The on-disk format is the bare `JoltMeshAssetData`
  struct as a binary ObjectStream — same split as PhysX's `MeshAsset`.
- **Node transforms are baked into the vertices at export**, using the group's
  `CoordinateSystemRule` — identical to PhysX — so the product stores no transforms
  and collision matches the render mesh's coordinate conversion.
- **Consumption mirrors PhysX too.** `JoltMeshColliderComponent` holds a
  `Physics::PhysicsAssetShapeConfiguration` and expands the asset into one
  collider/shape pair per asset shape (`GetShapeColliderPairs`), so the existing body
  path compounds them into one body with per-shape materials and offsets. The product
  uuid is `CreateName(manifest group id)`: renaming a mesh group in Scene Settings
  creates a *different* product and orphans old asset references — same behavior as
  PhysX, worth knowing before renaming.
- **Default groups select only `PhysicsMesh` nodes** (the `_phys` suffix convention
  from SceneProcessing's soft-name settings), exactly like PhysX: a scene without
  `_phys`-suffixed nodes gets a default "Jolt Mesh" group with an empty selection and
  the exporter skips it until nodes are selected in Scene Settings (or named to
  convention). Verified end-to-end with AssetProcessorBatch: authored selections
  produce `.joltmesh`, default empty groups are skipped cleanly.
- **Triangle meshes carry per-face materials, resolved live.** The blob (v2) stores a
  material-slot index per face, baked by Jolt into the mesh tree's 5-bit-per-triangle
  flags — so at most 32 slots per mesh (excess clamps to slot 31 with a warning).
  `JoltScene::GetMaterialForSubShape` reads the hit triangle's slot and resolves it
  against the collider's slot list, so material edits apply to existing bodies (the
  rejected alternative, baking `JPH::PhysicsMaterial` into the shape, is immutable).
  The `mMaterials` list Jolt requires on the shape is placeholder defaults only —
  nothing reads it.
- **Primitive export mode fits by PCA, not by PhysX's optimizer.** "Primitive" in
  Scene Settings fits a box/sphere/capsule (or best-fit by volume) per node from a
  deterministic principal-component analysis — cheaper and looser than PhysX's
  volume-minimizing fit; precision geometry belongs to convex/decompose. Non-uniform
  scale needs no subdivision pass (unlike PhysX): `JPH::ScaledShape` wraps the
  primitive natively.
- **Scale is honored everywhere now.** Shape configuration `m_scale` (entity scale ×
  asset scale) is applied as a `JPH::ScaledShape` decorator in
  `CreateJoltShapeFromConfig` for every shape type; previously the field was ignored.
  Boxes, convex hulls and triangle meshes take a non-uniform scale as authored. Spheres
  and capsules cannot (`JPH::Shape::IsValidScale`; a non-uniformly scaled sphere is not
  a sphere), so a non-uniform scale on those is clamped to the shape's own
  `MakeScaleValid` — the mean of the three components — and warns, rather than letting
  Jolt assert inside the shape. Authoring a squashed sphere or capsule means using a box
  or mesh collider. Heightfields are left unscaled entirely
  (`JoltColliderComponentBase::ApplyOverallScale` skips them).
  The scale is applied in **entity space**, outside any collider rotation — a rotated
  collider on a non-uniformly scaled entity squashes along the entity's axis, like its
  render mesh (`CreateJoltShapeFromPair` puts the `ScaledShape` outside the
  `RotatedTranslatedShape`; Jolt rotates the scale into the child when the rotation
  permutes axes). A rotation that does not map axes onto axes cannot take a non-uniform
  scale exactly — that would shear, which no Jolt shape represents (nor any PhysX one).
  Those approximate instead, scaling the collider's own axes by the length of each
  axis's image under the entity scale (with a warning naming the entity): exact at 0 and
  90 degrees and degrading continuously in between, which matters because the mesh
  pipeline's primitive fit routinely produces frames a few degrees off axis-aligned.
  The dropped shear means collision and visuals still diverge slightly on genuinely
  angled colliders; authoring the scale into the source mesh is the way out.
- **Baking into the prefab stays as the quick path.** The editor "Jolt Baked Mesh Collider"
  component (render-mesh bake) is untouched; the asset pipeline (exposed as "Jolt Mesh
  Collider", matching PhysX's asset-based Mesh Collider) is the shared,
  deduplicated workflow for real content. Per-face materials live only on the asset
  path (the baked component's blob has no material table — the render mesh reports no
  physics materials).

## Collision filtering

- **AzPhysics layer/group filtering is carried by Jolt's object layers.** AzPhysics
  filtering is per body - two bodies collide only if each one's group mask contains the
  other one's layer - which cannot be expressed as a function of two layer indices, so
  the gem registers one Jolt object layer per distinct combination of collision layer,
  collision group and motion class (`JoltObjectLayerRegistry`) and evaluates the rule in
  `ObjectLayerPairFilter`. Broadphase layers keep only the moving/non-moving split.
- **`JPH::CollisionGroup` is left to Jolt.** It is what Jolt's own subsystems use for
  their internal filtering (ragdolls install a `GroupFilterTable` there to disable
  parent/child collisions), so the gem stores nothing in it. An earlier design put the
  layer and mask in the collision group, which both collided with those subsystems and
  meant reading another filter's fields.
- **Scene queries filter by object layer** rather than inspecting each candidate body,
  so a query's collision group mask applies uniformly to every body in the scene,
  including ones Jolt creates itself.
- **Combinations are capped at 512.** Beyond that, further bodies fall back to
  colliding with everything and a warning is emitted. The count is per distinct
  (layer, group, moving) triple actually used, not per body.

## Scene queries and rigid body details

- **Async scene queries complete on the next `FinishSimulation`**, not on a worker
  thread. `QuerySceneAsync`/`QuerySceneAsyncBatch` queue the request and return
  immediately (non-blocking, as the contract requires); the query runs and its callback
  fires during the scene's next simulation finish, so results reflect the state that
  step produced. A scene that is never stepped never delivers its callbacks. The single
  request form is copied on queueing, since only a borrowed pointer is passed in.
- **Query hits identify the collider they came from.** Rigid and static bodies build
  one `Physics::Shape` per collider at creation, in compound sub-shape order, so a
  hit's Jolt sub-shape id maps straight to a collider: `SceneQueryHit::m_shape` is
  populated (with `ResultFlags::Shape` set) and the same pointer is handed to the
  request's filter callback. `AzPhysics::RigidBody::GetShapeCount`/`GetShape` are
  implemented from the same list. Body types that hold no shape objects —
  characters and ragdoll parts, whose bodies Jolt builds itself — still report a
  null shape.
- **Per-collider shapes share geometry with the body's compound rather than
  duplicating it.** Primitive shapes are a few bytes, and a cooked mesh caches its
  native shape on the configuration, so building a `Physics::Shape` alongside the
  compound reuses the same `JPH::Shape` instead of cooking twice.
- **`RigidBody::SetSleepThreshold` only toggles whether the body may sleep.** Jolt has
  no per-body sleep threshold — the velocity below which bodies may sleep is scene-wide
  (`PhysicsSettings::mPointVelocitySleepThreshold`), and what is per-body is a plain
  allow-sleeping flag. A threshold of zero or less is honoured exactly ("never sleep",
  and the body is woken if it was asleep); any positive value means "may sleep" but its
  magnitude is ignored, with a warning. PhysX's threshold is an energy value, so it
  would not have transferred numerically anyway.

## Ragdolls

- **Nodes are connected by swing-twist constraints**, built from the node's joint
  configuration (a Jolt swing-twist, D6 or cone config supplies the limits; anything
  else falls back to moderate defaults). A node with no joint configuration gets a
  point constraint instead, which holds the bodies together but has no limits and
  cannot be motor-driven.
- **`RagdollNodeState::m_strength` maps to the joint motor's spring frequency in Hz**
  (and `m_dampingRatio` to its damping ratio) when soft-keying with
  `DriveToPoseUsingMotors`. PhysX drives its joints from a stiffness/damping pair in
  different units, so authored strength values do not transfer numerically between
  the backends — they need retuning. A strength of zero releases the motor, leaving
  that joint purely physical, which is how a per-node animation/physics blend is
  expressed.
- **Animation driving is exposed through the Jolt-specific ragdoll object**
  (`DriveToPoseUsingKinematics` for hard keying, `DriveToPoseUsingMotors` for soft
  keying); `Physics::Ragdoll` has no driving entry point of its own in this engine
  release, so callers cast to `JoltPhysics::JoltRagdoll`.
- **`SetTransform` on the ragdoll is a no-op** — it is driven per node, so moving it
  as a whole means writing a repositioned pose through `SetState`.
- **All three of Jolt's driving modes are available**: `DriveToPoseUsingKinematics`
  (hard keying - the bodies become kinematic and follow the pose through anything),
  `DriveToPoseUsingVelocities` (soft keying - the bodies stay dynamic and are given a
  *capped* velocity towards the pose, so walls and impacts can overrule it) and
  `DriveToPoseUsingMotors` (joint motors). `ReleaseToPhysics` returns hard-keyed bodies
  to dynamic. The velocity caps are what make soft keying soft: an uncapped velocity is
  recomputed from the full remaining distance every step, which discards the solver's
  correction and drives through obstacles exactly like hard keying.
- **Ragdoll parts carry their own collision layer and group**, taken from each node's
  collider configuration, and Jolt's `GroupFilterTable` still disables parent/child
  collisions within the ragdoll. The two coexist because layer filtering lives in the
  object layer, not the collision group (see below).
- **Skeleton mapping is exposed through `JoltSkeletonMapper`** (wrapping
  `JPH::SkeletonMapper`), which maps between the ragdoll's low-detail skeleton and a
  high-detail animation skeleton in both directions. O3DE has no engine-level interface
  for this, so it is a gem-specific class rather than something reached through
  `Physics::Ragdoll`. Joints are matched **by name**: a ragdoll node's name comes from
  its `RagdollNodeConfiguration::m_debugName`, which therefore has to match the
  animation joint name. The animation skeleton may add joints between ragdoll joints
  and at the root or leaves; those are carried along by the mapping, and joints
  belonging to a chain between two mapped joints are reoriented towards the next
  mapped joint, so extreme poses can show artifacts (a Jolt limitation).

## Scene-level collision and trigger events

- **`RegisterSceneCollisionEventHandler` and `RegisterSceneTriggersEventHandler`
  were registered but never signalled.** `JoltSceneInterface` forwarded
  registration to `AzPhysics::Scene`, which stored the handler, and nothing ever
  called `Signal` on `m_sceneCollisionEvent`/`m_sceneTriggerEvent` — so a
  scene-level handler was silently dead for every body type while per-body
  `OnCollisionBegin`/`OnTriggerEnter` worked. Both are now signalled once per step
  with the whole batch.
- **The scene-level batch reports each pair once**, in its original orientation.
  The per-body dispatch that follows sends each event twice, once from each body's
  point of view with the contact normals flipped; the scene batch is signalled
  before that swapping happens rather than inheriting the doubling.

## Soft bodies

- **There is no PhysX counterpart at all.** PhysX 5 has soft bodies but the O3DE
  PhysX gem does not wrap them, and AzPhysics has no soft-body interface, so the
  whole feature is exposed through this gem's own `JoltSoftBodyComponent` and
  `JoltSoftBodyRequestBus` (rule 5 of the project brief). Nothing here is meant to
  be API-compatible with anything.
- **Geometry is procedural, not authored.** A body is one of three generated
  shapes — `Cloth` (a flat grid that drapes), `Cube` (a solid grid that keeps its
  bulk) or `Balloon` (a closed grid driven by internal pressure) — sized by
  `m_size` and `m_resolution`. There is no path from a mesh asset to a soft body,
  and cloth pinning is limited to three presets (`None`, `Corners`, `TopEdge`)
  rather than an arbitrary vertex selection.
- **Settings split into baked and live, and the split is enforced.** Shape,
  pinning, size, resolution, mass, compliance and allow-sleeping go into the
  particle layout at creation and cannot change on a live body; iterations,
  linear damping, pressure and gravity factor forward straight to Jolt's motion
  properties and can change every frame. The runtime bus only exposes the live
  ones.
- **Soft bodies are `AzPhysics::SimulatedBody`s**, created through
  `SceneInterface::AddSimulatedBody` from a `JoltSoftBodyConfiguration` and owned by
  the scene like any other body. They appear in scene queries, answer body-level
  `RayCast` against their *deformed* surface, raise collision events, and are
  reachable through their scene handle. The component holds a handle rather than
  the body.
- **Their world placement is part of the creation configuration**, not something
  applied afterwards: the particles are generated in world space when the body is
  built. `GetTransform`/`GetPosition` therefore report where the body was created,
  while `GetAabb` reports where its particles actually are — a draped cloth can be
  nowhere near its creation transform.
- **Soft body collision events come from a separate Jolt listener.**
  `JPH::ContactListener` never sees soft body contacts; Jolt reports them through
  `JPH::SoftBodyContactListener`, once per soft body per step, with a per-particle
  manifold rather than a shape manifold. The gem bridges that onto the same
  collision events every other body raises, grouping the contacting particles by
  the body they touch so one event is reported per body pair. The contacts carry
  particle positions and normals.
- **Soft body contacts have no removal callback.** Jolt simply stops reporting a
  pair, so End events are derived by sweeping, after each step, for pairs that were
  not refreshed. Begin/Persist/End therefore behave as they do for rigid bodies,
  but End arrives on the step after contact is lost rather than the moment it
  breaks.
- **Collision layer and group are live settings, not baked ones.** Jolt can move a
  body between object layers with `BodyInterface::SetObjectLayer`, so changing
  either re-resolves the object layer and moves the existing body rather than
  rebuilding it — a rebuild would discard whatever deformation the body had
  settled into. Soft bodies always register as *moving*: there is no static
  variety.
- **Drawing is debug-draw only.** A soft body has no mesh asset and its shape
  changes every step, so `JoltSoftBodyRender` draws it from particle positions —
  in game mode from the live body, and in the Edit viewport from the rest shape
  the settings would generate, so resolution and size changes are visible before
  pressing play. There is no Atom material or renderable mesh.
- **Locking is the inverse of the JoltBuoyancy gem's water volume**, and both are
  load-bearing. That volume runs inside a step listener where every body mutex is
  already held, so it uses the no-lock interface and never adds or removes bodies.
  This class creates and destroys bodies — legal only outside the step — and reads
  particle positions while the solver may run, so it takes a real body lock.
  Creating a body from a step listener would deadlock; reading particles without a
  lock would tear.
- **Why this lives in the physics gem while buoyancy does not:** a soft body *is* a
  body, so it needs object layers, collision filtering and eventually scene handles
  and collision events — all internals of this gem. Buoyancy only perturbs bodies
  something else owns, so it stays an extension gem.

## Editor components (PhysX-style editor/runtime split)

- **Every Jolt component family now has an editor variant** (`EditorJolt*` classes
  deriving from `AzToolsFramework::Components::EditorComponentBase`): box/sphere/
  capsule colliders, static and dynamic rigid bodies, heightfield collider, static
  and mutable compound colliders, character controller, vehicle, soft body, and all
  eight joint types (fixed, ball, hinge, prismatic, D6, distance, cone,
  swing-twist). This mirrors the PhysX gem's
  editor/runtime split. Editor components activate in the Edit viewport (plain
  `AZ::Component`s are silently wrapped by `GenericComponentWrapper` and never
  activate there) and spawn the runtime component via `BuildGameEntity` on
  game-mode/export, copying the serialized configuration through accessors added to
  the runtime components for this purpose.
- **Editor colliders draw their shape wireframe in the Edit viewport** (green),
  and the runtime collider base also draws (teal) for entities in prefabs saved
  before the split — `GenericComponentWrapper` forwards `DisplayEntityViewport` to
  the wrapped component even though it never activates it. Game-mode debug draw
  (`jolt_Debug 1`) works for both.
- **The runtime components no longer appear in the Add Component menu** (their
  `AppearsInAddComponentMenu` attribute was removed). This matches PhysX, where
  only the editor components are addable. Runtime components remain registered so
  prefabs saved before the split keep loading and simulating unchanged (they are
  wrapped by `GenericComponentWrapper` in the editor and instantiate into the game
  entity as before).

## Component modes

- **The primitive colliders use AzToolsFramework's own component modes** rather
  than the hand-written ones PhysX carries. 26.05 ships `BoxComponentMode`,
  `SphereComponentMode`, `CapsuleComponentMode` and `CylinderComponentMode` with
  their manipulators already built; a component joins in by answering the matching
  request bus (`BoxManipulatorRequests`, `RadiusManipulatorRequests`,
  `CapsuleManipulatorRequests`, `CylinderManipulatorRequests`) and connecting a
  `ComponentModeDelegate`. So the Jolt colliders get the same handles, keyboard
  shortcuts and viewport UI as an engine shape component, and PhysX's separate
  `ColliderBoxMode`/`ColliderCapsuleMode`/... classes have no counterpart here
  because none is needed.
- **The collider's translation offset is editable in every mode**, through
  `ShapeManipulatorRequests` on the shared editor base. PhysX splits offset and
  rotation into their own sub-modes; here the offset comes with the shape mode and
  rotation stays a property-grid field.
- **Manipulator edits refresh property values only**, not the whole tree:
  rebuilding the property tree mid-drag destroys and recreates the manipulators
  under the cursor.
- **Dimensions are clamped where the shape demands it.** A capsule's height is the
  total including both caps, so height is held at or above twice the radius and
  radius at or below half the height — otherwise a drag could produce a capsule
  the backend has to reject.
- **Joints have their own component mode**, `JoltJointComponentMode`, since
  AzToolsFramework ships nothing for a bare transform. It puts translation and
  rotation handles on the joint frame and writes back through this gem's
  `JoltJointFrameRequestBus`, so one mode serves all eight joint types and the
  limits each one draws follow the handles (they are drawn relative to that same
  frame). Two differences from PhysX's `JointsComponentMode`: it shows translation
  and rotation together rather than cycling sub-modes from a viewport UI cluster
  (the rotation circles are sized clear of the translation arrows so both stay
  clickable), and it has no snap-to-entity sub-mode. Lead and follower stay
  property-grid fields — they name entities, which no drag handle expresses.
- **Joints report selection bounds** — a cube the size of the drawn frame, centred
  on it. A joint has no geometry, so without them it could not be clicked in the
  viewport at all, which also left no way into component mode. The bounds follow
  the *follower* entity, since that is the space the frame is expressed in.
- **A joint drag is one undo step.** The frame is written on every mouse move so
  the viewport and inspector track the drag, but the undo batch is taken on mouse
  up, scoped so it always closes — a batch left open blocks saving the level.
- **Heightfield, mesh and compound colliders have no component mode**, and so no
  Edit button: their geometry comes from a baked blob, a terrain provider or the
  child entities, none of which a manipulator could drag. The mesh and heightfield
  colliders do report selection bounds (see below), so they are still clickable in
  the viewport; the compound colliders do not, having no geometry of their own.
- **The character controller joins `CapsuleComponentMode` too**, even though it is
  not a collider and does not derive from the collider base. It answers the same
  four manipulator buses directly and reuses the capsule clamping rule (height held
  at or above twice the radius, radius at or below half the height). PhysX's
  character controller has no equivalent mode. Two differences from a collider:
  the capsule is pinned to the entity — it stands on the origin rather than being
  centred on it, per the base-position convention above — so `SetTranslationOffset`
  is inert and the offset manipulator has nothing to move; and an explicitly assigned
  `ShapeConfiguration` outranks Height/Radius at runtime, so when a non-capsule
  shape drives the character the component reports no selection bounds and draws
  nothing rather than showing a capsule that would not match.

## Editor viewport debug draw

- **Every editor component that owns geometry draws it**, through
  `AzFramework::EntityDebugDisplayEventBus`. The shared primitives live in
  `EditorJoltDebugDrawUtils.h` (`EditorDebugDraw`), which builds on the older
  collider-only helpers rather than duplicating them.
- **The character controller previews its capsule.** It is centred on the entity
  origin and Z-aligned, matching what `JoltCharacterControllerComponent` builds
  from Height/Radius. An explicitly assigned `ShapeConfiguration` wins over those
  two fields exactly as at runtime, so the preview follows it; a non-capsule shape
  draws nothing rather than a capsule that would not match.
- **Joints draw their frame, their limits and a link to the lead entity.** The
  frame axes follow the convention `JoltJoint.cpp` uses everywhere: X is the
  primary axis (hinge, slider, twist, cone) and Y the normal/plane reference.
  Limits are amber, the lead link grey. PhysX draws joint helpers through its
  `JointHelpersInterface`; here it is a virtual `DrawJointLimits` on the editor
  joint base, which each joint type overrides.
- **Limit shapes follow the joint type**: hinge and prismatic draw an arc and a
  travel segment, ball/cone/swing-twist/D6 draw a cone, distance draws min and max
  spheres, fixed draws only the frame. Swing limits are named for the axis rotated
  *about*, so they cross over when expressed as cone extents — a swing about Y
  widens the cone towards Z.
- **Vehicles draw each wheel and its suspension travel.** Wheels sit below their
  attachment point along -Z and spin about Y, per `JoltVehicle`; the wheel is drawn
  mid-travel, which is roughly where it rests.
- **Wheel positions are draggable**, through `JoltVehicleComponentMode`: one
  translation handle per authored wheel on its suspension attachment point, writing
  back through this gem's `JoltVehicleWheelRequestBus`. Radius, width, suspension
  travel and steering lock stay property-grid fields - they are single numbers the
  drawn wheel already shows, and a handle each would bury the vehicle. A vehicle with
  no authored wheels offers no handles: an empty list means "use the type's default
  layout", which is built at simulation time and so has nothing to drag.
- **The rigid body draws its centre of mass only when set manually.** With
  `ComputeCenterOfMass` on, the real centre comes from the shapes and is not known
  at edit time, so drawing the stale offset would be actively misleading.
- **The heightfield collider draws the collision surface it is actually given**,
  read from `Physics::HeightfieldProviderRequestsBus` — which is not necessarily
  what the terrain renders, and seeing the difference is the point. Jolt
  heightfields are Y-up and `JoltHeightfieldUtils::WrapZUp` rotates them, so the
  grid runs along +X and -Y from the entity origin; `EditorColliderGeometry`
  computes sample positions to match, and a test pins them against the triangles of
  a real wrapped shape. Grids are strided to at most 64 lines per axis (a 513-sample
  terrain would otherwise be half a million segments) while always drawing the last
  row and column, so the wireframe still reaches the collider's edge.
- **The drawn grid is cached, and refreshed two ways.** `GetHeights` copies the whole
  grid - megabytes for a terrain - so it is not called per frame. A provider that
  announces its edits on `HeightfieldProviderNotificationBus` refreshes the cache on
  the next frame. A provider that mutates silently is caught by a re-read every 15
  draws (about four times a second at 60 fps), which bounds the staleness rather than
  leaving it indefinite. Detecting a silent change cannot be made cheaper: the
  provider interface exposes no revision or hash, so "did this change?" and "give me
  the data" are the same call. What the throttle does buy is that the *rebuild* - the
  expensive half - only happens when the grid really differs, and that the polling is
  counted in draws, so a heightfield nobody is looking at is never re-read at all.
  Selection queries deliberately do not poll; they can arrive in bursts, and anything
  being picked is also being drawn.
- **Mesh and heightfield colliders report selection bounds**, so they can be picked
  in the viewport: the mesh collider's come from the same triangle walk that builds
  its wireframe, the heightfield's from the grid extent and its height range.
- **Static rigid bodies and compound colliders still draw nothing**, deliberately:
  a static rigid body has no geometry of its own, and a compound's children are
  separate entities that draw and are picked as themselves — bounds on the parent
  would put an invisible pick target over all of them.

## Configuration window and collision property editors

- **Editor collider configurations are value members, not shared_ptrs** — matching
  PhysX's editor components and this gem's own character controller. The runtime
  colliders keep shared_ptrs (their shapes get shared with the physics body); the
  editor side gains nothing from the indirection and paid twice for it: the
  inspector showed the configuration as a "1 element" pointer container, and
  property writes through the custom collision layer/group handlers reached the
  in-memory object but never survived into the saved prefab — the same edit on the
  character controller's value member saved fine. The JSON is identical either way
  (the serializer stores a shared_ptr's pointee transparently), so existing data
  loads unchanged, pinned by JoltEditorColliderSerializationTests.

- **A "Jolt Physics Configuration" view pane** (Tools menu) mirrors the PhysX
  Configuration window: a Global Configuration tab (engine timestep/buffer
  settings, the default scene's gravity, and Jolt's capacity settings — max
  bodies, body pairs, contact constraints, temp allocator, job threads) and a
  Collision Filtering tab with Layers (the 64 layer names; layer 0 is pinned and
  names are kept unique) and Groups (a checkbox matrix of preset × named layer,
  with add/remove; read-only presets such as All/None are shown locked). Every
  change is applied to the live physics system and saved immediately — there is
  no separate save button.
- **The gem seeds the collision defaults itself.** AzFramework ships none:
  `CollisionLayers` is 64 blank names and `CollisionGroups` an empty preset list
  (`CollisionGroup::All`/`None` are bitmask constants, not presets). A default
  `JoltSystemConfiguration` therefore names layer 0 "Default" and creates read-only
  "All" and "None" groups, matching what PhysX seeds from its own configuration.
  Without this the Collision Layer and Collides With dropdowns are empty and every
  collider sits on an unnamed layer.
- **The two default group ids are fixed, not generated.** A collider serializes the
  group *id* it was authored with, so a freshly generated id each run would orphan
  every collider that referenced it.
- **There are always exactly 64 collision layers.** They are named, not created:
  the Layers tab edits a fixed array, and an empty name means an unused slot, which
  is why the dropdowns list only the named ones. This matches PhysX.
- **Configuration persists to `<project>/Registry/joltphysicsconfiguration.setreg`**
  under `/O3DE/Physics/JoltPhysics/{SystemConfiguration,DefaultSceneConfiguration}`.
  The settings registry merges project .setreg files at boot in every build
  flavor, so the Editor, launchers and servers all pick up the saved
  configuration; `EnablePhysics` falls back to defaults when nothing was saved.
  PhysX splits its configuration across several .setreg files; this gem uses one.
- **A configuration edit refreshes every open inspector.** The layer/group dropdowns
  rebuild their entries from the live configuration on each `ReadValuesIntoGUI`, but
  the inspector only calls that when something refreshes it — so renaming a layer left
  already-open panels showing the old name. The editor system component now listens for
  `OnConfigurationChangedEvent` (which every edit funnels through) and invalidates the
  property display. The configuration window's own property editor is standalone and
  does not hear the broadcast, so there is no feedback loop.
- **The gem supplies the `CollisionLayerSelector`/`CollisionGroupSelector`
  property handlers** that AzFramework's reflection asks for (Collision Layer /
  Collides With dropdowns). PhysX was previously the only implementer, so these
  fields rendered without dropdowns in PhysX-less projects. Registration is
  skipped if another gem already claimed the handler names. The dropdowns' pencil
  button opens the configuration window on the matching tab.
- **Caveats:** Jolt capacity settings take effect on the next physics system
  initialization (enter/leave game mode), not immediately. Group-mask edits do
  not re-resolve the object layers of already-created bodies — bodies pick up new
  masks when they are recreated (e.g. entering game mode). Gravity edits do apply
  live via the default-scene-configuration-changed event.

## Enhanced internal edge removal

- **On by default, unlike Jolt.** Jolt ships `mEnhancedInternalEdgeRemoval` off because
  it costs a little performance. The gem turns it on: ghost contacts against the seams
  between a mesh's or heightfield's triangles are a correctness problem an author
  cannot diagnose from content — a character catching on flat ground looks like a bug
  in the level, not a physics setting. It is exposed in the configuration window for
  anyone who wants the performance back.
- **Applied to the bodies that move**, not to statics. Jolt decides per contact pair
  with an OR (`Body::GetEnhancedInternalEdgeRemovalWithBody`), so setting it on the
  dynamic or kinematic body covers it sliding over a static mesh; putting it on the
  mesh as well would cost without buying anything. Both character backends get it too —
  `CharacterVirtual` carries the flag itself, and Jolt's rigid `Character` passes it
  through to the body it creates.
- **The vertex tolerance is exposed unsquared.** Jolt stores
  `mInternalEdgeRemovalVertexToleranceSq`; the configuration takes a plain distance in
  metres because that is the unit an author can reason about, and the scene squares it.
  Only reachable at all since Jolt 5.6.0 made it configurable.
- **No PhysX counterpart.** PhysX has its own mesh-contact handling, so there is no
  compatible setting to mirror; this is a Jolt-specific addition to the configuration.

## Solver settings

- **The tunable subset of `JPH::PhysicsSettings` is exposed** in the configuration
  window's Solver Settings group: velocity/position iterations, Baumgarte,
  speculative contact distance, penetration slop, sleep time/velocity threshold,
  allow sleeping, deterministic simulation, and collision (sub-)steps per update.
  The remaining `PhysicsSettings` fields (contact cache tolerances, manifold
  reduction, island splitting, warm starting, active edge checks, step listener
  batching) deliberately stay at Jolt's defaults — they are internal optimization
  toggles, and the window applies its values on top of a default-constructed
  `PhysicsSettings` so those defaults survive.
- **Solver settings apply to live scenes immediately** (`UpdateConfiguration`
  pushes them into every scene's `JPH::PhysicsSystem`); the capacity settings
  (max bodies/pairs/constraints, mutexes) only take effect when a scene's physics
  system is created. Scenes previously ignored the configured capacities entirely
  and used hard-coded constants.
- **Collision steps are system-wide, not per scene.** `AzPhysics::SceneConfiguration`
  is not polymorphic (`AZ_TYPE_INFO` only) and travels by value through the
  AzPhysics API, so a derived per-scene configuration is sliced before any backend
  can read it. The gem's former `JoltSceneConfiguration` was removed for exactly
  this reason — it could never actually reach a scene.

## Simulation state snapshots

- **Scene state can be snapshotted and restored** - the foundation for networked
  rollback and deterministic replay. PhysX's gem exposes nothing comparable, so this
  is gem-specific surface (rule 5): `JoltScene::SaveSimulationState` /
  `RestoreSimulationState` natively, and the same pair on
  `JoltPhysicsSystemRequests` by scene handle for code that does not link the gem's
  scene headers.
- **A snapshot covers everything the next step depends on**: body positions and
  velocities (soft body vertices included), contacts, constraints (vehicle wheel
  speeds, engine and gearbox state included) - and the characters, appended by the
  gem because a `CharacterVirtual` lives outside the body system and Jolt's own
  `SaveState` deliberately skips it. The tests hold replays to *exact* float
  equality: after a restore, resimulation retraces the original bit for bit,
  including a car mid-drive and a cloth mid-fall.
- **Snapshots are for rollback, not persistence.** A blob only restores into a
  scene with the same bodies, joints and characters in the same slots, from the
  same build of the gem (a version number up front makes a foreign or stale blob
  fail cleanly). Deterministic replay additionally requires the same binary, per
  Jolt's own determinism contract.
- **Composition changes fail atomically, which Jolt alone does not guarantee.**
  Jolt's `RestoreState` rejects a *removed* body only part-way through applying
  state, and silently skips a body *added* since the save - a rollback that misses
  one body. The gem writes its own live-body count into the blob and compares it
  before restoring anything, so both cases return false with the scene untouched.
  The count comes from the gem's bookkeeping, not `GetNumBodies()`, which still
  counts bodies awaiting deferred deletion. Only a same-count-different-identity
  mismatch can still leave a partial restore (Jolt's failure path); treat any false
  return as "restore a good snapshot or rebuild". Bodies an extension gem creates
  directly on the native physics system are outside the count and the guarantee.
- **On a successful restore, entity transforms sync immediately** rather than on
  the next simulation step, so a rollback is visible while the simulation is
  paused.

## Jolt features not wrapped

Not divergences from PhysX — Jolt capabilities this gem simply does not expose yet.
Listed so the gaps do not have to be re-derived from Jolt's headers.

- **Two of Jolt's twelve constraints are unwrapped:** path and pulley. A path
  constraint additionally needs spline authoring. Wrapped: fixed, point (exposed as
  "ball"), distance, hinge, slider (exposed as "prismatic"), cone, swing-twist,
  six-DOF (exposed as "D6"), gear and rack-and-pinion.
- **`AzPhysics::JointHelpersInterface` is not implemented** (joint auto-configuration)
  — also noted under M6. Joint-limit visualization is covered by this gem's own
  `DrawJointLimits`, not by that interface.
- **Jolt's own scene serialization (`PhysicsScene`, `ObjectStream`) is not used.**
  Scenes are built from O3DE entities; there is no import/export of Jolt's native
  scene format.
- **Hair is wrapped by the separate JoltHair gem, not by this one.** It runs Jolt's
  GPU strand solver on Atom's DirectX 12 device (proven working in the editor,
  2026-07-26), reaching this gem's backend through `JoltPhysicsSystemRequests` the way
  JoltBuoyancy does. It is a separate gem deliberately: hair needs the
  renderer's device, and this gem has no Atom dependency to keep - a dedicated server
  links physics, not hair. Jolt's own notes still call the solver work in progress,
  and environment collision supports only convex hulls and compound shapes - a hair
  strand ignores a plain box collider.
- **Buoyancy lives in the separate JoltBuoyancy gem**, which drives
  `Body::ApplyBuoyancyImpulse` from a step listener. It is deliberately outside
  this gem — see the reasoning under "Soft bodies".
