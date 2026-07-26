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
  joints are configured explicitly. Editor joint-limit visualization does exist, but
  through this gem's own `DrawJointLimits` rather than that interface — see
  "Editor viewport debug draw".
- **Rebinding a joint to different bodies requires recreating it**
  (`SetParentBody`/`SetChildBody` only update bookkeeping).
- **Breakable joints are not implemented** (the `Breakable` flag is parsed but ignored).

## M7 (vehicles)

- **No AzPhysics vehicle interfaces exist in O3DE 26.05** (the PhysXVehicle gem is not
  part of this engine release), so the vehicle system is exposed through the gem's own
  `JoltVehicleComponent` and `JoltVehicleRequestBus` (rule 5 of the project brief).
  There is no PhysXVehicle-API compatibility layer.
- **Chassis mass is set via `JoltVehicleConfiguration::m_chassisMass`** (applied with
  `ScaleToMass`, default 1200 kg) instead of relying on the rigid body's mass, because
  the gem's rigid bodies default to 1 kg which is unusable for a car.
- **Wheel collision uses `VehicleCollisionTesterRay`** (ray per wheel); the cylinder
  tester found no contacts in this integration (not investigated further).
- **The chassis is force-woken on driver input**: Jolt's vehicle anti-sleep only resets
  the sleep timer, so a body that fell asleep while parked would never wake up again
  (deadlock that leaves the tire constraints inactive).
- **No visual sync for wheels** — the chassis is a normal rigid body; wheel transforms
  for rendering are read from the native constraint by user code.
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
- **Convex hulls have no authoring path of their own.** `JoltMeshUtils` builds a
  `JPH::ConvexHullShape` when cooking, so hulls exist at runtime, but there is no
  convex-hull collider component or shape configuration to author one directly.

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
  the wrapped component even though it never activates it. Heightfield/mesh shapes
  are not drawn (the terrain provider already visualizes the surface; mesh geometry
  is not cheaply available). Game-mode debug draw (`jolt_Debug 1`) works for both.
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
- **Heightfield, mesh and compound colliders have no component mode.** They share
  the same editor base but leave its shape-bounds hook at its default, so they
  report no selection bounds and get no Edit button — the same reason they are not
  drawn in the viewport, their geometry is not cheaply available here.
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
- **The rigid body draws its centre of mass only when set manually.** With
  `ComputeCenterOfMass` on, the real centre comes from the shapes and is not known
  at edit time, so drawing the stale offset would be actively misleading.
- **Static rigid bodies, heightfield and compound colliders still draw nothing**,
  deliberately: a static rigid body has no geometry of its own, a compound's
  children draw their own wireframes, and the heightfield surface is already drawn
  by whatever provides it.

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

## Jolt features not wrapped

Not divergences from PhysX — Jolt capabilities this gem simply does not expose yet.
Listed so the gaps do not have to be re-derived from Jolt's headers.

- **`StateRecorder` (save/restore of simulation state) is untouched.**
  `PhysicsSystem::SaveState`/`RestoreState` and `StateRecorderImpl` are what
  networked rollback/resimulation and deterministic replay are built on, and
  nothing in the gem can snapshot or restore state today. The largest single gap.
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
- **Hair exists in Jolt 5.6.0 but is not wrapped.** `Jolt/Physics/Hair/` is a
  strand-based simulation that runs on the GPU through Jolt's new compute layer
  (`Jolt/Compute/`, with DX12/Vulkan/Metal backends), so wrapping it means standing
  that compute path up next to Atom's own device rather than just binding an API.
  Jolt's own notes call it work in progress. Environment collision currently
  supports only convex hulls and compound shapes.
- **Buoyancy lives in the separate JoltBuoyancy gem**, which drives
  `Body::ApplyBuoyancyImpulse` from a step listener. It is deliberately outside
  this gem — see the reasoning under "Soft bodies".
