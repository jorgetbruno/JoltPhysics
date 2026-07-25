# Divergences from PhysX API Compatibility

Per project rule 6, component names / serialize contexts / reflected properties should
match the PhysX gem equivalents. Every intentional divergence is logged here.

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
- **Runtime components are used directly in the editor** (mirroring how
  `BoxColliderComponent` works), but no editor wrapper components equivalent to
  PhysX's `EditorColliderComponent` / `EditorMeshColliderComponent` exist yet.
- **Only one collider component per entity is allowed** (`JoltColliderService` is
  self-incompatible). PhysX allows many colliders per entity; multi-collider
  (compound) support is milestone M3.
- **No cylinder collider.** The engine itself removed `CylinderShapeConfiguration`
  in 26.05, so there is nothing to mirror — noted for completeness.
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
- **Per-body sleep threshold is not configurable** (`SetSleepThreshold` stores the
  value but Jolt does not support per-body sleep thresholds; the config value is
  honored at body creation via `mAllowSleeping`).
- **Scene-query filter callbacks receive `nullptr` for the `Physics::Shape*`
  argument** (the gem has no Physics::Shape wrapper yet; the SimulatedBody argument
  is valid).
- **Query collision-group filtering is single-directional** (query group mask must
  contain the body's collision layer). PhysX additionally applies the symmetric
  body-group check against the query's layer; queries have no layer in practice.
- **Per-collider settings on multi-collider bodies** (collision layer/group,
  trigger flag, material) are taken from the first collider only until compound
  collider support lands in M3.
- **`AzPhysics::SceneInterface` (high-level scene management and scene-event
  registration) is not implemented yet.** Scene-level events
  (`OnSceneTriggersEvent`, `OnSceneCollisionsEvent`, simulation start/finish) are
  therefore unavailable; body-level events work.
- **Mesh colliders and `Physics::Shape` objects remain unimplemented** (cooking
  stubs, `CreateShape` returns nullptr, `AddShape`/`RemoveShape` on bodies are
  no-ops). Scheduled after compound collider work.

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

- **Requested velocities are applied by the scene at simulation start**, not via an
  `OnSceneSimulationStart` event handler like the PhysX gem; per-tick and per-physics-
  step velocity requests coincide in this backend (both are applied and flushed on the
  next simulation step).
- **The character is visible to the simulation through a kinematic inner body**
  (`CharacterVirtualSettings::mInnerBodyShape`): dynamic bodies collide with and are
  pushed by the character, and sensors fire trigger events for it. PhysX instead uses
  its CCT obstacle/shadow-body machinery.
- **The character's own collision layer/group is stored but not enforced** on its
  movement queries yet (default Jolt filters collide with everything); sensors never
  block movement either way.
- **Body-level `RayCast` on a character returns an empty hit** (use scene queries).
- **`AttachShape` is unsupported** (no `Physics::Shape` wrapper in the backend yet).
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
- **Runtime joint control is exposed through `JoltJointRequestBus`** (this gem's own
  bus) mirroring the PhysX gem's `JointRequestBus` surface, since AzPhysics defines no
  joint control bus and the PhysX bus lives in the PhysX gem.
- **`AzPhysics::JointHelpersInterface` is not implemented** (editor joint-limit
  visualization and auto-configuration); joints are configured explicitly.
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

## Shapes

- **Cylinder colliders are native.** AzFramework declares `Physics::ShapeType::Cylinder`
  but ships no configuration for it (PhysX has no native cylinder and approximates one
  with a cooked convex hull of `DefaultCylinderSubdivisionCount` sides), so the gem
  supplies `JoltCylinderShapeConfiguration` and maps it onto `JPH::CylinderShape`. The
  cylinder is Z-aligned to match the O3DE capsule convention, and is exact rather than
  faceted — colliders authored for PhysX will not transfer, and a rolling PhysX
  "cylinder" behaves subtly differently from a real one.
- **Jolt's tapered capsule, tapered cylinder and plane shapes are not exposed.** They
  have no `Physics::ShapeType` counterpart, so they would each need a gem-specific
  configuration in the same way as the cylinder.

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

## Editor components (PhysX-style editor/runtime split)

- **Every Jolt component family now has an editor variant** (`EditorJolt*` classes
  deriving from `AzToolsFramework::Components::EditorComponentBase`): box/sphere/
  capsule colliders, static and dynamic rigid bodies, heightfield collider, static
  and mutable compound colliders, character controller, vehicle, and all five joint
  types (fixed, ball, hinge, prismatic, D6). This mirrors the PhysX gem's
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
