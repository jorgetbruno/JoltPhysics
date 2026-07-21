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
