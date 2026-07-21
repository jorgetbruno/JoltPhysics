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
