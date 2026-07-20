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
