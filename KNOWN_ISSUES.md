# Known Issues

Tracked here per milestone process: anything that is broken, stubbed, or knowingly
incomplete, with the milestone expected to address it.

## Backend

- **Physics materials are not applied.** O3DE 26.05 moved material management to
  AzFramework's `Physics::Material` system (`PhysicsMaterialManager`); the gem's old
  `CreateMaterial`/`GetDefaultMaterial` overrides were removed in the 26.05 port and
  collider `MaterialSlots` are currently ignored when shapes are created. All bodies
  use Jolt's default friction/restitution. → M2
- **Collision layer/group from `ColliderConfiguration` is ignored.** Body creation
  hardcodes the Jolt object layer (`ObjectLayers::Moving`/`NonMoving`); per-collider
  layers and groups do not reach the simulation yet (scene-query filtering honors
  them only partially). → M2
- **Triggers unsupported.** `ColliderConfiguration::m_isTrigger` is ignored when
  building shapes; bodies always collide. → M2
- **Mesh colliders unavailable.** `CookConvexMeshToFile/Memory`,
  `CookTriangleMeshToFile/Memory` are stubs returning false; `SystemRequestBus::CreateShape`
  returns `nullptr` (no `Physics::Shape` wrapper exists). → M2/M3
- **`JoltRigidBody` stubbed methods** (added to satisfy the 26.05
  `AzPhysics::RigidBody` interface): `AddShape`, `RemoveShape`,
  `SetCenterOfMassOffset`, `GetInertiaLocal/World`, `UpdateMassProperties`.
  `SetKinematicTarget` performs a plain `SetTransform` (no velocity-continuous
  kinematic motion). → M2
- **`FindAttachedBodyHandleFromEntityId` is unimplemented** (always returns invalid). → M2
- **Debug draw not wired.** `Physics::SystemDebugRequestBus::DebugDrawPhysics` is not
  implemented despite Jolt's `JPH_DEBUG_RENDERER` being compiled in. → M2
- **No editor (edit-mode) world.** Only the game default world exists
  (`JoltDefaultWorldComponent`); edit-mode simulation and edit-mode scene queries
  are unavailable. → M2+

## Components

- **One collider per entity.** Collider components declare themselves mutually
  incompatible (`JoltColliderService`); multi-collider entities (compound colliders)
  arrive in M3. The backend already builds `StaticCompoundShape`s from
  `ShapeColliderPairList`s.
- **Cylinder collider removed.** AzFramework 26.05 no longer has
  `CylinderShapeConfiguration`; `ShapeType::Cylinder` falls through to a null shape.
- **No character controller, heightfield, joints, vehicles, soft bodies, water** —
  scheduled M4–M8.

## Build / Tooling

- Gem registers via `external_subdirectories` (O3DE 26.05 manifest behavior), not the
  legacy `gems` list — expected, not a bug.
- Stale engine entries in `C:\Users\jorge\.o3de\o3de_manifest.json` (25.05, 24.09.2,
  25.10.x) print `Invalid engine json` warnings from `o3de.bat`; harmless noise.
