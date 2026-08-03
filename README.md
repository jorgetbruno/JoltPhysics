# JoltPhysics Gem for O3DE

A standalone O3DE Gem that integrates [Jolt Physics](https://github.com/jrouwe/JoltPhysics)
as an AzPhysics backend, providing an alternative to the default PhysX Gem with API
compatibility as the goal. Every intentional difference from PhysX is recorded in
[DIVERGENCES.md](DIVERGENCES.md) — **that file, not this one, is the authority on how a
given feature behaves.**

The gem depends on **no renderer** and never will: extension gems that need one live
outside it (see [The gem family](#the-gem-family)).

## Features

### Implemented

**Core**
- Physics system integration via `AzPhysics::SystemInterface`, scene/world management
  with simulation stepping, incl. the game default world
- Dynamic, static and kinematic rigid bodies; body activation/deactivation; velocity
  and impulse control; per-scene gravity
- Rigid body buses (`Physics::RigidBodyRequestBus`,
  `AzPhysics::SimulatedBodyComponentRequestsBus`)
- Scene queries — raycast, shapecast and overlap, synchronous and deferred-async;
  hits identify the collider they came from
- Physics materials (friction/restitution), collision layers and groups (system level
  + per-collider filtering), trigger (sensor) colliders with enter/exit events
- Enhanced internal edge removal (on by default) and exposed solver settings

**Shapes and colliders**
- Primitives: Box, Sphere, Capsule, Cylinder
- Mesh colliders, two ways to author them:
  - **Jolt Mesh Collider** (PhysX-style): references a `.joltmesh` asset cooked by the
    Asset Processor from the source scene — authored in the Scene Settings "Jolt
    Physics" tab with triangle mesh, convex (one hull per node), merge, and VHACD
    decomposition options; shared across entities and re-cooked automatically when the
    model changes
  - **Jolt Baked Mesh Collider**: bakes the entity's own render mesh into the prefab —
    triangle mesh, single hull, hull per mesh node, or VHACD decomposition — with a
    **Bake from render mesh** button that retries until the asset finishes loading
- Shape configuration scale honored for every shape type (wrapped as `JPH::ScaledShape`)
- Compound colliders: multiple colliders per entity, plus static/mutable compound
  components that gather child-entity colliders, with per-sub-shape materials
- Heightfield collider fed by `Physics::HeightfieldProviderRequestsBus` (terrain gems),
  incl. runtime height updates and per-triangle materials

**Articulated and specialised bodies**
- Character controller (`JoltCharacterControllerComponent` over `JPH::CharacterVirtual`):
  ground detection, slope limits, step offset, stick-to-floor, pushing dynamic bodies,
  trigger/sensor interaction, and gravity — it falls on its own, with a multiplier to
  turn that off for an animation-driven character and a writable falling velocity to
  jump with. Collider components on the character's entity are attached to it, on a
  kinematic body that follows the character rather than on the shape it walks with, so
  hitboxes and weapon volumes work without changing how it moves
- Ragdolls through O3DE's `Physics::Ragdoll` interface, authorable in the Animation
  Editor: the gem answers `AzPhysics::JointHelpersInterface` and
  `EditorJointHelpersInterface`, which its ragdoll joint tools run on
- Soft bodies (`JoltSoftBodyComponent`): procedural cloth, cube and balloon plus
  mesh-sourced bodies from `.joltmesh` assets, simulated as real bodies the rest of
  the scene collides with, with runtime per-particle pinning/velocity, opt-in LRA
  tethers for inextensible cloth, bulk vertex reads for script-driven rendering,
  per-particle contact notifications (`JoltSoftBodyNotificationBus`), Jolt collision
  group/filter exposure, wrapped skinned constraints for skeleton-attached cloth,
  and an edit-mode live preview that drapes over the editor colliders' geometry. No
  PhysX counterpart exists — this is gem-specific surface
- Joints: fixed, hinge, ball, prismatic, distance, cone and 6-DOF components, with
  limits, motors, **soft (spring) limits** and **breakable** thresholds that consume
  the constraint's own reaction impulses. Gear and rack-and-pinion are wrapped at the
  configuration level, without components
- Wheeled, motorcycle and tracked vehicles (`JoltVehicleComponent` over `JPH::VehicleConstraint`):
  engine with torque curve and idle, automatic/manual transmission with shift tuning and
  gear control from script, per-wheel tire friction curves, multiple differentials with
  limited slip (AWD), steering, brakes, suspension (preload, force point, spring modes),
  selectable wheel collision testers, a pitch/roll limit, anti-roll bars, per-vehicle
  gravity override and solver/wheel-test knobs, a bus publishing live wheel state
  incl. slip and contact readouts for skid audio/VFX, script-side configuration
  authoring, and an editor suspension-settle preview (rest pose without game mode)

**Editor**
- An `EditorJolt*` component for every feature (PhysX-style editor/runtime split): they
  are what the Add Component menu offers, and they spawn the runtime components via
  `BuildGameEntity`. Prefabs saved before the split keep working unchanged
- Viewport debug draw for everything that owns geometry — primitives, mesh hulls,
  heightfield wireframes (bounded staleness when a provider mutates silently),
  compounds, characters, joint frames and limits, vehicle wheels
- Manipulators: the engine's own shape component modes for primitives, plus
  hand-written modes for joint frames and vehicle wheels, with undo batching
- A collision configuration window whose edits refresh every open inspector
- Debug visualization via `Physics::SystemDebugRequestBus::DebugDrawPhysics`, and the
  `jolt_Debug 1` console variable for per-frame collider wireframes

**Scripting**
- The gem's own gameplay buses are reflected to the behavior context, so Lua and
  ScriptCanvas can drive a vehicle (`JoltVehicleRequestBus`), tune and read a soft body
  (`JoltSoftBodyRequestBus`), drive and query a joint motor (`JoltJointRequestBus`, with
  a `JoltJointNotificationBus` handler for a joint breaking) and ask a character whether
  it is grounded, or make it jump (`JoltCharacterGameplayRequestBus`).
- **`Physics::RigidBodyRequestBus` is reflected by this gem** — velocities, impulses,
  mass, damping, sleep, kinematic and gravity control on any Jolt body. AzFramework
  declares that bus but leaves the script binding to a backend, and in 26.05 the only
  one that supplies it is the PhysX gem, which a Jolt project has to disable; without
  this the most-used physics script surface was missing entirely. The registration
  yields if something else claimed the name first.
- Scene queries and body enable/disable come from AzFramework's own reflection and
  work regardless.

**Determinism**
- Scene simulation state snapshot and restore (`SaveSimulationState` /
  `RestoreSimulationState`) covering bodies, contacts, constraints, vehicle drivetrain
  state and characters — the foundation for networked rollback and replay. Composition
  changes fail atomically, which Jolt alone does not guarantee

### Not wrapped yet

Jolt capabilities this gem does not expose; see the *Jolt features not wrapped* section
of [DIVERGENCES.md](DIVERGENCES.md) for the full list and the reasoning:

- Path and pulley constraints (path additionally needs spline authoring)
- Soft bodies from mesh assets — geometry is procedural, and cloth pinning is preset-based
- PhysX-*named* compatibility components; everything is `Jolt`-prefixed on purpose, so
  an incomplete backend cannot silently hijack PhysX-authored levels

## The gem family

The physics gem is deliberately renderer-free. Features that need Atom, or that Jolt
provides and this gem does not wrap, live in sibling gems that reach the backend through
`JoltPhysicsSystemRequests` — no changes to this gem required:

| Gem | Adds | Needs Atom |
|---|---|---|
| **JoltBuoyancy** | `Jolt Water Volume` — boxes of water that float, sink and drift rigid bodies | no |
| **JoltHair** | Jolt's GPU strand solver on Atom's DX12 device, rendered as GPU ribbons | **yes** |
| ~~JoltSoftBody~~ | migrated *into* this gem; the tree survives only for its history | — |

`JoltPhysicsSystemRequests::GetNativePhysicsSystem` is what makes this work: it hands
back a `JPH::PhysicsSystem*` for a scene, or null when the scene belongs to another
backend, so an extension gem does no harm in a project not running Jolt.

Cooked collision geometry crosses the same line. `JoltPhysics/Pipeline/JoltMeshAsset.h`
is public, so a sibling gem can load an `AZ::Data::Asset<Pipeline::JoltMeshAsset>` — the
`.joltmesh` product the Asset Processor already built and deduplicated — instead of
cooking its own; `GetColliderShapesFromMeshAsset` on the same bus expands one into
collider/shape pairs exactly as the Jolt Mesh Collider does, and the engine's
`Physics::SystemRequestBus::CreateShape` turns each into a shape. The expansion is on the
bus rather than beside the asset type because `JoltPhysics.API` is an INTERFACE target:
there is no library to link, so anything not inline has to be dispatched.

### Writing an extension gem: install the Jolt globals

**Jolt is statically linked into every module that uses it, so each one owns a private
copy of Jolt's globals — and the ones that matter start null.** This gem installing its
copy does nothing for yours. Missing the allocation hooks is a jump through a null
pointer on a physics job thread; missing `JPH::Factory::sInstance` is an access violation
inside `Factory::Find`. Both cost real debugging to find the first time.

Link `Gem::JoltPhysics.API` and call the installer from **both** module entry points:

```cpp
#include <JoltPhysics/JoltModuleGlobals.h>

MyGemModule::MyGemModule()          { JoltPhysics::InstallJoltModuleGlobals(); /* ... */ }
MyGemModule::~MyGemModule()         { JoltPhysics::UninstallJoltModuleGlobals(); }
```

Two things it is doing for you. The hooks forward to `AZ::SystemAllocator`, byte for byte
what this gem installs, because allocations cross the module boundary — your
`AddStepListener` grows an array that this gem's `~PhysicsSystem` frees, so
`JPH::RegisterDefaultAllocator` would hand it a malloc block `AZ::SystemAllocator` never
issued. And tearing down from the module destructor rather than static destruction keeps
Jolt's globals dying while the AZ allocators are still alive; the other way round the
process aborts silently on exit.

A module that forgets the call still crashes, and no unit test in another module can
catch that — so keep it in the entry point, not somewhere clever.

## Requirements

- **O3DE 26.05** (engine version `2.6.0`) — the gem is pinned to this engine release; see [BUILDING.md](BUILDING.md) for the exact setup.
- **Jolt Physics v5.6.0**, fetched automatically at configure time (`JOLTPHYSICS_GIT_TAG` in `Code/CMakeLists.txt`)
- CMake 3.23+, Ninja
- Windows 11 + Visual Studio 2022 (primary dev target) or Linux (Ubuntu 20.04+, must not break)
- C++20 compatible compiler

## Installation

### Adding the Gem to Your Project

1. Clone this repository into your O3DE project's `Gems` directory or the engine's `Gems` directory:

```bash
cd /path/to/your/o3de-project/Gems
git clone https://github.com/your-repo/JoltPhysics.git
```

2. Register the Gem with your project using the O3DE CLI:

```bash
# From your project directory
o3de register --gem-path Gems/JoltPhysics
```

3. Enable the Gem in your project:

```bash
o3de enable-gem --gem-name JoltPhysics --project-path /path/to/your/project
```

Or use Project Manager:
- Open O3DE Project Manager
- Select your project
- Click "Configure Gems"
- Find "Jolt Physics" and enable it

### Disabling PhysX Gem

To avoid conflicts, you must disable the PhysX Gem when using JoltPhysics:

```bash
o3de disable-gem --gem-name PhysX --project-path /path/to/your/project
```

Or in Project Manager:
- Find "PhysX" in the Gems list
- Disable it

## Configuration

### Settings Registry

The Gem uses O3DE's Settings Registry for configuration. Defaults ship in
`Registry/joltphysics.setreg`, which also seeds the default collision layers and groups:

```json
{
    "O3DE": {
        "Physics": {
            "DefaultBackend": "JoltPhysics",
            "JoltPhysics": {
                "Enabled": true,
                "SystemConfiguration": {
                    "MaxBodies": 65536,
                    "NumBodyMutexes": 128,
                    "MaxBodyPairs": 65536,
                    "MaxContactConstraints": 16384,
                    "TempAllocatorSize": 268435456,
                    "MaxJobThreads": 0,
                    "FixedTimestep": 0.0166667
                },
                "DefaultSceneConfiguration": { "...": "gravity, CCD, PCM, ..." },
                "CollisionConfiguration": { "...": "layers and groups" }
            }
        }
    }
}
```

### CMake Options

When building, you can customize Jolt integration:

- `JOLTPHYSICS_USE_EXTERNAL=OFF` - Use FetchContent (default) or external Jolt installation
- `JOLTPHYSICS_EXTERNAL_PATH=""` - Path to external Jolt installation if `JOLTPHYSICS_USE_EXTERNAL=ON`

## Building and testing

See [BUILDING.md](BUILDING.md) for the exact, tested build commands for Windows and Linux,
including how the gem is registered and how the unit tests are run.

The suites are the gem's safety net and are expected to be green before any commit:

| Suite | Tests |
|---|---|
| `JoltPhysics.Tests.dll` | 212 |
| `JoltPhysics.Editor.Tests.dll` | 49 |

```bat
cd <project>\build\windows\bin\profile
AzTestRunner.exe JoltPhysics.Tests.dll AzRunUnitTests
AzTestRunner.exe JoltPhysics.Editor.Tests.dll AzRunUnitTests
```

Check the process exit code, not the console text — a suite that aborts during teardown
still prints `PASSED` lines above the failure.

## Verifying Jolt is Active

### Method 1: Check Console Output

When the engine starts, look for:

```
JoltPhysics: System initialized with X threads
```

### Method 2: Draw the colliders

In the O3DE Editor or game console:

```
jolt_Debug 1
```

Every Jolt collider in the scene draws as a wireframe each frame. Nothing appears if
another backend is simulating.

### Method 3: Check Component Services

The JoltPhysicsSystemComponent provides:
- `PhysicsService`
- `JoltPhysicsService`

And is incompatible with:
- `PhysXService`

### Method 4: The manual checklist

Mostly covered by the unit suites and the smoke level below, but useful when bringing the
gem up in a new project: a static box that must not move; a dynamic box that must fall and
rest on it; each primitive collider colliding; a raycast finding a body; a scripted gravity
change taking effect; linear/angular velocity and an impulse producing the motion expected.

## Architecture

```
JoltPhysics/
├── Code/
│   ├── Include/JoltPhysics/        # Public API headers (buses extension gems use)
│   ├── Source/
│   │   ├── Character/              # Character controller + ragdolls
│   │   ├── Clients/                # SystemComponent and runtime components
│   │   ├── Configuration/          # Settings management
│   │   ├── Debug/                  # Debug draw
│   │   ├── Editor/                 # Editor components, Scene Builder pipeline plugins,
│   │   │                           #   component modes, viewport draw
│   │   ├── Joint/                  # Constraint wrapping
│   │   ├── Material/               # Physics materials
│   │   ├── Pipeline/               # .joltmesh asset type and handler
│   │   ├── RigidBody/              # Rigid body implementations
│   │   ├── Scene/                  # Scene/World management, state snapshots
│   │   ├── Shape/                  # Shape, mesh cooking and heightfield utilities
│   │   ├── SoftBody/               # Soft body simulation and render sync
│   │   ├── System/                 # Core Jolt integration
│   │   ├── Utils/                  # Conversion utilities
│   │   └── Vehicle/                # Vehicle constraint wrapping
│   ├── Tests/                      # Unit tests
│   └── Platform/                   # Platform-specific code
├── Registry/                       # Settings registry files
├── DIVERGENCES.md                  # Every intentional difference from PhysX
├── gem.json                        # Gem metadata
└── CMakeLists.txt                  # Build configuration
```

## Milestone Status

| Milestone | Scope | Status |
|---|---|---|
| M0 | Baseline, O3DE 26.05 port, build health, smoke level | ✅ Done |
| M1 | Jolt upgrade (now pinned to v5.6.0) | ✅ Done |
| M2 | Stabilize existing features (materials, filtering, queries, debug draw) | ✅ Done |
| M3 | Compound colliders | ✅ Done |
| M4 | Heightfield collider | ✅ Done |
| M5 | Character controllers | ✅ Done |
| M6 | Joints | ✅ Done |
| M7 | Vehicles | ✅ Done |
| M8 | Soft bodies (in-gem); water via the JoltBuoyancy gem | ✅ Done |
| M9 | Editor parity — viewport draw, manipulators, component modes | ✅ Done |
| M10 | Simulation state snapshots (rollback foundation) | ✅ Done |

## Test project levels

The reference test project (`JoltPhysicsTest`) carries a level per feature area:
`SmokeBox` (below), `SoftBody`, `Buoyancy` (JoltBuoyancy gem) and `Hair` (JoltHair gem).

### Smoke test level

`Levels/SmokeBox/SmokeBox.prefab` (generated by `gen_smoke_level.py` in the project root):

- **Ground** entity (from the template's default level): `Jolt Static Rigid Body` +
  `Jolt Box Collider` (512×512×1 m, collider offset z=-0.5 so its top surface is z=0).
- **FallingBox** entity at (0, 0, 3): `Jolt Rigid Body` + `Jolt Box Collider` (1 m) + visible box mesh.
- **FallingSphere / FallingCapsule** at (±3, 0, 4): primitive collider coverage.
- **KinematicPlatform** at (6, 0, 2): kinematic rigid body (must hover forever).
- **TriggerVolume** at (0, 0, 1): static trigger box (4×4×2 m) — enter/exit events only.
- **CompoundBody** at (8, 0, 4): `Jolt Mutable Compound Collider` + `Jolt Rigid Body`
  gathering two child-entity box colliders (±1 m on x) into one rigid body.
- **Terrain** at (0, 0, 0): `Jolt Heightfield Collider` + `Jolt Static Rigid Body` +
  `HeightfieldProviderTestComponent` (from the test project's gem): a 16×16 grid,
  1 m spacing, slope descending from z=3 at x=0 to z=0 at x≥6, flat afterwards.
  After 2.5 simulated seconds it raises the region x∈[6,10), y∈[-12,-4] by +2 m
  over ~1 s, firing heightfield-change notifications (runtime update coverage).
- **SlopeBall** at (4, -4, 5): sphere that must land on the slope and roll downhill (+x).
- **StepBall** at (8, -8, 6): sphere resting in the step region; it must ride the
  raising terrain up and come to rest at z≈2.48.
- **Step1–Step3 / Platform** (y=6 lane): static stair steps (0.3 m increments) up to a
  0.9 m platform, exercising the character's step offset.
- **PushBox** at (5, 6, 1.4): dynamic box on the platform the character pushes aside.
- **Player** at (0, 6, 1): `Jolt Character Controller` + `CharacterDriverTestComponent`
  (from the test project's gem): walks +x at 1.5 m/s, climbs the stairs, jumps at t=3 s,
  shoves the box aside, and comes to a stand mid-platform.
- **PendulumAnchor / PendulumBob** (y=12–14): `Jolt Hinge Joint` (unlimited) with the
  pivot at the anchor; the bob is released 2 m to the side and swings in the YZ plane.
- **ChainAnchor / ChainLink1 / ChainLink2** (y=16): two `Jolt Ball Joint`s (60° cone)
  forming a ragdoll-like chain that dangles below the anchor.
- **DoorFrame / SlidingDoor** (y=20): `Jolt Prismatic Joint` (limits 0–1.5 m, motor)
  + `JointDriverTestComponent` driving the door open to its limit; the frame slab
  floats above the door so the two jointed bodies do not overlap.
- **Vehicle** at (0, 24, 0.9): `Jolt Rigid Body` chassis + `Jolt Vehicle` (default
  4-wheel layout) + `VehicleDriverTestComponent`: launches over a 12° ramp
  (**VehicleRamp**/**VehiclePlatform**), steers gently, and brakes to a stop.

To reproduce: open the level in the O3DE Editor, press **Ctrl+G** (game mode).
The box falls and comes to rest with its center at z≈0.48 m (0.5 m half-extent minus
the 2 cm contact offset) and stays there; the slope ball rolls downhill; the step ball
is lifted ~2 m by the runtime heightfield update. The automated variant
(`smoke_test.py` in the project root) runs 10 simulated seconds and asserts all of this:

```bat
cd C:\O3DE\26.05\bin\Windows\profile\Default
Editor.exe --project-path=C:\path\to\JoltPhysicsTest -BatchMode -autotest_mode --runpython C:\path\to\JoltPhysicsTest\smoke_test.py
```

Result is written to `smoke_result.txt` in the project root (`RESULT: PASS`).

## API Compatibility

This Gem implements the standard O3DE AzPhysics interfaces:

- `AzPhysics::SystemInterface` - Physics system management
- `AzPhysics::Scene` - Scene/world simulation
- `AzPhysics::RigidBody` - Dynamic rigid bodies
- `AzPhysics::StaticRigidBody` - Static rigid bodies
- `Physics::SystemRequestBus` - Physics system requests
- `Physics::CollisionRequestBus` - Collision layer/group management
- `Physics::Ragdoll` - Ragdoll physics

Existing O3DE physics components should work without modification, with the documented
exceptions in [DIVERGENCES.md](DIVERGENCES.md) — component *names* being the deliberate
one: they are `Jolt`-prefixed, so a PhysX-authored level does not silently switch backend.

## License

This Gem is released under the MIT License. See LICENSE file for details.

Jolt Physics is also released under the MIT License. See [Jolt Physics License](https://github.com/jrouwe/JoltPhysics/blob/master/LICENSE).

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## References

- [O3DE Documentation](https://www.o3de.org/docs/)
- [Jolt Physics Documentation](https://jrouwe.github.io/JoltPhysics/)
- [O3DE Gem Development Guide](https://www.o3de.org/docs/user-guide/gems/)
