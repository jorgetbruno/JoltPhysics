# JoltPhysics Gem for O3DE

A standalone O3DE Gem that integrates [Jolt Physics](https://github.com/jrouwe/JoltPhysics) as an AzPhysics backend, providing an alternative to the default PhysX Gem with full API compatibility.

## Features

### Implemented
- Core physics system integration via `AzPhysics::SystemInterface`
- Scene/World management with simulation stepping, incl. the game default world
- Dynamic and static rigid bodies with primitive colliders (Box, Sphere, Capsule)
- Compound colliders: multiple colliders per entity + static/mutable compound collider
  components that gather child-entity colliders, with per-sub-shape materials
- Heightfield collider fed by `Physics::HeightfieldProviderRequestsBus` (terrain gems),
  incl. runtime height updates and per-triangle materials
- Character controller (`JoltCharacterControllerComponent` over `JPH::CharacterVirtual`):
  ground detection, slope limits, step offset, stick-to-floor, pushing dynamic bodies,
  trigger/sensor interaction
- Joints: fixed, hinge (limits + motor), ball-and-socket (swing cone), prismatic
  (limits + motor) and 6-DOF (swing/twist limits) components mapped to Jolt constraints
- Wheeled vehicles (`JoltVehicleComponent` over `JPH::VehicleConstraint`): engine,
  automatic transmission, steering, brakes, suspension, ramp driving
- Editor components for every feature (PhysX-style editor/runtime split): the
  `EditorJolt*` components are what the Add Component menu offers; they draw
  collider wireframes in the Edit viewport and spawn the runtime components via
  `BuildGameEntity` (colliders, rigid bodies, heightfield, compounds, character,
  vehicle, joints). Prefabs saved before the split keep working unchanged.
- Rigid body buses (`Physics::RigidBodyRequestBus`, `AzPhysics::SimulatedBodyComponentRequestsBus`)
- Scene queries: raycast, shapecast and overlap through O3DE's physics query API
- Physics materials (friction/restitution) on colliders
- Trigger (sensor) colliders with enter/exit events
- Collision layers and groups (system level + per-collider filtering)
- Gravity control per scene
- Body activation/deactivation
- Linear and angular velocity control, impulse application
- Debug visualization via `Physics::SystemDebugRequestBus::DebugDrawPhysics`

### Planned / TODO
- Convex hull and mesh colliders (cooking)
- Async scene queries
- Soft bodies

## Requirements

- **O3DE 26.05** (engine version `2.6.0`) — the gem is pinned to this engine release; see [BUILDING.md](BUILDING.md) for the exact setup.
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

The Gem uses O3DE's Settings Registry for configuration. Default settings are in `Registry/joltphysics.setreg`:

```json
{
    "O3DE": {
        "Physics": {
            "DefaultBackend": "JoltPhysics",
            "JoltPhysics": {
                "Enabled": true,
                "SystemConfiguration": {
                    "MaxBodies": 65536,
                    "MaxJobThreads": 0,
                    "FixedTimestep": 0.0166667
                }
            }
        }
    }
}
```

### CMake Options

When building, you can customize Jolt integration:

- `JOLTPHYSICS_USE_EXTERNAL=OFF` - Use FetchContent (default) or external Jolt installation
- `JOLTPHYSICS_EXTERNAL_PATH=""` - Path to external Jolt installation if `JOLTPHYSICS_USE_EXTERNAL=ON`

## Building

See [BUILDING.md](BUILDING.md) for the exact, tested build commands for Windows and Linux,
including how the gem is registered and how the unit tests are run.

## Verifying Jolt is Active

### Method 1: Check Console Output

When the engine starts, look for these log messages:

```
JoltPhysics: System initialized with X threads
JoltPhysics: Scene 'DefaultPhysicsScene' initialized
```

### Method 2: Use Debug Commands

In the O3DE Editor console:

```
physics_GetBackendName
```

Should return "JoltPhysics"

### Method 3: Check Component Services

The JoltPhysicsSystemComponent provides:
- `PhysicsService`
- `JoltPhysicsService`

And is incompatible with:
- `PhysXService`

## Test Level Checklist

To verify the physics backend is working correctly:

1. **Static Body Test**
   - Create an entity with a Box Collider (static)
   - Verify it doesn't move when simulation runs

2. **Dynamic Body Test**
   - Create an entity with a Rigid Body + Box Collider
   - Place it above a static floor
   - Run simulation - body should fall and rest on floor

3. **Primitive Shapes Test**
   - Test Box, Sphere, Capsule, and Cylinder colliders
   - Verify collision detection works for each

4. **Raycast Test**
   - Create a script that performs raycasts
   - Verify hits are detected against physics bodies

5. **Gravity Test**
   - Modify scene gravity via script
   - Verify dynamic bodies respond to gravity changes

6. **Velocity Test**
   - Apply linear/angular velocity to a rigid body via script
   - Verify body moves/rotates accordingly

7. **Impulse Test**
   - Apply impulse to a rigid body
   - Verify instantaneous velocity change

## Architecture

```
JoltPhysics/
├── Code/
│   ├── Include/JoltPhysics/        # Public API headers
│   ├── Source/
│   │   ├── Clients/                # SystemComponent
│   │   ├── Configuration/          # Settings management
│   │   ├── Editor/                 # Editor integration
│   │   ├── RigidBody/              # Rigid body implementations
│   │   ├── Scene/                  # Scene/World management
│   │   ├── Shape/                  # Shape utilities
│   │   ├── System/                 # Core Jolt integration
│   │   └── Utils/                  # Conversion utilities
│   ├── Tests/                      # Unit tests
│   └── Platform/                   # Platform-specific code
├── Registry/                       # Settings registry files
├── gem.json                        # Gem metadata
└── CMakeLists.txt                  # Build configuration
```

## Milestone Status

| Milestone | Scope | Status |
|---|---|---|
| M0 | Baseline, O3DE 26.05 port, build health, smoke level | ✅ Done |
| M1 | Jolt 5.5.0 upgrade | ✅ Done |
| M2 | Stabilize existing features (materials, filtering, queries, debug draw) | ✅ Done |
| M3 | Compound colliders | ✅ Done |
| M4 | Heightfield collider | ✅ Done |
| M5 | Character controllers | ✅ Done |
| M6 | Joints | ✅ Done |
| M7 | Vehicles | ✅ Done |
| M8 | Soft bodies, water | ⬜ Planned |

## Smoke Test Level

The reference test project `C:\path\to\JoltPhysicsTest` contains
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

Existing O3DE physics components should work without modification.

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
