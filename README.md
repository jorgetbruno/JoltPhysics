# JoltPhysics Gem for O3DE

A standalone O3DE Gem that integrates [Jolt Physics](https://github.com/jrouwe/JoltPhysics) as an AzPhysics backend, providing an alternative to the default PhysX Gem with full API compatibility.

## Features

### Implemented
- Core physics system integration via `AzPhysics::SystemInterface`
- Scene/World management with simulation stepping, incl. the game default world
- Dynamic and static rigid bodies with primitive colliders (Box, Sphere, Capsule)
- Editor components: Jolt Box/Sphere/Capsule Collider, Jolt Rigid Body, Jolt Static Rigid Body
- Rigid body buses (`Physics::RigidBodyRequestBus`, `AzPhysics::SimulatedBodyComponentRequestsBus`)
- Raycast queries through O3DE's physics query API
- Collision layers and groups (system level)
- Gravity control per scene
- Body activation/deactivation
- Linear and angular velocity control, impulse application
- Collider offset/rotation and multi-collider compound shapes in the backend

### Planned / TODO
- Character controllers
- Heightfield colliders
- Convex hull and mesh colliders (cooking)
- Compound collider components (multiple colliders per entity)
- Joints (fixed, hinge, slider, etc.)
- Shape cast and overlap queries
- Async scene queries
- Physics materials (friction/restitution) on colliders
- Trigger colliders
- Soft bodies
- Vehicle simulation
- Debug visualization

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
| M4 | Heightfield collider | ⬜ Planned |
| M5 | Character controllers | ⬜ Planned |
| M6 | Joints | ⬜ Planned |
| M7 | Vehicles | ⬜ Planned |
| M8 | Soft bodies, water | ⬜ Planned |

## Smoke Test Level

The reference test project `C:\Users\jorge\O3DE\Projects\JoltPhysicsTest` contains
`Levels/SmokeBox/SmokeBox.prefab` (generated by `gen_smoke_level.py` in the project root):

- **Ground** entity (from the template's default level): `Jolt Static Rigid Body` +
  `Jolt Box Collider` (512×512×1 m, collider offset z=-0.5 so its top surface is z=0).
- **FallingBox** entity at (0, 0, 3): `Jolt Rigid Body` + `Jolt Box Collider` (1 m) + visible box mesh.

To reproduce: open the level in the O3DE Editor, press **Ctrl+G** (game mode).
The box falls and comes to rest with its center at z≈0.48 m (0.5 m half-extent minus
the 2 cm contact offset) and stays there. The automated variant
(`smoke_test.py` in the project root) runs 10 simulated seconds and asserts exactly this:

```bat
cd C:\O3DE\26.05\bin\Windows\profile\Default
Editor.exe --project-path=C:\Users\jorge\O3DE\Projects\JoltPhysicsTest -BatchMode -autotest_mode --runpython C:\Users\jorge\O3DE\Projects\JoltPhysicsTest\smoke_test.py
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
