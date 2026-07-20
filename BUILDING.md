# Building the JoltPhysics Gem

## Prerequisites

- **O3DE 26.05** (engine `o3de-sdk`, engine version `2.6.0`), installed at `C:\O3DE\26.05`.
  This is the only engine version the gem is currently pinned to.
- Visual Studio 2022 (Community or higher) with the C++ workload.
- CMake >= 3.23 and Ninja. The Visual Studio-bundled ones work:
  `C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\{CMake,Ninja}`.
- Git (for FetchContent of Jolt Physics).

## Register the gem

```bat
C:\O3DE\26.05\scripts\o3de.bat register --gem-path C:\Users\jorge\O3DE\Gems\JoltPhysics
```

## Create / configure a project

The gem **replaces PhysX**: disable the PhysX gems before enabling JoltPhysics.

```bat
C:\O3DE\26.05\scripts\o3de.bat create-project --project-path C:\path\to\YourProject
C:\O3DE\26.05\scripts\o3de.bat disable-gem --project-path C:\path\to\YourProject --gem-name PhysX5
C:\O3DE\26.05\scripts\o3de.bat enable-gem  --project-path C:\path\to\YourProject --gem-path C:\Users\jorge\O3DE\Gems\JoltPhysics
```

(The reference test project used by the maintainers is `C:\Users\jorge\O3DE\Projects\JoltPhysicsTest`,
created from the `DefaultProject` template with `PhysX5` disabled and `JoltPhysics` enabled.)

## Configure and build (Windows, Ninja, profile)

CMake must run inside a Visual Studio developer environment. From a
`x64 Native Tools Command Prompt` (or via a wrapper that calls
`vcvars64.bat` and adds the VS-bundled CMake/Ninja to `PATH` first):

```bat
cd C:\path\to\YourProject
cmake -B build/windows -G Ninja -DCMAKE_BUILD_TYPE=profile -S .
cmake --build build/windows --target YourProject.GameLauncher Editor JoltPhysics JoltPhysics.Editor
```

The first configure clones Jolt Physics (currently `v5.5.0`) via CMake FetchContent;
all other dependencies come from the O3DE SDK.

The same commands run from Git Bash in this repo's dev environment via the helper
`build-env.cmd` checked into the test project root, e.g.:

```bash
cmd //c "build-env.cmd cmake --build build/windows --target JoltPhysics JoltPhysics.Editor"
```

## Run the unit tests

```bat
cmake --build build/windows --target JoltPhysics.Tests JoltPhysics.Editor.Tests
ctest --test-dir build/windows -R JoltPhysics --output-on-failure
```

Expected: 2 CTest entries (`Gem::JoltPhysics.Tests`, `Gem::JoltPhysics.Editor.Tests`), all passing.

## Linux

The same flow works with the Linux engine and generator (`-G "Ninja Multi-Config"` or
single-config Ninja). Linux is a supported-but-secondary target; CI is Windows-first.

## CMake options

- `JOLTPHYSICS_USE_EXTERNAL=OFF` — use FetchContent (default) or an external Jolt installation.
- `JOLTPHYSICS_EXTERNAL_PATH=""` — path to the external Jolt install when the above is `ON`.
