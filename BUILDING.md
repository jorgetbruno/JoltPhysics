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

## Configure and build (Windows, Visual Studio generator, profile)

The default generator on Windows is `Visual Studio 17 2022`, which resolves the MSVC
toolchain itself — no developer environment needed, and builds from the **O3DE Project
Manager work out of the box**:

```bat
cd C:\path\to\YourProject
cmake -B build/windows -S .
cmake --build build/windows --config profile --target YourProject.GameLauncher Editor JoltPhysics JoltPhysics.Editor
```

(If `cmake` is not on `PATH`, use the full path to the VS-bundled one:
`C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe`.)

The first configure clones Jolt Physics (currently `v5.5.0`) via CMake FetchContent;
all other dependencies come from the O3DE SDK.

Build outputs land in `build/windows\profile\...` (multi-config layout).

### Optional: Ninja (faster CLI builds)

Ninja needs a Visual Studio developer environment (`vcvars64.bat`); use a separate
build dir so the Project Manager keeps working on the VS one:

```bat
cmake -B build/windows-ninja -G Ninja -DCMAKE_BUILD_TYPE=profile -S .
cmake --build build/windows-ninja --target YourProject.GameLauncher Editor JoltPhysics JoltPhysics.Editor
```

(The test project's `build-env.cmd` wrapper sets up that environment.)

## Run the unit tests

```bat
cmake --build build/windows --config profile --target JoltPhysics.Tests JoltPhysics.Editor.Tests
ctest --test-dir build/windows -C profile -R JoltPhysics --output-on-failure
```

Expected: 2 CTest entries (`Gem::JoltPhysics.Tests`, `Gem::JoltPhysics.Editor.Tests`), all passing.

## Building from the O3DE Project Manager

With the Visual Studio generator (the default flow above), the Project Manager's
**Build Project** works out of the box: it spawns CMake without `-G`, which reuses
the cached VS generator, and MSBuild resolves the toolchain without a developer
environment.

If the build cache was created with Ninja instead, GUI builds fail at the compiler
check with `LNK1104: cannot open file 'kernel32.lib'` (Ninja needs the Windows SDK
`LIB`/`INCLUDE` paths that only exist after `vcvars64.bat`). Fix by either deleting
`build/windows` and reconfiguring without `-G` (back to the VS default), or by
launching the Project Manager from a `x64 Native Tools Command Prompt` so spawned
processes inherit the environment.

## Linux

The same flow works with the Linux engine and generator (`-G "Ninja Multi-Config"` or
single-config Ninja). Linux is a supported-but-secondary target; CI is Windows-first.

## CMake options

- `JOLTPHYSICS_USE_EXTERNAL=OFF` — use FetchContent (default) or an external Jolt installation.
- `JOLTPHYSICS_EXTERNAL_PATH=""` — path to the external Jolt install when the above is `ON`.
