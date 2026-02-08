# Biomechanics Simulator (C++)

Minimal C++ biomechanics simulator with ragdoll physics using [Jolt Physics](https://github.com/jrouwe/JoltPhysics): rigid bodies (capsules for limbs), 6-DOF joint constraints, and a dynamics world. Jolt is used in Horizon Forbidden West and Death Stranding 2.

## Requirements

- **C++17** compiler (MSVC, GCC, or Clang)
- **CMake** 3.16+
- **Git** (for FetchContent; or use vcpkg)
- **GLFW** (fetched automatically via FetchContent; or from vcpkg) for the visualizer window
- **Dear ImGui** (fetched automatically via FetchContent) for the stance UI panel

## Build

**Option A – CMake only (Jolt fetched automatically)**  
From the project root:

```bash
cmake -B build
cmake --build build
```

On Windows with Visual Studio: `cmake -B build -G "Visual Studio 17 2022" -A x64` then open `build/biomechanics_simulator.sln` or run `cmake --build build --config Release`.

**Option B – vcpkg**  
Jolt is built from source via FetchContent; vcpkg is optional for other deps. If using vcpkg:

```bash
cmake -B build -DCMAKE_TOOLCHAIN_FILE=[vcpkg]/scripts/buildsystems/vcpkg.cmake
cmake --build build
```

Replace `[vcpkg]` with your vcpkg root (e.g. `C:/vcpkg` or `$HOME/vcpkg`).

## Run

From the project root, after building:

**With window (default)** – opens a GLFW/OpenGL window; close the window to exit.

```bash
# Windows (PowerShell or cmd)
.\build\Release\biomechanics_simulator.exe

# Windows Debug
.\build\Debug\biomechanics_simulator.exe

# Linux / macOS
./build/biomechanics_simulator
```

**Headless** – no window; runs a fixed number of steps and prints pelvis position (for CI/automation):

```bash
.\build\Release\biomechanics_simulator.exe --headless   # Windows
./build/biomechanics_simulator --headless              # Linux/macOS
```

## Stance and movement (visualizer)

The model has default **standing**, **walking**, and **jumping**:

- **Standing** – PD control holds the ragdoll in an upright rest pose (default when the window opens).
- **Walking** – Cyclic gait: hip/knee/arm targets drive a walking motion.
- **Jump** – One-shot upward impulse on the pelvis.

**Stance panel (top-left):** Use **Standing**, **Walk**, **Ragdoll**, **Jump**, and **Reset** to change mode or reset the simulation to the default standing pose.

**Keys (with window focused):**

| Key   | Action   |
|-------|----------|
| **1** | Standing |
| **2** | Walking  |
| **3** | Ragdoll (no pose control) |
| **Space** | Jump     |

**Camera:** Drag with the **left mouse button** to orbit; **scroll** to zoom in/out.

## What it does

- Creates a Jolt `PhysicsSystem` with gravity.
- Adds a ground plane and a single ragdoll (pelvis, spine, head, arms, legs) built from capsule shapes and `SixDOFConstraint` joints.
- **Default**: runs with a visualizer window; simulation steps in real time and the body is drawn as a wireframe. Starts in **Standing**. Use the on-screen **Stance** panel (Standing / Walk / Ragdoll / Jump / Reset) or keys **1** / **2** / **3** / **Space**. **Reset** restores the scene to the initial standing pose. **Camera**: left-drag to orbit, scroll to zoom. Close the window to exit.
- **`--headless`**: steps the simulation 300 times at 60 Hz with pose control (standing by default; set `default_motion_mode` in config for walking/ragdoll), prints the pelvis position and "Done.", then exits.

## Project layout

- `CMakeLists.txt` – root build; Jolt and GLFW (FetchContent), `biomechanics_simulator`, optional tests.
- `vcpkg.json` – vcpkg manifest (optional; Jolt is fetched from source).
- `include/biomechanics/` – public API: `Config.hpp`, `JoltLayers.hpp`, `Ragdoll.hpp`, `Simulator.hpp`, `SimulatorScene.hpp`, `Visualizer.hpp`, `OpenGLDebugDrawer.hpp`, `PoseController.hpp`.
- `src/main.cpp` – entry point; parses `--headless`, calls `run_demo_visual()` (default) or `run_demo()`.
- `src/Ragdoll.cpp` – ragdoll creation (Jolt capsules + SixDOF constraints).
- `src/Simulator.cpp` – headless demo: physics, ground, pose control, step loop, cleanup (uses `SimulatorScene`).
- `src/SimulatorScene.cpp` – shared scene setup: `create_simulator_scene()`, `destroy_simulator_scene()`.
- `src/PoseController.cpp` – stance/walking/jump: PD control toward rest pose or cyclic gait; jump impulse.
- `src/Visualizer.cpp` – visual mode: GLFW window, orbit/zoom camera (mouse drag + scroll), ImGui stance panel (Stand/Walk/Ragdoll/Jump), OpenGL debug drawer, key bindings (1/2/3/Space), step-and-draw loop.
- `src/OpenGLDebugDrawer.cpp` – wireframe rendering of Jolt bodies (capsules, box).
- `tests/` – optional smoke test; build with `-DBUILD_TESTS=ON`.
- `docs/PROJECT_STRUCTURE.md` – folder layout and roles.

## License

See [LICENSE](LICENSE).
