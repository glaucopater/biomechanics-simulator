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

**With HTTP control** – enable the REST API on a local port (e.g. for automation or external clients):

```bash
.\build\Release\biomechanics_simulator.exe --http-port 8765   # Windows
./build/biomechanics_simulator --http-port 8765              # Linux/macOS
```

Then use `GET http://127.0.0.1:8765/status`, `PATCH http://127.0.0.1:8765/stance`, `POST http://127.0.0.1:8765/stance/standing_raise_leg`, etc. OpenAPI spec: `GET http://127.0.0.1:8765/openapi.yaml` (when run from repo root or build dir). See [docs/openapi.yaml](docs/openapi.yaml) for the full API.

**Headless** – no window; runs a fixed number of steps and prints pelvis position (for CI/automation):

```bash
.\build\Release\biomechanics_simulator.exe --headless   # Windows
./build/biomechanics_simulator --headless              # Linux/macOS
```

## Stance and movement (visualizer)

The model supports **standing**, **raise leg**, **walking**, and **ragdoll**:

- **Standing** – Pose control holds the ragdoll upright (default when the window opens). Root is pinned.
- **Raise leg** – Static stance with one leg raised; root pinned, other limbs driven by motors.
- **Walking** – Cyclic gait: hip/knee/arm targets drive a walking motion.
- **Ragdoll** – No pose control; full physics.
- **Jump** – One-shot upward impulse on the pelvis.

**Stance panel (top-left):** **Standing**, **Raise leg**, **Walk**, **Ragdoll**, **Jump**, **Reset**, **Test (float 2s)**, **Freeze/Unfreeze**.

**Keys (with window focused):**

| Key   | Action   |
|-------|----------|
| **1** | Standing |
| **2** | Walking  |
| **3** | Ragdoll  |
| **4** | Raise leg |
| **Space** | Jump     |

**Camera:** Left mouse drag to orbit; scroll to zoom.

## What it does

- Creates a Jolt `PhysicsSystem` with gravity.
- Adds a ground plane and a single ragdoll (pelvis, spine, head, arms, legs) from capsule shapes and constraints (Human.tof when available, else procedural).
- **Default**: runs with a visualizer window; simulation in real time, wireframe rendering. Starts in **Standing**. Stance panel and keys **1**–**4** / **Space** switch mode; **Reset** restores initial standing. **Camera**: left-drag to orbit, scroll to zoom.
- **`--http-port PORT`**: starts an HTTP server on `127.0.0.1:PORT` with REST endpoints for stance, status, log, position, and actions. OpenAPI 3.0 spec at `GET /openapi.yaml` (and in [docs/openapi.yaml](docs/openapi.yaml)) for client integration.
- **`--headless`**: runs a fixed number of steps with pose control (standing by default), prints pelvis position and exits.

## Project layout

- `CMakeLists.txt` – root build; Jolt, GLFW, ImGui (FetchContent); `biomechanics_simulator`, optional tests.
- `vcpkg.json` – vcpkg manifest (optional).
- `include/biomechanics/` – public API: `Config.hpp`, `HttpControl.hpp`, `JoltLayers.hpp`, `Log.hpp`, `PoseController.hpp`, `Ragdoll.hpp`, `Simulator.hpp`, `SimulatorScene.hpp`, `Visualizer.hpp`, `OpenGLDebugDrawer.hpp`, etc.
- `src/main.cpp` – entry point; parses `--headless`, `--http-port`, calls `run_demo_visual()` (default) or `run_demo()`.
- `src/PoseController.cpp` – stance logic: standing, standing raise-leg, walking, ragdoll; jump impulse.
- `src/Visualizer.cpp` – GLFW window, orbit camera, ImGui stance panel, key bindings (1–4, Space), step-and-draw loop.
- `src/HttpControl.cpp` – HTTP server: `/status`, `/log`, `/stance`, `/stance/standing`, `/stance/standing_raise_leg`, `/stance/walking`, `/stance/ragdoll`, `/jump`, `/reset`, `/position`, `/openapi.yaml`.
- `src/SimulatorScene.cpp` – scene setup (ground, ragdoll from Human.tof or procedural).
- `src/HumanRagdoll.cpp` – Human.tof loading, procedural human rig.
- `src/OpenGLDebugDrawer.cpp` – wireframe rendering of Jolt bodies.
- `docs/openapi.yaml` – OpenAPI 3.0 spec for the HTTP API.
- `docs/PROJECT_STRUCTURE.md`, `docs/MILESTONES.md` – layout and milestones.
- `tests/` – optional tests; build with `-DBUILD_TESTS=ON`.

## License

See [LICENSE](LICENSE).
