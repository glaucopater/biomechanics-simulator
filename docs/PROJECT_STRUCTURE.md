# Project structure

```
biomechanics-simulator/
├── CMakeLists.txt          # Root build: Jolt + GLFW (FetchContent), main app, optional tests
├── vcpkg.json              # vcpkg manifest placeholder (deps fetched via CMake FetchContent)
├── include/
│   └── biomechanics/       # Public API
│       ├── Config.hpp      # SimulatorConfig (gravity, time step, ragdoll, window size, etc.)
│       ├── DrawableBodyCount.hpp  # count_drawable_bodies() – test helper
│       ├── HttpControl.hpp # REST API for stance/status/log control
│       ├── JoltLayers.hpp  # Broad-phase and object layers for Jolt
│       ├── Log.hpp         # In-memory log buffer + optional file logging
│       ├── OpenGLDebugDrawer.hpp  # Wireframe drawing of Jolt bodies
│       ├── PoseController.hpp  # MotionMode, ControllerState, apply_pose_control()
│       ├── ShapeWireframeSupport.hpp  # Shared shape traversal for wireframe draw/count
│       ├── Simulator.hpp   # run_demo(SimulatorConfig) – headless
│       ├── SimulatorScene.hpp  # SimulatorScene, create/destroy scene
│       └── Visualizer.hpp  # run_demo_visual(SimulatorConfig) – window + OpenGL
├── src/
│   ├── main.cpp            # Entry point: --headless, --http-port, --walk
│   ├── Log.cpp             # Log buffer and file output
│   ├── HumanRagdoll.cpp    # Human.tof loading, procedural human rig
│   ├── Simulator.cpp       # Headless step loop
│   ├── SimulatorScene.cpp  # Shared world/ground/ragdoll setup and teardown
│   ├── PoseController.cpp  # Standing, walking, ragdoll, jump pose control
│   ├── Visualizer.cpp      # GLFW window, ImGui panel, orbit camera, step-and-draw loop
│   ├── OpenGLDebugDrawer.cpp  # OpenGL wireframe rendering
│   ├── HttpControl.cpp     # Background-thread HTTP server
│   └── DrawableBodyCount.cpp  # Test-only: count bodies with drawable shapes
├── tests/
│   ├── CMakeLists.txt      # Built when BUILD_TESTS=ON (default ON)
│   ├── smoke_test.cpp      # Minimal Jolt physics step
│   └── e2e_verification.cpp  # Standing/walking stability, drawable body count
├── assets/                 # Human.tof rig and walk/neutral animations
├── docs/
│   └── PROJECT_STRUCTURE.md
└── README.md
```

- **include/biomechanics/** – Public headers; use `#include "biomechanics/..."` with `include/` on the include path.
- **src/** – Application implementation linked into `biomechanics_simulator`; `DrawableBodyCount.cpp` is compiled only for E2E tests.
- **tests/** – Enabled by default (`BUILD_TESTS=ON`); run with `ctest` or `.\run-e2e.ps1`.
