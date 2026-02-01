# Project structure

```
biomechanics-simulator/
├── CMakeLists.txt          # Root build: Bullet + GLFW (vcpkg/FetchContent), main app, optional tests
├── vcpkg.json              # vcpkg manifest (bullet3)
├── include/
│   └── biomechanics/       # Public API
│       ├── Config.hpp      # SimulatorConfig (gravity, time step, ragdoll, window size, etc.)
│       ├── Ragdoll.hpp     # BodyPart enum, RagdollHandles, setup_ragdoll(), destroy_ragdoll()
│       ├── Simulator.hpp   # run_demo(SimulatorConfig) – headless
│       ├── SimulatorScene.hpp  # SimulatorScene, create_simulator_scene(), destroy_simulator_scene()
│       ├── Visualizer.hpp  # run_demo_visual(SimulatorConfig) – window + OpenGL
│       ├── OpenGLDebugDrawer.hpp  # btIDebugDraw implementation for wireframe
│       └── PoseController.hpp  # MotionMode, ControllerState, apply_pose_control() – standing/walking/jump
├── src/
│   ├── main.cpp            # Entry point: --headless → run_demo(), else run_demo_visual()
│   ├── Ragdoll.cpp         # Ragdoll creation (capsules + 6-DOF constraints)
│   ├── Simulator.cpp       # Headless: create_simulator_scene, pose control, step loop, destroy_simulator_scene
│   ├── SimulatorScene.cpp  # Shared world/ground/ragdoll setup and teardown
│   ├── PoseController.cpp  # Standing (PD to rest pose), walking (cyclic gait), jump impulse
│   ├── Visualizer.cpp      # GLFW window, OpenGL context, debug drawer, key bindings (1/2/3/Space), step-and-draw loop
│   └── OpenGLDebugDrawer.cpp  # drawLine, setDebugMode, etc. (OpenGL wireframe)
├── tests/
│   ├── CMakeLists.txt      # Built when -DBUILD_TESTS=ON
│   └── smoke_test.cpp     # Minimal Bullet world step
├── docs/
│   └── PROJECT_STRUCTURE.md
└── README.md
```

- **include/biomechanics/** – Public headers; use `#include "biomechanics/..."` with `include/` on the include path.
- **src/** – Application and library implementation; no headers required outside `include/`.
- **tests/** – Optional; enable with `cmake -DBUILD_TESTS=ON` and run `ctest` or the `smoke_test` executable.
