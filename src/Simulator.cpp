#include "biomechanics/Config.hpp"
#include "biomechanics/Log.hpp"
#include "biomechanics/PoseController.hpp"
#include "biomechanics/Simulator.hpp"
#include "biomechanics/SimulatorScene.hpp"
#include <cstdio>

namespace biomechanics {

void run_demo(const SimulatorConfig& config) {
  clear_log(true);
  log("Headless run started (log file: %s)", get_log_path());

  SimulatorScene scene;
  create_simulator_scene(config, scene);
  if (!scene.physics) {
    std::fprintf(stderr, "create_simulator_scene failed (physics is null)\n");
    return;
  }
  JPH::BodyInterface& bi = scene.physics->GetBodyInterface();
  zero_ragdoll_velocities(scene.ragdoll, bi);

  ControllerState ctrl_state;
  int dm = config.default_motion_mode;
  ctrl_state.mode = (dm >= 0 && dm <= 2) ? static_cast<MotionMode>(dm) : MotionMode::Standing;

  std::printf("Biomechanics Simulator: stepping %d times (dt=%.4f s), mode=%s\n",
              config.num_steps, config.time_step,
              ctrl_state.mode == MotionMode::Standing ? "standing" :
              ctrl_state.mode == MotionMode::Walking ? "walking" : "ragdoll");

  const float dt = static_cast<float>(config.time_step);
  for (int i = 0; i < config.num_steps; ++i) {
    apply_pose_control(scene, ctrl_state, config, dt);
    scene.physics->Update(dt, 1, scene.temp_allocator, scene.job_system);
    clamp_ragdoll_velocities(scene.ragdoll, bi);
  }

  if (scene.ragdoll) {
    JPH::BodyID root_id = scene.ragdoll->GetBodyID(0);
    if (!root_id.IsInvalid()) {
      JPH::RVec3 pos = bi.GetPosition(root_id);
      std::printf("Root (LowerBody) position after simulation: (%.3f, %.3f, %.3f)\n",
                  static_cast<double>(pos.GetX()), static_cast<double>(pos.GetY()), static_cast<double>(pos.GetZ()));
    }
  }
  std::printf("Physics system has %u bodies.\n", scene.physics->GetNumBodies());

  destroy_simulator_scene(scene);
  std::printf("Done.\n");
}

}  // namespace biomechanics
