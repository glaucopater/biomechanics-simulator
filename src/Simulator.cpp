#include "biomechanics/Config.hpp"
#include "biomechanics/Log.hpp"
#include "biomechanics/PoseController.hpp"
#include "biomechanics/Ragdoll.hpp"
#include "biomechanics/Simulator.hpp"
#include "biomechanics/SimulatorScene.hpp"
#include <cstdio>

namespace biomechanics {

void run_demo(const SimulatorConfig& config) {
  clear_log(true);
  log("Headless run started (log file: %s)", get_log_path());

  SimulatorScene scene;
  create_simulator_scene(config, scene);
  zero_ragdoll_velocities(scene.ragdoll);

  ControllerState ctrl_state;
  int dm = config.default_motion_mode;
  ctrl_state.mode = (dm >= 0 && dm <= 2) ? static_cast<MotionMode>(dm) : MotionMode::Standing;
  capture_rest_pose(scene.ragdoll, ctrl_state);

  std::printf("Biomechanics Simulator: stepping %d times (dt=%.4f s), mode=%s\n",
              config.num_steps, config.time_step,
              ctrl_state.mode == MotionMode::Standing ? "standing" :
              ctrl_state.mode == MotionMode::Walking ? "walking" : "ragdoll");

  for (int i = 0; i < config.num_steps; ++i) {
    apply_pose_control(scene.world, scene.ragdoll, ctrl_state, config);
    scene.world->stepSimulation(static_cast<btScalar>(config.time_step), 1);
    clamp_ragdoll_velocities(scene.ragdoll);
  }

  btTransform trans;
  scene.ragdoll.bodies[static_cast<int>(BodyPart::Pelvis)]->getMotionState()->getWorldTransform(trans);
  std::printf("Pelvis position after simulation: (%.3f, %.3f, %.3f)\n",
              trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z());
  std::printf("World has %d collision objects.\n", scene.world->getNumCollisionObjects());

  destroy_simulator_scene(scene);
  std::printf("Done.\n");
}

}  // namespace biomechanics
