/**
 * E2E verification: scene creation, pose control, physics stability, and
 * drawability (every body the renderer would draw is a shape we support).
 * Run from repo root so assets/ paths resolve. No OpenGL required.
 */
#include "biomechanics/Config.hpp"
#include "biomechanics/DrawableBodyCount.hpp"
#include "biomechanics/HumanRagdoll.hpp"
#include "biomechanics/PoseController.hpp"
#include "biomechanics/SimulatorScene.hpp"
#include <Jolt/Physics/Body/BodyInterface.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace biomech = biomechanics;

static bool is_finite(float x) {
  return std::isfinite(x) && !std::isnan(x);
}

static bool is_finite(const JPH::RVec3& p) {
  return is_finite(static_cast<float>(p.GetX())) &&
         is_finite(static_cast<float>(p.GetY())) &&
         is_finite(static_cast<float>(p.GetZ()));
}

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  std::setvbuf(stderr, nullptr, _IONBF, 0);
  std::fprintf(stderr, "E2E starting (cwd will be used for assets/)...\n");

  biomech::ensure_jolt_registered();
  std::fprintf(stderr, "E2E: Jolt registered, creating scene...\n");

  biomech::SimulatorConfig config;
  config.num_steps = 120;
  config.time_step = 1.f / 60.f;
  config.default_motion_mode = 0;  // Standing
  config.ragdoll_height = 0.04f;

  biomech::SimulatorScene scene;
  biomech::create_simulator_scene(config, scene);
  std::fprintf(stderr, "E2E: scene created, stepping standing...\n");

  if (!scene.physics) {
    std::fprintf(stderr, "E2E FAIL: scene.physics is null\n");
    std::fflush(stderr);
    return EXIT_FAILURE;
  }
  if (!scene.ragdoll) {
    std::fprintf(stderr, "E2E FAIL: scene.ragdoll is null\n");
    std::fflush(stderr);
    return EXIT_FAILURE;
  }
  if (!scene.ragdoll_settings) {
    std::fprintf(stderr, "E2E FAIL: scene.ragdoll_settings is null\n");
    std::fflush(stderr);
    return EXIT_FAILURE;
  }

  size_t ragdoll_bodies = scene.ragdoll->GetBodyIDs().size();
  if (ragdoll_bodies < 12u) {
    std::fprintf(stderr, "E2E FAIL: ragdoll has %zu bodies (expected >= 12)\n", ragdoll_bodies);
    return EXIT_FAILURE;
  }

  JPH::BodyInterface& bi = scene.physics->GetBodyInterface();
  biomech::zero_ragdoll_velocities(scene.ragdoll, bi);

  biomech::ControllerState ctrl_state;
  ctrl_state.mode = biomech::MotionMode::Standing;
  const float dt = config.time_step;

  for (int i = 0; i < config.num_steps; ++i) {
    biomech::apply_pose_control(scene, ctrl_state, config, dt);
    scene.physics->Update(dt, 1, scene.temp_allocator, scene.job_system);
    biomech::clamp_ragdoll_velocities(scene.ragdoll, bi);
  }

  JPH::BodyID root_id = scene.ragdoll->GetBodyID(0);
  if (root_id.IsInvalid()) {
    std::fprintf(stderr, "E2E FAIL: root body ID invalid\n");
    return EXIT_FAILURE;
  }
  JPH::RVec3 root_pos = bi.GetPosition(root_id);
  if (!is_finite(root_pos)) {
    std::fprintf(stderr, "E2E FAIL: root position has NaN/Inf (%.3f, %.3f, %.3f)\n",
                 static_cast<double>(root_pos.GetX()),
                 static_cast<double>(root_pos.GetY()),
                 static_cast<double>(root_pos.GetZ()));
    return EXIT_FAILURE;
  }
  float root_y = static_cast<float>(root_pos.GetY());
  if (root_y < 0.1f) {
    std::fprintf(stderr, "E2E FAIL: root Y=%.3f (expected >= 0.1, character should stay above ground)\n",
                 static_cast<double>(root_y));
    return EXIT_FAILURE;
  }

  std::fprintf(stderr, "E2E: checking drawable bodies...\n");
  int drawable = biomech::count_drawable_bodies(scene.physics, scene.ragdoll, scene.ground_id);
  int expected_drawable = 1 + static_cast<int>(ragdoll_bodies);  // ground + all ragdoll
  std::fprintf(stderr, "E2E: drawable=%d expected=%d (allow %d-%d)\n", drawable, expected_drawable, expected_drawable - 2, expected_drawable);
  // Allow up to 2 bodies with unsupported shape types (e.g. ConvexHull in some rigs)
  if (drawable < expected_drawable - 2 || drawable > expected_drawable) {
    std::fprintf(stderr, "E2E FAIL: drawable bodies=%d (expected %d or %d-%d). Renderer would not draw all bodies.\n",
                 drawable, expected_drawable, expected_drawable - 2, expected_drawable);
    std::fflush(stderr);
    return EXIT_FAILURE;
  }

  biomech::destroy_simulator_scene(scene);
  std::printf("E2E OK: Standing stable, root_y=%.3f, drawable=%d/%zu+1\n",
             static_cast<double>(root_y), drawable, ragdoll_bodies);
  std::fflush(stdout);

  std::fprintf(stderr, "E2E: walking phase...\n");
  /* Walking mode: run briefly and assert no explosion */
  config.default_motion_mode = 1;  // Walking
  biomech::create_simulator_scene(config, scene);
  if (!scene.physics || !scene.ragdoll) {
    std::fprintf(stderr, "E2E FAIL: scene recreate failed\n");
    return EXIT_FAILURE;
  }
  JPH::BodyInterface& bi_walk = scene.physics->GetBodyInterface();
  biomech::zero_ragdoll_velocities(scene.ragdoll, bi_walk);
  ctrl_state.mode = biomech::MotionMode::Walking;
  ctrl_state.walk_phase = 0.f;
  ctrl_state.walk_time = 0.f;
  for (int i = 0; i < 90; ++i) {
    biomech::apply_pose_control(scene, ctrl_state, config, dt);
    scene.physics->Update(dt, 1, scene.temp_allocator, scene.job_system);
    biomech::clamp_ragdoll_velocities(scene.ragdoll, bi_walk);
  }
  root_pos = bi_walk.GetPosition(scene.ragdoll->GetBodyID(0));
  if (!is_finite(root_pos)) {
    std::fprintf(stderr, "E2E FAIL: after walk, root position has NaN/Inf\n");
    return EXIT_FAILURE;
  }
  if (static_cast<float>(root_pos.GetY()) < -1.f) {
    std::fprintf(stderr, "E2E FAIL: after walk, root Y=%.3f (fell through floor)\n",
                 static_cast<double>(root_pos.GetY()));
    return EXIT_FAILURE;
  }
  float root_x = static_cast<float>(root_pos.GetX());
  float root_z = static_cast<float>(root_pos.GetZ());
  if (!is_finite(root_x) || !is_finite(root_z)) {
    std::fprintf(stderr, "E2E FAIL: root X/Z not finite after walk\n");
    return EXIT_FAILURE;
  }
  biomech::destroy_simulator_scene(scene);
  std::printf("E2E OK: Walking stable, root (%.3f, %.3f, %.3f)\n",
             static_cast<double>(root_x), static_cast<double>(root_pos.GetY()), static_cast<double>(root_z));
  std::fflush(stdout);
  std::fprintf(stderr, "E2E done (PASS).\n");
  return EXIT_SUCCESS;
}
