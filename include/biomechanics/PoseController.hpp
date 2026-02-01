#pragma once

#include "biomechanics/Config.hpp"
#include "biomechanics/Ragdoll.hpp"
#include <btBulletDynamicsCommon.h>
#include <vector>

namespace biomechanics {

/** Motion mode for the ragdoll: stance or gait. */
enum class MotionMode : int {
  Standing,
  Walking,
  /** No pose control; pure ragdoll (e.g. after jump or for debugging). */
  Ragdoll
};

/** Per-frame state for the pose controller (mode, phase, jump trigger, cached rest pose). */
struct ControllerState {
  MotionMode mode = MotionMode::Standing;
  float walk_phase = 0.f;
  bool jump_triggered = false;
  /** Frames to skip pelvis hold after jump so character rises before going Ragdoll (0 = inactive). */
  int jump_frames_hold = 0;
  /** World-space rest orientations (standing pose), filled on capture. */
  std::vector<btQuaternion> rest_orientations;
  /** World-space rest position of pelvis (for position hold in Standing/Walking). */
  btVector3 rest_pelvis_position;
  bool rest_captured = false;
};

/**
 * Apply pose control: standing (PD toward rest pose), walking (cyclic leg/arm torques),
 * and one-shot jump impulse. Call once per frame before stepSimulation.
 */
void apply_pose_control(btDynamicsWorld* world,
                       const RagdollHandles& ragdoll,
                       ControllerState& state,
                       const SimulatorConfig& config);

/**
 * Clamp linear and angular velocities of all ragdoll bodies to prevent explosion
 * when the model falls or collides. Call after stepSimulation each frame.
 */
void clamp_ragdoll_velocities(RagdollHandles& ragdoll,
                              btScalar max_linear_speed = 25.f,
                              btScalar max_angular_speed = 15.f);

/**
 * Zero linear and angular velocities of all ragdoll bodies. Call after
 * create_simulator_scene so the model starts completely still in Standing mode.
 */
void zero_ragdoll_velocities(RagdollHandles& ragdoll);

/**
 * Capture current body orientations as the rest (standing) pose. Call after
 * create_simulator_scene so Standing holds the initial pose.
 */
void capture_rest_pose(const RagdollHandles& ragdoll, ControllerState& state);

}  // namespace biomechanics
