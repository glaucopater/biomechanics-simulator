#pragma once

#include "biomechanics/Config.hpp"
#include "biomechanics/Ragdoll.hpp"
#include <Jolt.h>
#include <Math/Quat.h>
#include <Math/Vec3.h>
#include <Math/Real.h>
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
  std::vector<JPH::Quat> rest_orientations;
  /** World-space rest position of pelvis (for position hold in Standing/Walking). */
  JPH::RVec3 rest_pelvis_position = JPH::RVec3::sZero();
  bool rest_captured = false;
};

class JPH::PhysicsSystem;
class JPH::BodyInterface;

/**
 * Apply pose control: standing (PD toward rest pose), walking (cyclic leg/arm torques),
 * and one-shot jump impulse. Call once per frame before PhysicsSystem::Update.
 */
void apply_pose_control(JPH::PhysicsSystem* physics,
                        const RagdollHandles& ragdoll,
                        ControllerState& state,
                        const SimulatorConfig& config);

/**
 * Clamp linear and angular velocities of all ragdoll bodies. Call after Update each frame.
 */
void clamp_ragdoll_velocities(RagdollHandles& ragdoll,
                              JPH::BodyInterface& body_interface,
                              float max_linear_speed = 25.f,
                              float max_angular_speed = 15.f);

/**
 * Zero linear and angular velocities of all ragdoll bodies. Call after create_simulator_scene.
 */
void zero_ragdoll_velocities(RagdollHandles& ragdoll, JPH::BodyInterface& body_interface);

/**
 * Capture current body orientations as the rest (standing) pose. Call after create_simulator_scene.
 */
void capture_rest_pose(const RagdollHandles& ragdoll, JPH::BodyInterface& body_interface, ControllerState& state);

}  // namespace biomechanics
