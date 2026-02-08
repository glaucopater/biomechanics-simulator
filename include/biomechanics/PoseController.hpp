#pragma once

#include "biomechanics/Config.hpp"
#include "biomechanics/SimulatorScene.hpp"
#include <Jolt.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
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

/** Per-frame state for the pose controller (mode, phase, jump trigger). */
struct ControllerState {
  MotionMode mode = MotionMode::Standing;
  float walk_phase = 0.f;   // used for procedural walk
  float walk_time = 0.f;   // used for animation-driven walk (seconds)
  bool jump_triggered = false;
  /** Frames to skip after jump before switching to Ragdoll (0 = inactive). */
  int jump_frames_hold = 0;
};

/**
 * Apply pose control using Jolt Ragdoll: Standing = DriveToPoseUsingMotors(neutral or animation),
 * Walking = DriveToPoseUsingMotors(walk pose from phase or animation). Call before each PhysicsSystem::Update.
 * Uses scene.standing_anim / scene.walking_anim when set; otherwise procedural poses.
 * inDeltaTime is the time step for this call (use sub-step dt when using multiple sub-steps per frame).
 */
void apply_pose_control(SimulatorScene& scene,
                        ControllerState& state,
                        const SimulatorConfig& config,
                        float inDeltaTime = -1.f);

/**
 * Clamp linear and angular velocities of all ragdoll bodies. Call after Update each frame.
 */
void clamp_ragdoll_velocities(JPH::Ragdoll* ragdoll,
                              JPH::BodyInterface& body_interface,
                              float max_linear_speed = 25.f,
                              float max_angular_speed = 15.f);

/**
 * Zero linear and angular velocities of all ragdoll bodies. Call after create_simulator_scene.
 */
void zero_ragdoll_velocities(JPH::Ragdoll* ragdoll, JPH::BodyInterface& body_interface);

/**
 * Reset ragdoll to a neutral standing pose at the given root offset and reset constraint warm start.
 * If body_interface is non-null, zeros all linear and angular velocities (stops spinning/drift).
 * Use for Reset button (no destroy).
 */
void reset_ragdoll_to_standing(JPH::Ragdoll* ragdoll,
                               const JPH::RagdollSettings* ragdoll_settings,
                               JPH::RVec3 root_offset,
                               JPH::BodyInterface* body_interface = nullptr);

}  // namespace biomechanics
