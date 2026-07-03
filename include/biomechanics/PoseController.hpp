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

/** Motion mode for the ragdoll: stance or gait. Values used by HTTP/config. */
enum class MotionMode : int {
  Standing = 0,
  Walking = 1,
  Ragdoll = 2,
  /** Static stance: stand still with one leg raised (same root pinning as Standing). */
  StandingRaiseLeg = 3,
  /** Forward punch with the right arm. */
  PunchRight = 4,
  /** Forward punch with the left arm. */
  PunchLeft = 5,
  /** Front kick with the right leg (foot extends forward). */
  FrontKick = 6,
  /** Static semi-squat: knees/hips bent, pelvis lowered, arms slightly back. */
  Squat = 7
};

/** Modes that pin root XZ like Standing (kinematic root anchor in Visualizer). */
inline bool is_pinned_stance_mode(MotionMode mode) {
  return mode == MotionMode::Standing || mode == MotionMode::StandingRaiseLeg
      || mode == MotionMode::PunchRight || mode == MotionMode::PunchLeft
      || mode == MotionMode::FrontKick || mode == MotionMode::Squat;
}

/** Kinematic jump sequence phase (all phases driven with DriveToPoseUsingKinematics). */
enum class JumpPhase : int {
  None = 0,
  /** Knees/hips bend, pelvis drops, arms swing back. */
  Crouch,
  /** Legs extend, root rises back to standing height, arms swing up. */
  Launch,
  /** Ballistic root Y parabola; legs near-straight, arms up. */
  Flight,
  /** Landing absorb: brief knee bend, arms come down, then Standing. */
  Land
};

/** Per-frame state for the pose controller (mode, phase, jump trigger). */
struct ControllerState {
  MotionMode mode = MotionMode::Standing;
  float walk_phase = 0.f;   // used for procedural walk
  float walk_time = 0.f;   // used for animation-driven walk (seconds)
  /** World root at walk start; animation root delta is added for locomotion. */
  JPH::RVec3 walk_root_origin{0, 0, 0};
  JPH::RVec3 walk_anim_root_at_start{0, 0, 0};
  bool walk_root_origin_valid = false;
  bool jump_triggered = false;
  /** Current phase of the kinematic jump sequence (None = not jumping). */
  JumpPhase jump_phase = JumpPhase::None;
  /** Elapsed time in the current jump phase (seconds). */
  float jump_phase_time = 0.f;
  /** Elapsed time in the current pinned action pose (raise leg / punch / kick). */
  float action_time = 0.f;
};

/**
 * Apply pose control using Jolt Ragdoll: Standing = kinematics/motors to hold upright,
 * Walking = DriveToPoseUsingKinematics(walk animation or procedural gait). Call before each PhysicsSystem::Update.
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

/**
 * Reset ragdoll to the initial standing pose (same limb positions in space as at start).
 * Uses pose stored in scene (captured after settling or at creation). If no stored pose, falls back to reset_ragdoll_to_standing.
 */
void reset_ragdoll_to_initial_standing(SimulatorScene& scene,
                                     JPH::BodyInterface* body_interface = nullptr);

/**
 * Store the ragdoll's current pose as the initial standing pose (for reset). Call after the character has settled in Standing.
 */
void capture_standing_pose_as_initial(SimulatorScene& scene);

}  // namespace biomechanics
