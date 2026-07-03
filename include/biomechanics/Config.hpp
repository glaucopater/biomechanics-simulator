#pragma once

namespace biomechanics {

/** Default simulation parameters. */
struct SimulatorConfig {
  float gravity_y       = -10.f;
  float time_step       = 1.f / 60.f;
  int   num_steps       = 300;
  float ragdoll_scale   = 1.f;
  float ragdoll_height  = 0.04f;  // so feet rest on ground (lowest body point ~ -0.035 local)
  float standing_min_height = 0.95f;  // minimum root Y when standing (stops slow sink from gravity)
  int   window_width    = 1024;
  int   window_height   = 768;

  // Pose controller (standing / walking / jump)
  float pose_stiffness   = 380.f;  // PD gain for orientation (needs to be strong enough to hold vs gravity)
  float pose_damping     = 38.f;   // PD damping
  float pose_linear_stiffness = 1200.f;  // PD gain for pelvis position hold (Standing/Walking)
  float pose_linear_damping  = 80.f;   // PD damping for linear velocity
  float standing_max_torque  = 320.f;  // max PD torque when standing (higher than walk so we don't collapse)
  float walk_speed       = 1.2f;   // gait cycle Hz (procedural walk only)
  float walk_anim_playback = 0.55f;  // animation walk time scale (1 = full speed, lower = slower)
  float walk_forward_speed = 0.f;  // extra m/s forward (0 = animation root motion only)
  float walk_hip_amplitude  = 0.55f;   // rad (hip swing, procedural)
  float walk_knee_amplitude = 0.65f;  // rad (knee bend on swing leg, procedural)
  float walk_arm_amplitude  = 0.4f;  // rad (counter-swing arms, procedural)
  float walk_joint_spring_stiffness = 25.f;  // soften limb springs so PD can drive walk
  float walk_max_torque  = 140.f;  // max PD torque for limbs during walk (higher than standing)
  float jump_velocity_y  = 3.4f;  // m/s takeoff velocity (kinematic parabola; apex = v^2/2g ~ 0.58 m)
  float jump_crouch_duration = 0.35f;  // seconds counter-movement dip (protocol: fast, ~0.25-0.35 s)
  // Squat/crouch targets. Keep the triple consistent so feet stay planted:
  // tests/squat_probe.cpp fine search gives (knee, hip, drop) rows; ankle = hip - knee.
  float jump_crouch_knee = 1.50f;     // rad knee flexion at full crouch (deep semi-squat)
  float jump_crouch_hip = 0.66f;      // rad hip flexion at full crouch (thighs incline, chest up)
  float jump_crouch_drop = 0.26f;     // m pelvis lowers during crouch (geometrically consistent with knee/hip)
  float jump_extend_duration = 0.18f;  // seconds legs extend from crouch to takeoff
  float jump_land_duration = 0.35f;    // seconds landing absorb (bend then straighten)
  float jump_land_knee = 0.90f;        // rad knee flexion at deepest landing absorb
  float action_pose_duration = 0.40f;  // seconds to ease into raise-leg / punch / kick
  float raise_leg_hip = 0.85f;        // rad hip flexion (raised leg)
  float raise_leg_knee = 0.90f;       // rad knee flexion (raised leg)
  float punch_arm = 1.35f;            // rad punching upper arm forward
  float punch_elbow = 0.55f;          // rad punching elbow bend at full extension
  float punch_guard_arm = 0.75f;      // rad guard upper arm
  float kick_hip = 1.05f;             // rad kicking hip flexion
  float kick_knee = 0.08f;            // rad kicking knee (near straight)
  float kick_plant_knee = 0.15f;      // rad support knee bend
  int   default_motion_mode = 0;   // 0=Standing, 1=Walking, 2=Ragdoll (visualizer start; headless uses this)
};

}  // namespace biomechanics
