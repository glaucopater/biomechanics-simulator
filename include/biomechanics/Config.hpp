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
  float walk_speed       = 1.8f;   // gait cycle Hz
  float walk_forward_speed = 0.4f;  // m/s forward when using walk animation (0 = in place)
  float walk_hip_amplitude  = 0.5f;   // rad (hip swing)
  float walk_knee_amplitude = 0.45f;  // rad (knee bend; moderate so feet stay above ground)
  float walk_arm_amplitude  = 0.35f;  // rad
  float walk_joint_spring_stiffness = 25.f;  // soften limb springs so PD can drive walk
  float walk_max_torque  = 140.f;  // max PD torque for limbs during walk (higher than standing)
  float jump_impulse_y   = 22.f;   // upward impulse on pelvis when jumping
  int   default_motion_mode = 0;   // 0=Standing, 1=Walking, 2=Ragdoll (visualizer start; headless uses this)
};

}  // namespace biomechanics
