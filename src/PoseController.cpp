#include "biomechanics/Config.hpp"
#include "biomechanics/Log.hpp"
#include "biomechanics/PoseController.hpp"
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Skeleton/SkeletonPose.h>
#include <Jolt/Skeleton/SkeletalAnimation.h>
#include <cmath>
#include <algorithm>
#include <cstdio>

namespace biomechanics {

namespace {

using namespace JPH;

constexpr float TWO_PI = 6.28318530718f;

// Procedural 12-bone rig indices (create_human_ragdoll_settings).
enum ProceduralJoint : int {
  LowerBody = 0,
  MidBody = 1,
  UpperBody = 2,
  Head = 3,
  UpperArmL = 4,
  UpperArmR = 5,
  LowerArmL = 6,
  LowerArmR = 7,
  UpperLegL = 8,
  UpperLegR = 9,
  LowerLegL = 10,
  LowerLegR = 11,
};

/** Resolved skeleton joint indices for pose authoring (Human.tof or procedural). */
struct RigJoints {
  int r_thigh = -1;
  int r_shin = -1;
  int l_thigh = -1;
  int l_shin = -1;
  int r_upper_arm = -1;
  int r_forearm = -1;
  int l_upper_arm = -1;
  int l_forearm = -1;
  int torso = -1;
  int torso_upper = -1;
  int r_foot = -1;
  int l_foot = -1;
  /** When true, rotations are composed on top of the sampled neutral pose. */
  bool compose_on_base = false;
};

Quat rotation_axis(Vec3Arg axis, float angle_rad) {
  return Quat::sRotation(axis, angle_rad);
}

RigJoints resolve_rig_joints(const Skeleton* skel) {
  RigJoints j;
  if (!skel)
    return j;
  if (skel->GetJointIndex("R_Arm_sjnt_0") >= 0) {
    j.compose_on_base = true;
    j.r_thigh = skel->GetJointIndex("R_Leg_sjnt_0");
    j.r_shin = skel->GetJointIndex("R_Leg_sjnt_1");
    j.l_thigh = skel->GetJointIndex("L_Leg_sjnt_0");
    j.l_shin = skel->GetJointIndex("L_Leg_sjnt_1");
    j.r_upper_arm = skel->GetJointIndex("R_Arm_sjnt_0");
    j.r_forearm = skel->GetJointIndex("R_Arm_sjnt_1");
    j.l_upper_arm = skel->GetJointIndex("L_Arm_sjnt_0");
    j.l_forearm = skel->GetJointIndex("L_Arm_sjnt_1");
    j.torso = skel->GetJointIndex("C_Spine_sjnt_0");
    j.torso_upper = skel->GetJointIndex("C_Spine_sjnt_4");
    j.r_foot = skel->GetJointIndex("R_Foot_sjnt_0");
    j.l_foot = skel->GetJointIndex("L_Foot_sjnt_0");
    return j;
  }
  if (skel->GetJointCount() >= 12) {
    j.r_thigh = UpperLegR;
    j.r_shin = LowerLegR;
    j.l_thigh = UpperLegL;
    j.l_shin = LowerLegL;
    j.r_upper_arm = UpperArmR;
    j.r_forearm = LowerArmR;
    j.l_upper_arm = UpperArmL;
    j.l_forearm = LowerArmL;
    j.torso = MidBody;
    j.torso_upper = UpperBody;
  }
  return j;
}

bool rig_has_limbs(const RigJoints& rig, bool arms, bool legs) {
  if (arms && (rig.r_upper_arm < 0 || rig.l_upper_arm < 0 || rig.r_forearm < 0))
    return false;
  if (legs && (rig.r_thigh < 0 || rig.r_shin < 0 || rig.l_thigh < 0))
    return false;
  return true;
}

void apply_joint_rotation(SkeletonPose& pose, const RigJoints& rig, int joint_idx,
                          Vec3Arg axis, float angle_rad, const Quat& absolute_rot) {
  if (joint_idx < 0)
    return;
  auto& joint = pose.GetJoint(static_cast<uint>(joint_idx));
  if (rig.compose_on_base)
    joint.mRotation = rotation_axis(axis, angle_rad) * joint.mRotation;
  else
    joint.mRotation = absolute_rot;
}

float smoothstep01(float t) {
  t = std::clamp(t, 0.f, 1.f);
  return t * t * (3.f - 2.f * t);
}

void build_neutral_pose(const Ragdoll* ragdoll, const RagdollSettings* settings, SkeletonPose& pose) {
  const Skeleton* skel = settings->GetSkeleton();
  if (!skel || skel->GetJointCount() == 0)
    return;
  pose.SetSkeleton(skel);
  RVec3 root_pos;
  Quat root_rot;
  ragdoll->GetRootTransform(root_pos, root_rot, true);
  pose.SetRootOffset(root_pos);
  for (uint i = 0; i < skel->GetJointCount(); ++i) {
    pose.GetJoint(i).mRotation = root_rot;  // only joint 0 is root; others we overwrite below
    pose.GetJoint(i).mTranslation = Vec3::sZero();
  }
  pose.GetJoint(0).mRotation = root_rot;
  for (uint i = 1; i < skel->GetJointCount(); ++i) {
    pose.GetJoint(i).mRotation = Quat::sIdentity();
    pose.GetJoint(i).mTranslation = Vec3::sZero();
  }
  pose.CalculateJointMatrices();
}

void sample_standing_base_pose(const Ragdoll* ragdoll, const RagdollSettings* settings,
                               const SkeletalAnimation* standing_anim,
                               const SimulatorConfig& config, SkeletonPose& pose) {
  const Skeleton* skel = settings->GetSkeleton();
  if (!skel || skel->GetJointCount() == 0)
    return;
  pose.SetSkeleton(skel);
  if (standing_anim) {
    standing_anim->Sample(0.0f, pose);
    SkeletonPose::JointState& joint0 = pose.GetJoint(0);
    joint0.mTranslation = Vec3::sZero();
    RVec3 root_offset;
    ragdoll->GetRootTransform(root_offset, joint0.mRotation, true);
    root_offset = RVec3(root_offset.GetX(), (JPH::Real)config.standing_min_height, root_offset.GetZ());
    pose.SetRootOffset(root_offset);
  } else {
    build_neutral_pose(ragdoll, settings, pose);
    RVec3 root_off = pose.GetRootOffset();
    root_off = RVec3(root_off.GetX(), (JPH::Real)config.standing_min_height, root_off.GetZ());
    pose.SetRootOffset(root_off);
  }
  pose.CalculateJointMatrices();
}

void apply_kinematic_pose(Ragdoll* ragdoll, const SkeletonPose& pose, float dt) {
  ragdoll->DriveToPoseUsingKinematics(pose, dt);
}

void build_walk_pose(const Ragdoll* ragdoll, const RagdollSettings* settings,
                    float phase, const SimulatorConfig& config, SkeletonPose& pose) {
  const Skeleton* skel = settings->GetSkeleton();
  if (!skel || skel->GetJointCount() < 12)
    return;
  pose.SetSkeleton(skel);
  RVec3 root_pos;
  Quat root_rot;
  ragdoll->GetRootTransform(root_pos, root_rot, true);
  pose.SetRootOffset(root_pos);
  for (uint i = 0; i < skel->GetJointCount(); ++i) {
    pose.GetJoint(i).mRotation = (i == 0) ? root_rot : Quat::sIdentity();
    pose.GetJoint(i).mTranslation = Vec3::sZero();
  }
  const float sin_l = std::sin(phase);
  const float sin_r = std::sin(phase + JPH_PI);
  const float hip_l = config.walk_hip_amplitude * sin_l;
  const float hip_r = config.walk_hip_amplitude * sin_r;
  const float knee_l = config.walk_knee_amplitude * std::max(0.f, sin_l);
  const float knee_r = config.walk_knee_amplitude * std::max(0.f, sin_r);
  const float arm_l = config.walk_arm_amplitude * sin_r;
  const float arm_r = config.walk_arm_amplitude * sin_l;
  const float torso_sway = 0.06f * sin_l;
  const RigJoints rig = resolve_rig_joints(skel);
  if (!rig_has_limbs(rig, true, true))
    return;
  apply_joint_rotation(pose, rig, rig.l_thigh, -Vec3::sAxisY(), hip_l,
                       rotation_axis(-Vec3::sAxisY(), hip_l));
  apply_joint_rotation(pose, rig, rig.r_thigh, -Vec3::sAxisY(), hip_r,
                       rotation_axis(-Vec3::sAxisY(), hip_r));
  apply_joint_rotation(pose, rig, rig.l_shin, Vec3::sAxisX(), knee_l,
                       rotation_axis(Vec3::sAxisX(), knee_l));
  apply_joint_rotation(pose, rig, rig.r_shin, Vec3::sAxisX(), knee_r,
                       rotation_axis(Vec3::sAxisX(), knee_r));
  apply_joint_rotation(pose, rig, rig.l_upper_arm, -Vec3::sAxisX(), arm_l,
                       rotation_axis(-Vec3::sAxisX(), arm_l));
  apply_joint_rotation(pose, rig, rig.r_upper_arm, Vec3::sAxisX(), arm_r,
                       rotation_axis(Vec3::sAxisX(), arm_r));
  apply_joint_rotation(pose, rig, rig.torso, Vec3::sAxisY(), torso_sway,
                       rotation_axis(Vec3::sAxisY(), torso_sway));
  pose.CalculateJointMatrices();
}

void build_standing_raise_leg_pose(const Ragdoll* ragdoll, const RagdollSettings* settings,
                                   const SkeletalAnimation* standing_anim,
                                   float ease, const SimulatorConfig& config, SkeletonPose& pose) {
  const Skeleton* skel = settings->GetSkeleton();
  const RigJoints rig = resolve_rig_joints(skel);
  if (!skel || !rig_has_limbs(rig, false, true))
    return;
  sample_standing_base_pose(ragdoll, settings, standing_anim, config, pose);
  const float hip_raise = config.raise_leg_hip * ease;
  const float knee_raise = config.raise_leg_knee * ease;
  const float lean_support_rad = -0.12f * ease;
  if (rig.compose_on_base) {
    apply_joint_rotation(pose, rig, rig.r_thigh, Vec3::sAxisX(), hip_raise, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.r_shin, Vec3::sAxisX(), knee_raise, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.torso_upper, Vec3::sAxisZ(), lean_support_rad, Quat::sIdentity());
  } else {
    apply_joint_rotation(pose, rig, rig.l_thigh, Vec3::sAxisY(), 0.f, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.l_shin, Vec3::sAxisY(), 0.f, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.r_thigh, -Vec3::sAxisY(), hip_raise,
                         rotation_axis(-Vec3::sAxisY(), hip_raise));
    apply_joint_rotation(pose, rig, rig.r_shin, Vec3::sAxisX(), knee_raise,
                         rotation_axis(Vec3::sAxisX(), knee_raise));
    apply_joint_rotation(pose, rig, rig.torso, Vec3::sAxisZ(), lean_support_rad,
                         rotation_axis(Vec3::sAxisZ(), lean_support_rad));
    apply_joint_rotation(pose, rig, rig.torso_upper, Vec3::sAxisZ(), lean_support_rad,
                         rotation_axis(Vec3::sAxisZ(), lean_support_rad));
  }
  pose.CalculateJointMatrices();
}

void build_punch_pose(const Ragdoll* ragdoll, const RagdollSettings* settings,
                      const SkeletalAnimation* standing_anim, float ease,
                      const SimulatorConfig& config, bool left_punch, SkeletonPose& pose) {
  const Skeleton* skel = settings->GetSkeleton();
  const RigJoints rig = resolve_rig_joints(skel);
  if (!skel || !rig_has_limbs(rig, true, false))
    return;
  sample_standing_base_pose(ragdoll, settings, standing_anim, config, pose);
  const float punch = config.punch_arm * ease;
  const float elbow = config.punch_elbow * ease;
  const float guard = config.punch_guard_arm * ease;
  const float torso_twist = 0.18f * ease;
  const int punch_upper = left_punch ? rig.l_upper_arm : rig.r_upper_arm;
  const int punch_forearm = left_punch ? rig.l_forearm : rig.r_forearm;
  const int guard_upper = left_punch ? rig.r_upper_arm : rig.l_upper_arm;
  const int guard_forearm = left_punch ? rig.r_forearm : rig.l_forearm;
  // Limbs are already mirrored in the rig; swap sides but keep the same joint deltas as Punch R.
  const float torso_twist_signed = left_punch ? -torso_twist : torso_twist;
  if (rig.compose_on_base) {
    apply_joint_rotation(pose, rig, punch_upper, Vec3::sAxisY(), punch, Quat::sIdentity());
    apply_joint_rotation(pose, rig, punch_forearm, Vec3::sAxisX(), -elbow, Quat::sIdentity());
    apply_joint_rotation(pose, rig, guard_upper, Vec3::sAxisY(), -guard, Quat::sIdentity());
    apply_joint_rotation(pose, rig, guard_forearm, Vec3::sAxisX(), 0.55f * ease, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.torso_upper, Vec3::sAxisY(), torso_twist_signed, Quat::sIdentity());
  } else {
    apply_joint_rotation(pose, rig, punch_upper, Vec3::sAxisX(), punch,
                         rotation_axis(Vec3::sAxisX(), punch));
    apply_joint_rotation(pose, rig, punch_forearm, Vec3::sAxisX(), -elbow,
                         rotation_axis(Vec3::sAxisX(), -elbow));
    apply_joint_rotation(pose, rig, guard_upper, Vec3::sAxisX(), -guard,
                         rotation_axis(Vec3::sAxisX(), -guard));
    apply_joint_rotation(pose, rig, guard_forearm, Vec3::sAxisX(), 0.55f * ease,
                         rotation_axis(Vec3::sAxisX(), 0.55f * ease));
    apply_joint_rotation(pose, rig, rig.torso_upper, Vec3::sAxisY(), torso_twist_signed,
                         rotation_axis(Vec3::sAxisY(), torso_twist_signed));
    apply_joint_rotation(pose, rig, rig.torso, Vec3::sAxisY(), 0.5f * torso_twist_signed,
                         rotation_axis(Vec3::sAxisY(), 0.5f * torso_twist_signed));
  }
  pose.CalculateJointMatrices();
}

void build_front_kick_pose(const Ragdoll* ragdoll, const RagdollSettings* settings,
                           const SkeletalAnimation* standing_anim, float ease,
                           const SimulatorConfig& config, SkeletonPose& pose) {
  const Skeleton* skel = settings->GetSkeleton();
  const RigJoints rig = resolve_rig_joints(skel);
  if (!skel || !rig_has_limbs(rig, true, true))
    return;
  sample_standing_base_pose(ragdoll, settings, standing_anim, config, pose);
  const float hip = config.kick_hip * ease;
  const float knee = config.kick_knee * ease;
  const float plant_knee = config.kick_plant_knee * ease;
  const float lean_back = 0.10f * ease;
  const float guard = 0.45f * ease;
  if (rig.compose_on_base) {
    // Hip flex forward (-Z); +X was driving the foot behind the body.
    apply_joint_rotation(pose, rig, rig.r_thigh, -Vec3::sAxisY(), hip, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.r_shin, Vec3::sAxisX(), knee, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.r_foot, Vec3::sAxisX(), 0.25f * ease, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.l_shin, Vec3::sAxisX(), plant_knee, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.l_upper_arm, Vec3::sAxisY(), -guard, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.r_upper_arm, Vec3::sAxisY(), guard, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.l_forearm, Vec3::sAxisX(), 0.35f * ease, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.r_forearm, Vec3::sAxisX(), -0.35f * ease, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.torso_upper, Vec3::sAxisX(), -lean_back, Quat::sIdentity());
  } else {
    apply_joint_rotation(pose, rig, rig.r_thigh, -Vec3::sAxisY(), hip,
                         rotation_axis(-Vec3::sAxisY(), hip));
    apply_joint_rotation(pose, rig, rig.r_shin, Vec3::sAxisX(), knee,
                         rotation_axis(Vec3::sAxisX(), knee));
    apply_joint_rotation(pose, rig, rig.l_thigh, Vec3::sAxisY(), 0.f, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.l_shin, Vec3::sAxisX(), plant_knee,
                         rotation_axis(Vec3::sAxisX(), plant_knee));
    apply_joint_rotation(pose, rig, rig.l_upper_arm, -Vec3::sAxisX(), guard,
                         rotation_axis(-Vec3::sAxisX(), guard));
    apply_joint_rotation(pose, rig, rig.r_upper_arm, Vec3::sAxisX(), guard,
                         rotation_axis(Vec3::sAxisX(), guard));
    apply_joint_rotation(pose, rig, rig.l_forearm, -Vec3::sAxisX(), 0.35f * ease,
                         rotation_axis(-Vec3::sAxisX(), 0.35f * ease));
    apply_joint_rotation(pose, rig, rig.r_forearm, Vec3::sAxisX(), 0.35f * ease,
                         rotation_axis(Vec3::sAxisX(), 0.35f * ease));
    apply_joint_rotation(pose, rig, rig.torso, Vec3::sAxisX(), -lean_back,
                         rotation_axis(Vec3::sAxisX(), -lean_back));
    apply_joint_rotation(pose, rig, rig.torso_upper, Vec3::sAxisX(), -lean_back,
                         rotation_axis(Vec3::sAxisX(), -lean_back));
  }
  pose.CalculateJointMatrices();
}

/** Target joint angles / root height for one frame of the squat or jump sequence. */
struct JumpPoseParams {
  float knee = 0.f;       // rad knee flexion (both legs)
  float hip = 0.f;        // rad hip flexion (both legs)
  float ankle = 0.f;      // rad; positive = plantarflexion (toes down), negative = dorsiflexion
  float root_y = 0.f;     // absolute pelvis height (m)
  float arm_swing = 0.f;  // rad shoulder flexion; negative = arms behind body, positive = arms up/forward
  float elbow = 0.f;      // rad elbow flexion (0 = straight arms)
  float torso_lean = 0.f; // rad forward torso lean
};

void build_jump_pose(const Ragdoll* ragdoll, const RagdollSettings* settings,
                     const SkeletalAnimation* standing_anim, const JumpPoseParams& p,
                     const SimulatorConfig& config, SkeletonPose& pose) {
  const Skeleton* skel = settings->GetSkeleton();
  const RigJoints rig = resolve_rig_joints(skel);
  if (!skel || !rig_has_limbs(rig, true, true))
    return;
  if (standing_anim)
    sample_standing_base_pose(ragdoll, settings, standing_anim, config, pose);
  else
    build_neutral_pose(ragdoll, settings, pose);

  RVec3 root_offset = pose.GetRootOffset();
  root_offset = RVec3(root_offset.GetX(), (JPH::Real)p.root_y, root_offset.GetZ());
  pose.SetRootOffset(root_offset);

  if (rig.compose_on_base) {
    // Axes verified empirically for the Human.tof rig (tests/squat_probe.cpp):
    // hip flexion = -X, knee flexion = +X, arm swing forward/up = +Y, torso forward lean = +X.
    apply_joint_rotation(pose, rig, rig.l_thigh, -Vec3::sAxisX(), p.hip, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.r_thigh, -Vec3::sAxisX(), p.hip, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.l_shin, Vec3::sAxisX(), p.knee, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.r_shin, Vec3::sAxisX(), p.knee, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.l_foot, Vec3::sAxisX(), p.ankle, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.r_foot, Vec3::sAxisX(), p.ankle, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.l_upper_arm, Vec3::sAxisY(), p.arm_swing, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.r_upper_arm, Vec3::sAxisY(), p.arm_swing, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.l_forearm, Vec3::sAxisX(), p.elbow, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.r_forearm, Vec3::sAxisX(), p.elbow, Quat::sIdentity());
    apply_joint_rotation(pose, rig, rig.torso_upper, Vec3::sAxisX(), p.torso_lean, Quat::sIdentity());
  } else {
    apply_joint_rotation(pose, rig, rig.l_thigh, -Vec3::sAxisY(), p.hip,
                         rotation_axis(-Vec3::sAxisY(), p.hip));
    apply_joint_rotation(pose, rig, rig.r_thigh, -Vec3::sAxisY(), p.hip,
                         rotation_axis(-Vec3::sAxisY(), p.hip));
    apply_joint_rotation(pose, rig, rig.l_shin, Vec3::sAxisX(), p.knee,
                         rotation_axis(Vec3::sAxisX(), p.knee));
    apply_joint_rotation(pose, rig, rig.r_shin, Vec3::sAxisX(), p.knee,
                         rotation_axis(Vec3::sAxisX(), p.knee));
    apply_joint_rotation(pose, rig, rig.l_upper_arm, -Vec3::sAxisX(), p.arm_swing,
                         rotation_axis(-Vec3::sAxisX(), p.arm_swing));
    apply_joint_rotation(pose, rig, rig.r_upper_arm, Vec3::sAxisX(), p.arm_swing,
                         rotation_axis(Vec3::sAxisX(), p.arm_swing));
    apply_joint_rotation(pose, rig, rig.l_forearm, -Vec3::sAxisX(), p.elbow,
                         rotation_axis(-Vec3::sAxisX(), p.elbow));
    apply_joint_rotation(pose, rig, rig.r_forearm, Vec3::sAxisX(), p.elbow,
                         rotation_axis(Vec3::sAxisX(), p.elbow));
    apply_joint_rotation(pose, rig, rig.torso, Vec3::sAxisX(), p.torso_lean,
                         rotation_axis(Vec3::sAxisX(), p.torso_lean));
    apply_joint_rotation(pose, rig, rig.torso_upper, Vec3::sAxisX(), p.torso_lean,
                         rotation_axis(Vec3::sAxisX(), p.torso_lean));
  }
  pose.CalculateJointMatrices();
}

// Jump choreography constants (see protocol_jump.md).
constexpr float kJumpArmBack = -0.6f;      // rad arms down/slightly back during loading
constexpr float kJumpArmUp = 1.6f;         // rad arms swung up toward overhead at takeoff/flight
constexpr float kJumpElbowSoft = 0.25f;    // rad soft elbows while arms are back
constexpr float kJumpAnklePlantar = 0.40f; // rad plantarflexion at takeoff/flight (toes down)

/** Staggered ease: remaps t so the segment eases in over [start, end] of the phase. */
float staggered_ease(float t, float start, float end) {
  return smoothstep01((t - start) / std::max(1e-4f, end - start));
}

/** Semi-squat target at the given ease (0 = standing, 1 = full squat). Shared by the Squat stance and the jump crouch phase. */
JumpPoseParams squat_pose_params(float ease, const SimulatorConfig& config) {
  JumpPoseParams p;
  p.knee = config.jump_crouch_knee * ease;
  p.hip = config.jump_crouch_hip * ease;
  // Dorsiflex so the foot stays flat: the foot inherits the shin's world tilt
  // (knee - hip), so the ankle must counter-rotate by exactly that amount.
  p.ankle = p.hip - p.knee;
  p.root_y = config.standing_min_height - config.jump_crouch_drop * ease;
  p.arm_swing = kJumpArmBack * ease;
  p.elbow = kJumpElbowSoft * ease;
  p.torso_lean = 0.10f * ease;  // chest stays up; only a slight lean
  return p;
}

void apply_action_pose(Ragdoll* ragdoll, const RagdollSettings* settings,
                       const SkeletalAnimation* standing_anim, float dt,
                       const SimulatorConfig& config, ControllerState& state,
                       MotionMode mode) {
  state.action_time += dt;
  const float t = std::min(1.f, state.action_time / config.action_pose_duration);
  const float ease = smoothstep01(t);
  SkeletonPose pose;
  switch (mode) {
    case MotionMode::StandingRaiseLeg:
      build_standing_raise_leg_pose(ragdoll, settings, standing_anim, ease, config, pose);
      break;
    case MotionMode::PunchRight:
      build_punch_pose(ragdoll, settings, standing_anim, ease, config, false, pose);
      break;
    case MotionMode::PunchLeft:
      build_punch_pose(ragdoll, settings, standing_anim, ease, config, true, pose);
      break;
    case MotionMode::FrontKick:
      build_front_kick_pose(ragdoll, settings, standing_anim, ease, config, pose);
      break;
    case MotionMode::Squat:
      build_jump_pose(ragdoll, settings, standing_anim, squat_pose_params(ease, config),
                      config, pose);
      break;
    default:
      return;
  }
  apply_kinematic_pose(ragdoll, pose, dt);
}

void apply_standing_pose(Ragdoll* ragdoll, const RagdollSettings* settings,
                        const SkeletalAnimation* standing_anim, float dt,
                        const SimulatorConfig& config) {
  SkeletonPose pose;
  const Skeleton* skel = settings->GetSkeleton();
  if (!skel || skel->GetJointCount() == 0) return;
  pose.SetSkeleton(skel);
  if (standing_anim) {
    standing_anim->Sample(0.0f, pose);
    SkeletonPose::JointState& joint0 = pose.GetJoint(0);
    joint0.mTranslation = Vec3::sZero();
    RVec3 root_offset;
    ragdoll->GetRootTransform(root_offset, joint0.mRotation, true);
    // Hold root at fixed standing height so we don't follow gravity and sink
    root_offset = RVec3(root_offset.GetX(), (JPH::Real)config.standing_min_height, root_offset.GetZ());
    pose.SetRootOffset(root_offset);
    pose.CalculateJointMatrices();
    ragdoll->DriveToPoseUsingKinematics(pose, dt);  // file rig often has no motors; Kinematics holds pose
  } else {
    build_neutral_pose(ragdoll, settings, pose);
    ragdoll->DriveToPoseUsingMotors(pose);
  }
}

void apply_walking_pose(Ragdoll* ragdoll, const RagdollSettings* settings,
                       ControllerState& state, const SimulatorConfig& config, float dt,
                       const SkeletalAnimation* walking_anim) {
  SkeletonPose pose;
  const Skeleton* skel = settings->GetSkeleton();
  if (!skel || skel->GetJointCount() == 0) return;
  pose.SetSkeleton(skel);
  if (walking_anim) {
    state.walk_time += dt * config.walk_anim_playback;
    walking_anim->Sample(state.walk_time, pose);
    if (!state.walk_root_origin_valid) {
      Quat root_rot;
      ragdoll->GetRootTransform(state.walk_root_origin, root_rot, true);
      state.walk_anim_root_at_start = pose.GetRootOffset();
      state.walk_root_origin_valid = true;
    }
    RVec3 anim_root = pose.GetRootOffset();
    RVec3 root_offset = state.walk_root_origin + (anim_root - state.walk_anim_root_at_start);
    if (config.walk_forward_speed > 0.f) {
      SkeletonPose::JointState& joint0 = pose.GetJoint(0);
      Vec3 forward = joint0.mRotation * Vec3(0.f, 0.f, -1.f);
      root_offset += RVec3(forward * (config.walk_forward_speed * state.walk_time));
    }
    pose.SetRootOffset(root_offset);
    SkeletonPose::JointState& joint0 = pose.GetJoint(0);
    joint0.mTranslation = Vec3::sZero();
    pose.CalculateJointMatrices();
    ragdoll->DriveToPoseUsingKinematics(pose, dt);
  } else {
    const uint joint_count = skel->GetJointCount();
    if (joint_count == 12) {
      build_walk_pose(ragdoll, settings, state.walk_phase, config, pose);
      state.walk_phase += config.walk_speed * dt * TWO_PI;
      if (state.walk_phase >= TWO_PI) state.walk_phase -= TWO_PI;
      if (state.walk_phase < 0.f) state.walk_phase += TWO_PI;
    } else {
      build_neutral_pose(ragdoll, settings, pose);
    }
    ragdoll->DriveToPoseUsingKinematics(pose, dt);
  }
}

void reset_ragdoll_to_standing_impl(Ragdoll* ragdoll, const RagdollSettings* settings, RVec3 root_offset) {
  const Skeleton* skel = settings->GetSkeleton();
  if (!skel || skel->GetJointCount() == 0)
    return;
  SkeletonPose pose;
  pose.SetSkeleton(skel);
  pose.SetRootOffset(root_offset);
  for (uint i = 0; i < skel->GetJointCount(); ++i) {
    pose.GetJoint(i).mRotation = Quat::sIdentity();
    pose.GetJoint(i).mTranslation = Vec3::sZero();
  }
  pose.CalculateJointMatrices();
  ragdoll->SetPose(pose);
  ragdoll->ResetWarmStart();
}

/** Ballistic flight time for the configured takeoff velocity: 2*v/g. */
float jump_flight_duration(const SimulatorConfig& config) {
  const float g = std::abs(config.gravity_y);
  return (g > 1e-3f) ? 2.f * config.jump_velocity_y / g : 0.f;
}

/**
 * Advance the kinematic jump phase machine by dt and drive the ragdoll toward
 * the phase's target pose. Root XZ / rotation are pinned by the Visualizer
 * (mode stays Standing), so the jump is vertical and on the spot.
 */
void apply_jump_sequence(SimulatorScene& scene, ControllerState& state,
                         const SimulatorConfig& config, float dt) {
  state.jump_phase_time += dt;
  const float stand_y = config.standing_min_height;
  JumpPoseParams p;
  p.root_y = stand_y;

  switch (state.jump_phase) {
    // Counter-movement (loading): quick dip, arms down/back, ankles dorsiflex,
    // torso stays upright (protocol_jump.md section 2).
    case JumpPhase::Crouch: {
      const float t = std::min(1.f, state.jump_phase_time / config.jump_crouch_duration);
      p = squat_pose_params(smoothstep01(t), config);
      if (state.jump_phase_time >= config.jump_crouch_duration) {
        state.jump_phase = JumpPhase::Launch;
        state.jump_phase_time = 0.f;
        log("[Pose] Jump launch (extend legs)");
      }
      break;
    }
    // Push-off: proximal-to-distal extension (hips -> knees -> ankles), arms
    // swing up aggressively, elbows extend (protocol_jump.md section 3).
    case JumpPhase::Launch: {
      const float t = std::min(1.f, state.jump_phase_time / config.jump_extend_duration);
      const float hip_e = staggered_ease(t, 0.00f, 0.60f);    // hips fire first
      const float knee_e = staggered_ease(t, 0.15f, 0.85f);   // knees follow
      const float ankle_e = staggered_ease(t, 0.45f, 1.00f);  // ankles are the final push
      const float arm_e = smoothstep01(t);
      p.hip = config.jump_crouch_hip * (1.f - hip_e);
      p.knee = config.jump_crouch_knee * (1.f - knee_e);
      // Feet stay flat (ankle counters shin tilt) until the final plantarflexion push.
      p.ankle = (p.hip - p.knee) * (1.f - ankle_e) + kJumpAnklePlantar * ankle_e;
      p.root_y = stand_y - config.jump_crouch_drop * (1.f - knee_e);
      p.arm_swing = kJumpArmBack + (kJumpArmUp - kJumpArmBack) * arm_e;
      p.elbow = kJumpElbowSoft * (1.f - arm_e);  // elbows extend as arms reach up
      p.torso_lean = 0.10f * (1.f - hip_e);
      if (state.jump_phase_time >= config.jump_extend_duration) {
        state.jump_phase = JumpPhase::Flight;
        state.jump_phase_time = 0.f;
        log("[Pose] Jump airborne vy=%.2f m/s", config.jump_velocity_y);
      }
      break;
    }
    // Flight: full extension, arms overhead, toes pointed; small knee bend late
    // in flight to prepare for landing (protocol_jump.md section 4).
    case JumpPhase::Flight: {
      const float g = std::abs(config.gravity_y);
      const float total = jump_flight_duration(config);
      const float ft = std::min(state.jump_phase_time, total);
      p.root_y = stand_y + config.jump_velocity_y * ft - 0.5f * g * ft * ft;
      const float progress = (total > 1e-4f) ? ft / total : 1.f;
      const float land_prep = staggered_ease(progress, 0.70f, 1.00f);
      p.knee = 0.06f + 0.24f * land_prep;
      p.hip = 0.4f * p.knee;
      p.ankle = kJumpAnklePlantar;  // toes down for a forefoot-first touch
      p.arm_swing = kJumpArmUp;
      p.elbow = 0.05f;
      p.torso_lean = -0.05f;  // slightly extended, chest up
      if (state.jump_phase_time >= total) {
        state.jump_phase = JumpPhase::Land;
        state.jump_phase_time = 0.f;
        log("[Pose] Jump landing");
      }
      break;
    }
    // Landing: absorb with controlled hip/knee flexion, roll forefoot -> flat,
    // arms come down and slightly back, then stick (protocol_jump.md section 5).
    case JumpPhase::Land: {
      const float t = std::min(1.f, state.jump_phase_time / config.jump_land_duration);
      const float bump = std::sin(JPH_PI * t);  // bend then straighten to absorb
      const float settle = smoothstep01(t);
      p.knee = config.jump_land_knee * bump;
      p.hip = 0.5f * config.jump_land_knee * bump;
      // Roll from forefoot (plantarflexed) to whole foot flat on the ground.
      const float roll = staggered_ease(t, 0.f, 0.4f);
      p.ankle = kJumpAnklePlantar * (1.f - roll) + (p.hip - p.knee) * roll;
      p.root_y = stand_y - 0.8f * config.jump_crouch_drop * bump;
      // Arms drop from overhead to slightly back/down to counterbalance, then relax.
      p.arm_swing = kJumpArmUp * (1.f - settle) - 0.2f * bump;
      p.elbow = kJumpElbowSoft * bump;
      p.torso_lean = 0.12f * bump;
      if (state.jump_phase_time >= config.jump_land_duration) {
        state.jump_phase = JumpPhase::None;
        state.jump_phase_time = 0.f;
        log("[Pose] Jump complete -> Standing");
      }
      break;
    }
    case JumpPhase::None:
      return;
  }

  SkeletonPose pose;
  build_jump_pose(scene.ragdoll, scene.ragdoll_settings, scene.standing_anim.GetPtr(),
                  p, config, pose);
  scene.ragdoll->DriveToPoseUsingKinematics(pose, dt);
}

}  // namespace

void zero_ragdoll_velocities(JPH::Ragdoll* ragdoll, JPH::BodyInterface& body_interface) {
  if (!ragdoll) return;
  for (JPH::BodyID id : ragdoll->GetBodyIDs()) {
    if (!id.IsInvalid()) {
      body_interface.SetLinearVelocity(id, JPH::Vec3::sZero());
      body_interface.SetAngularVelocity(id, JPH::Vec3::sZero());
    }
  }
}

void clamp_ragdoll_velocities(JPH::Ragdoll* ragdoll,
                              JPH::BodyInterface& body_interface,
                              float max_linear_speed,
                              float max_angular_speed) {
  if (!ragdoll) return;
  for (JPH::BodyID id : ragdoll->GetBodyIDs()) {
    if (id.IsInvalid()) continue;
    JPH::Vec3 lin = body_interface.GetLinearVelocity(id);
    float lin_sq = lin.LengthSq();
    if (lin_sq > max_linear_speed * max_linear_speed) {
      float lin_len = std::sqrt(lin_sq);
      body_interface.SetLinearVelocity(id, lin * (max_linear_speed / lin_len));
    }
    JPH::Vec3 ang = body_interface.GetAngularVelocity(id);
    float ang_sq = ang.LengthSq();
    if (ang_sq > max_angular_speed * max_angular_speed) {
      float ang_len = std::sqrt(ang_sq);
      body_interface.SetAngularVelocity(id, ang * (max_angular_speed / ang_len));
    }
  }
}

void reset_ragdoll_to_standing(JPH::Ragdoll* ragdoll,
                               const JPH::RagdollSettings* ragdoll_settings,
                               JPH::RVec3 root_offset,
                               JPH::BodyInterface* body_interface) {
  if (!ragdoll || !ragdoll_settings) return;
  reset_ragdoll_to_standing_impl(ragdoll, ragdoll_settings, root_offset);
  if (body_interface)
    zero_ragdoll_velocities(ragdoll, *body_interface);
}

void reset_ragdoll_to_initial_standing(SimulatorScene& scene,
                                      JPH::BodyInterface* body_interface) {
  if (!scene.ragdoll || !scene.ragdoll_settings) return;
  const JPH::Skeleton* skel = scene.ragdoll_settings->GetSkeleton();
  if (!skel || skel->GetJointCount() == 0) return;
  const size_t num_joints = scene.initial_standing_joint_rotations.size();
  if (num_joints != skel->GetJointCount() || num_joints != scene.initial_standing_joint_translations.size()) {
    reset_ragdoll_to_standing(scene.ragdoll, scene.ragdoll_settings, scene.initial_standing_root_offset, body_interface);
    return;
  }
  JPH::SkeletonPose pose;
  pose.SetSkeleton(skel);
  pose.SetRootOffset(scene.initial_standing_root_offset);
  for (size_t i = 0; i < num_joints; ++i) {
    pose.GetJoint(static_cast<JPH::uint>(i)).mRotation = scene.initial_standing_joint_rotations[i];
    pose.GetJoint(static_cast<JPH::uint>(i)).mTranslation = scene.initial_standing_joint_translations[i];
  }
  pose.CalculateJointMatrices();
  scene.ragdoll->SetPose(pose);
  scene.ragdoll->ResetWarmStart();
  if (body_interface)
    zero_ragdoll_velocities(scene.ragdoll, *body_interface);
}

void capture_standing_pose_as_initial(SimulatorScene& scene) {
  if (!scene.ragdoll || !scene.ragdoll_settings) return;
  const JPH::Skeleton* skel = scene.ragdoll_settings->GetSkeleton();
  if (!skel || skel->GetJointCount() == 0) return;
  JPH::SkeletonPose pose;
  pose.SetSkeleton(skel);
  scene.ragdoll->GetPose(pose);
  scene.initial_standing_root_offset = pose.GetRootOffset();
  scene.initial_standing_root_rotation = pose.GetJoint(0).mRotation;
  const JPH::uint num_joints = skel->GetJointCount();
  scene.initial_standing_joint_rotations.resize(num_joints);
  scene.initial_standing_joint_translations.resize(num_joints);
  for (JPH::uint i = 0; i < num_joints; ++i) {
    const JPH::SkeletonPose::JointState& j = pose.GetJoint(i);
    scene.initial_standing_joint_rotations[i] = j.mRotation;
    scene.initial_standing_joint_translations[i] = j.mTranslation;
  }
}

void apply_pose_control(SimulatorScene& scene,
                        ControllerState& state,
                        const SimulatorConfig& config,
                        float inDeltaTime) {
  if (!scene.physics || !scene.ragdoll || !scene.ragdoll_settings)
    return;
  float dt = inDeltaTime >= 0.f ? inDeltaTime : config.time_step;
  JPH::BodyInterface& bi = scene.physics->GetBodyInterface();

  if (state.jump_triggered && state.jump_phase == JumpPhase::None) {
    state.jump_phase = JumpPhase::Crouch;
    state.jump_phase_time = 0.f;
    state.mode = MotionMode::Standing;
    state.walk_root_origin_valid = false;
    log("[Pose] Jump crouch");
  }
  state.jump_triggered = false;  // ignore re-triggers mid-jump

  if (state.jump_phase != JumpPhase::None) {
    if (state.mode != MotionMode::Standing) {
      // User switched mode mid-jump (Ragdoll/Walk): cancel the sequence.
      state.jump_phase = JumpPhase::None;
      state.jump_phase_time = 0.f;
    } else {
      apply_jump_sequence(scene, state, config, dt);
      return;
    }
  }

  switch (state.mode) {
    case MotionMode::Standing:
      state.walk_time = 0.f;
      state.walk_root_origin_valid = false;
      apply_standing_pose(scene.ragdoll, scene.ragdoll_settings,
                          scene.standing_anim.GetPtr(), dt, config);
      break;
    case MotionMode::StandingRaiseLeg:
    case MotionMode::PunchRight:
    case MotionMode::PunchLeft:
    case MotionMode::FrontKick:
    case MotionMode::Squat:
      state.walk_root_origin_valid = false;
      apply_action_pose(scene.ragdoll, scene.ragdoll_settings, scene.standing_anim.GetPtr(),
                        dt, config, state, state.mode);
      break;
    case MotionMode::Walking:
      for (JPH::BodyID id : scene.ragdoll->GetBodyIDs()) {
        if (!id.IsInvalid())
          bi.SetMotionType(id, EMotionType::Kinematic, EActivation::DontActivate);
      }
      apply_walking_pose(scene.ragdoll, scene.ragdoll_settings, state, config, dt,
                        scene.walking_anim.GetPtr());
      break;
    case MotionMode::Ragdoll:
      state.walk_root_origin_valid = false;
      break;
  }
}

}  // namespace biomechanics
