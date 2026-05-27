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

// Human ragdoll body indices (from HumanRagdoll): 0=LowerBody, 1=MidBody, 2=UpperBody, 3=Head,
// 4=UpperArmL, 5=UpperArmR, 6=LowerArmL, 7=LowerArmR, 8=UpperLegL, 9=UpperLegR, 10=LowerLegL, 11=LowerLegR
enum HumanBodyIndex : int {
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

Quat rotation_axis(Vec3Arg axis, float angle_rad) {
  return Quat::sRotation(axis, angle_rad);
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
  pose.GetJoint(UpperLegL).mRotation = rotation_axis(-Vec3::sAxisY(), hip_l);
  pose.GetJoint(UpperLegR).mRotation = rotation_axis(-Vec3::sAxisY(), hip_r);
  pose.GetJoint(LowerLegL).mRotation = rotation_axis(Vec3::sAxisX(), knee_l);
  pose.GetJoint(LowerLegR).mRotation = rotation_axis(Vec3::sAxisX(), knee_r);
  pose.GetJoint(UpperArmL).mRotation = rotation_axis(-Vec3::sAxisX(), arm_l);
  pose.GetJoint(UpperArmR).mRotation = rotation_axis(Vec3::sAxisX(), arm_r);
  pose.GetJoint(MidBody).mRotation = rotation_axis(Vec3::sAxisY(), torso_sway);
  pose.CalculateJointMatrices();
}

void build_standing_raise_leg_pose(const Ragdoll* ragdoll, const RagdollSettings* settings,
                                   const SkeletalAnimation* standing_anim,
                                   float ease, const SimulatorConfig& config, SkeletonPose& pose) {
  const Skeleton* skel = settings->GetSkeleton();
  if (!skel || skel->GetJointCount() < 12)
    return;
  sample_standing_base_pose(ragdoll, settings, standing_anim, config, pose);
  const float hip_raise = config.raise_leg_hip * ease;
  const float knee_raise = config.raise_leg_knee * ease;
  const float lean_support_rad = -0.12f * ease;
  pose.GetJoint(UpperLegL).mRotation = Quat::sIdentity();
  pose.GetJoint(UpperLegR).mRotation = rotation_axis(-Vec3::sAxisY(), hip_raise);
  pose.GetJoint(LowerLegL).mRotation = Quat::sIdentity();
  pose.GetJoint(LowerLegR).mRotation = rotation_axis(Vec3::sAxisX(), knee_raise);
  pose.GetJoint(MidBody).mRotation = rotation_axis(Vec3::sAxisZ(), lean_support_rad);
  pose.GetJoint(UpperBody).mRotation = rotation_axis(Vec3::sAxisZ(), lean_support_rad);
  pose.CalculateJointMatrices();
}

void build_fist_pose(const Ragdoll* ragdoll, const RagdollSettings* settings,
                     const SkeletalAnimation* standing_anim, float ease,
                     const SimulatorConfig& config, SkeletonPose& pose) {
  const Skeleton* skel = settings->GetSkeleton();
  if (!skel || skel->GetJointCount() < 12)
    return;
  sample_standing_base_pose(ragdoll, settings, standing_anim, config, pose);
  const float punch = config.fist_punch_arm * ease;
  const float elbow = config.fist_punch_elbow * ease;
  const float guard = config.fist_guard_arm * ease;
  const float torso_twist = 0.18f * ease;
  pose.GetJoint(UpperArmR).mRotation = rotation_axis(Vec3::sAxisX(), punch);
  pose.GetJoint(LowerArmR).mRotation = rotation_axis(Vec3::sAxisX(), -elbow);
  pose.GetJoint(UpperArmL).mRotation = rotation_axis(-Vec3::sAxisX(), guard);
  pose.GetJoint(LowerArmL).mRotation = rotation_axis(-Vec3::sAxisX(), 0.55f * ease);
  pose.GetJoint(UpperBody).mRotation = rotation_axis(Vec3::sAxisY(), torso_twist);
  pose.GetJoint(MidBody).mRotation = rotation_axis(Vec3::sAxisY(), 0.5f * torso_twist);
  pose.CalculateJointMatrices();
}

void build_front_kick_pose(const Ragdoll* ragdoll, const RagdollSettings* settings,
                           const SkeletalAnimation* standing_anim, float ease,
                           const SimulatorConfig& config, SkeletonPose& pose) {
  const Skeleton* skel = settings->GetSkeleton();
  if (!skel || skel->GetJointCount() < 12)
    return;
  sample_standing_base_pose(ragdoll, settings, standing_anim, config, pose);
  const float hip = config.kick_hip * ease;
  const float knee = config.kick_knee * ease;
  const float plant_knee = config.kick_plant_knee * ease;
  const float lean_back = 0.10f * ease;
  const float guard = 0.45f * ease;
  pose.GetJoint(UpperLegR).mRotation = rotation_axis(-Vec3::sAxisY(), hip);
  pose.GetJoint(LowerLegR).mRotation = rotation_axis(Vec3::sAxisX(), knee);
  pose.GetJoint(UpperLegL).mRotation = Quat::sIdentity();
  pose.GetJoint(LowerLegL).mRotation = rotation_axis(Vec3::sAxisX(), plant_knee);
  pose.GetJoint(UpperArmL).mRotation = rotation_axis(-Vec3::sAxisX(), guard);
  pose.GetJoint(UpperArmR).mRotation = rotation_axis(Vec3::sAxisX(), guard);
  pose.GetJoint(LowerArmL).mRotation = rotation_axis(-Vec3::sAxisX(), 0.35f * ease);
  pose.GetJoint(LowerArmR).mRotation = rotation_axis(Vec3::sAxisX(), 0.35f * ease);
  pose.GetJoint(MidBody).mRotation = rotation_axis(Vec3::sAxisX(), -lean_back);
  pose.GetJoint(UpperBody).mRotation = rotation_axis(Vec3::sAxisX(), -lean_back);
  pose.CalculateJointMatrices();
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
    case MotionMode::Fist:
      build_fist_pose(ragdoll, settings, standing_anim, ease, config, pose);
      break;
    case MotionMode::FrontKick:
      build_front_kick_pose(ragdoll, settings, standing_anim, ease, config, pose);
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

void build_jump_crouch_pose(const Ragdoll* ragdoll, const RagdollSettings* settings,
                            const SkeletalAnimation* standing_anim, float ease,
                            const SimulatorConfig& config, SkeletonPose& pose) {
  const Skeleton* skel = settings->GetSkeleton();
  if (!skel || skel->GetJointCount() < 12)
    return;
  pose.SetSkeleton(skel);
  if (standing_anim)
    standing_anim->Sample(0.0f, pose);
  else
    build_neutral_pose(ragdoll, settings, pose);

  RVec3 root_offset;
  Quat root_rot;
  ragdoll->GetRootTransform(root_offset, root_rot, true);
  SkeletonPose::JointState& joint0 = pose.GetJoint(0);
  joint0.mTranslation = Vec3::sZero();
  if (!standing_anim)
    joint0.mRotation = root_rot;

  const float knee = config.jump_crouch_knee * ease;
  const float hip = config.jump_crouch_hip * ease;
  const float drop = config.jump_crouch_drop * ease;
  root_offset = RVec3(root_offset.GetX(), config.standing_min_height - drop, root_offset.GetZ());
  pose.SetRootOffset(root_offset);

  pose.GetJoint(UpperLegL).mRotation = rotation_axis(-Vec3::sAxisY(), hip);
  pose.GetJoint(UpperLegR).mRotation = rotation_axis(-Vec3::sAxisY(), hip);
  pose.GetJoint(LowerLegL).mRotation = rotation_axis(Vec3::sAxisX(), knee);
  pose.GetJoint(LowerLegR).mRotation = rotation_axis(Vec3::sAxisX(), knee);
  pose.GetJoint(UpperArmL).mRotation = rotation_axis(-Vec3::sAxisX(), -0.25f * ease);
  pose.GetJoint(UpperArmR).mRotation = rotation_axis(Vec3::sAxisX(), -0.25f * ease);
  pose.CalculateJointMatrices();
}

void apply_jump_crouch_pose(Ragdoll* ragdoll, const RagdollSettings* settings,
                            const SkeletalAnimation* standing_anim, float dt,
                            const SimulatorConfig& config, float ease) {
  SkeletonPose pose;
  build_jump_crouch_pose(ragdoll, settings, standing_anim, ease, config, pose);
  ragdoll->DriveToPoseUsingKinematics(pose, dt);
}

void launch_jump(Ragdoll* ragdoll, BodyInterface& bi, ControllerState& state,
                 const SimulatorConfig& config) {
  BodyID root_id = ragdoll->GetBodyID(0);
  if (root_id.IsInvalid())
    return;

  for (BodyID id : ragdoll->GetBodyIDs()) {
    if (!id.IsInvalid())
      bi.SetMotionType(id, EMotionType::Dynamic, EActivation::Activate);
  }

  constexpr float kLimbJumpVelocityRatio = 0.35f;
  for (BodyID id : ragdoll->GetBodyIDs()) {
    if (id.IsInvalid())
      continue;
    const float vy = (id == root_id) ? config.jump_velocity_y
                                     : config.jump_velocity_y * kLimbJumpVelocityRatio;
    bi.SetLinearVelocity(id, Vec3(0.f, vy, 0.f));
    bi.SetAngularVelocity(id, Vec3::sZero());
  }

  state.mode = MotionMode::Ragdoll;
  state.walk_root_origin_valid = false;
  state.jump_in_air = true;
  state.jump_was_airborne = false;
  state.jump_air_steps = 0;
  log("[Pose] Jump launch vy=%.2f m/s", config.jump_velocity_y);
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

namespace {

void recover_standing_at(SimulatorScene& scene, JPH::BodyInterface& bi,
                         JPH::RVec3 root_offset, JPH::Quat root_rotation) {
  if (!scene.ragdoll || !scene.ragdoll_settings) return;
  const JPH::Skeleton* skel = scene.ragdoll_settings->GetSkeleton();
  if (!skel || skel->GetJointCount() == 0) return;
  const size_t num_joints = scene.initial_standing_joint_rotations.size();
  if (num_joints != skel->GetJointCount() || num_joints != scene.initial_standing_joint_translations.size()) {
    reset_ragdoll_to_standing(scene.ragdoll, scene.ragdoll_settings, root_offset, &bi);
    return;
  }
  JPH::SkeletonPose pose;
  pose.SetSkeleton(skel);
  pose.SetRootOffset(root_offset);
  for (size_t i = 0; i < num_joints; ++i) {
    const JPH::uint ji = static_cast<JPH::uint>(i);
    pose.GetJoint(ji).mRotation = (i == 0) ? root_rotation : scene.initial_standing_joint_rotations[i];
    pose.GetJoint(ji).mTranslation = scene.initial_standing_joint_translations[i];
  }
  pose.CalculateJointMatrices();
  scene.ragdoll->SetPose(pose);
  scene.ragdoll->ResetWarmStart();
  zero_ragdoll_velocities(scene.ragdoll, bi);
}

}  // namespace

bool try_recover_standing_after_jump(SimulatorScene& scene,
                                     ControllerState& state,
                                     const SimulatorConfig& config,
                                     JumpLandingResult* out) {
  if (!state.jump_in_air || state.mode != MotionMode::Ragdoll || !scene.physics || !scene.ragdoll)
    return false;

  ++state.jump_air_steps;
  JPH::BodyInterface& bi = scene.physics->GetBodyInterface();
  JPH::BodyID root_id = scene.ragdoll->GetBodyID(0);
  if (root_id.IsInvalid())
    return false;

  JPH::RVec3 root_pos = bi.GetPosition(root_id);
  JPH::Vec3 root_vel = bi.GetLinearVelocity(root_id);
  const float root_y = static_cast<float>(root_pos.GetY());
  const float root_vy = root_vel.GetY();

  if (root_vy > 0.5f || root_y > config.standing_min_height + 0.12f)
    state.jump_was_airborne = true;

  constexpr int kMinAirSteps = 12;
  if (!state.jump_was_airborne || state.jump_air_steps < kMinAirSteps)
    return false;

  const size_t num_bodies = scene.ragdoll->GetBodyIDs().size();
  float foot_y = root_y;
  if (num_bodies >= 12) {
    for (int foot_idx : {10, 11}) {
      JPH::BodyID foot_id = scene.ragdoll->GetBodyID(foot_idx);
      if (!foot_id.IsInvalid())
        foot_y = std::min(foot_y, static_cast<float>(bi.GetPosition(foot_id).GetY()));
    }
  }

  const bool feet_on_ground = foot_y < 0.22f;
  const bool root_low_enough = root_y < config.standing_min_height + 0.18f;
  const bool settled = root_vy > -2.5f && root_vy < 0.6f;
  if (!feet_on_ground || !root_low_enough || !settled)
    return false;

  JPH::RVec3 landing_root(root_pos.GetX(), config.standing_min_height, root_pos.GetZ());
  recover_standing_at(scene, bi, landing_root, scene.initial_standing_root_rotation);
  state.mode = MotionMode::Standing;
  state.jump_in_air = false;
  state.jump_was_airborne = false;
  state.jump_air_steps = 0;
  state.jump_crouching = false;
  state.jump_crouch_time = 0.f;
  state.walk_root_origin_valid = false;

  if (out) {
    out->anchor_x = static_cast<float>(root_pos.GetX());
    out->anchor_z = static_cast<float>(root_pos.GetZ());
    out->anchor_rot = scene.initial_standing_root_rotation;
  }
  log("[Pose] Jump landed -> Standing at (%.2f, %.2f)", static_cast<double>(root_pos.GetX()),
      static_cast<double>(root_pos.GetZ()));
  return true;
}

void apply_pose_control(SimulatorScene& scene,
                        ControllerState& state,
                        const SimulatorConfig& config,
                        float inDeltaTime) {
  if (!scene.physics || !scene.ragdoll || !scene.ragdoll_settings)
    return;
  float dt = inDeltaTime >= 0.f ? inDeltaTime : config.time_step;
  JPH::BodyInterface& bi = scene.physics->GetBodyInterface();

  if (state.jump_triggered && !state.jump_crouching && !state.jump_in_air) {
    state.jump_triggered = false;
    state.jump_crouching = true;
    state.jump_crouch_time = 0.f;
    state.mode = MotionMode::Standing;
    state.walk_root_origin_valid = false;
    log("[Pose] Jump crouch");
  }

  if (state.jump_crouching) {
    state.jump_crouch_time += dt;
    const float t = std::min(1.f, state.jump_crouch_time / config.jump_crouch_duration);
    const float eased = t * t * (3.f - 2.f * t);  // smoothstep: slow in/out
    apply_jump_crouch_pose(scene.ragdoll, scene.ragdoll_settings, scene.standing_anim.GetPtr(),
                           dt, config, eased);
    if (state.jump_crouch_time >= config.jump_crouch_duration) {
      launch_jump(scene.ragdoll, bi, state, config);
      state.jump_crouching = false;
    }
    return;
  }

  switch (state.mode) {
    case MotionMode::Standing:
      state.walk_time = 0.f;
      state.walk_root_origin_valid = false;
      apply_standing_pose(scene.ragdoll, scene.ragdoll_settings,
                          scene.standing_anim.GetPtr(), dt, config);
      break;
    case MotionMode::StandingRaiseLeg:
    case MotionMode::Fist:
    case MotionMode::FrontKick:
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
