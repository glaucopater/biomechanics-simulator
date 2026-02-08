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
  float hip_l = config.walk_hip_amplitude * std::sin(phase);
  float hip_r = config.walk_hip_amplitude * std::sin(phase + JPH_PI);
  float knee_l = config.walk_knee_amplitude * (std::sin(phase) > 0 ? std::sin(phase) : 0.f);
  float knee_r = config.walk_knee_amplitude * (std::sin(phase + JPH_PI) > 0 ? std::sin(phase + JPH_PI) : 0.f);
  float arm_l = config.walk_arm_amplitude * std::sin(phase + JPH_PI);
  float arm_r = config.walk_arm_amplitude * std::sin(phase);
  // Twist axes from HumanRagdoll: legs -Y, arms left -X right +X
  pose.GetJoint(UpperLegL).mRotation = rotation_axis(-Vec3::sAxisY(), hip_l);
  pose.GetJoint(UpperLegR).mRotation = rotation_axis(-Vec3::sAxisY(), hip_r);
  pose.GetJoint(LowerLegL).mRotation = rotation_axis(Vec3::sAxisX(), knee_l);
  pose.GetJoint(LowerLegR).mRotation = rotation_axis(Vec3::sAxisX(), knee_r);
  pose.GetJoint(UpperArmL).mRotation = rotation_axis(-Vec3::sAxisX(), arm_l);
  pose.GetJoint(UpperArmR).mRotation = rotation_axis(Vec3::sAxisX(), arm_r);
  pose.CalculateJointMatrices();
}

void build_standing_raise_leg_pose(const Ragdoll* ragdoll, const RagdollSettings* settings, SkeletonPose& pose) {
  const Skeleton* skel = settings->GetSkeleton();
  if (!skel || skel->GetJointCount() < 12) return;
  pose.SetSkeleton(skel);
  RVec3 root_pos;
  Quat root_rot;
  ragdoll->GetRootTransform(root_pos, root_rot, true);
  pose.SetRootOffset(root_pos);
  for (uint i = 0; i < skel->GetJointCount(); ++i) {
    pose.GetJoint(i).mRotation = (i == 0) ? root_rot : Quat::sIdentity();
    pose.GetJoint(i).mTranslation = Vec3::sZero();
  }
  // Raise right leg: hip flexion (forward) ~0.85 rad, knee flexion ~0.9 rad. Left leg straight.
  const float hip_raise = 0.85f;
  const float knee_raise = 0.9f;
  pose.GetJoint(UpperLegL).mRotation = Quat::sIdentity();
  pose.GetJoint(UpperLegR).mRotation = rotation_axis(-Vec3::sAxisY(), hip_raise);
  pose.GetJoint(LowerLegL).mRotation = Quat::sIdentity();
  pose.GetJoint(LowerLegR).mRotation = rotation_axis(Vec3::sAxisX(), knee_raise);
  // Lean torso slightly toward support leg (left) so COM stays over standing foot and balance is stable.
  const float lean_support_rad = -0.12f;  // rotation around Z: lean left
  pose.GetJoint(MidBody).mRotation = rotation_axis(Vec3::sAxisZ(), lean_support_rad);
  pose.GetJoint(UpperBody).mRotation = rotation_axis(Vec3::sAxisZ(), lean_support_rad);
  pose.CalculateJointMatrices();
}

void apply_standing_raise_leg_pose(Ragdoll* ragdoll, const RagdollSettings* settings,
                                   const SimulatorConfig& config) {
  const Skeleton* skel = settings->GetSkeleton();
  if (!skel || skel->GetJointCount() < 12) return;
  SkeletonPose pose;
  build_standing_raise_leg_pose(ragdoll, settings, pose);
  RVec3 root_off = pose.GetRootOffset();
  root_off = RVec3(root_off.GetX(), (JPH::Real)config.standing_min_height, root_off.GetZ());
  pose.SetRootOffset(root_off);
  pose.CalculateJointMatrices();
  ragdoll->DriveToPoseUsingMotors(pose);
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
    state.walk_time += dt;
    walking_anim->Sample(state.walk_time, pose);
    SkeletonPose::JointState& joint0 = pose.GetJoint(0);
    joint0.mTranslation = Vec3::sZero();
    RVec3 root_offset;
    ragdoll->GetRootTransform(root_offset, joint0.mRotation, true);
    if (config.walk_forward_speed > 0.f) {
      Vec3 forward = joint0.mRotation * Vec3(0.f, 0.f, -1.f);
      root_offset += RVec3(forward * (config.walk_forward_speed * dt));
    }
    pose.SetRootOffset(root_offset);
    pose.CalculateJointMatrices();
    // #region agent log
    debug_instrument_walk("PoseController.cpp:apply_walking_pose", "D,E", "target",
      0.f, 0.f, 0.f, static_cast<float>(root_offset.GetX()), static_cast<float>(root_offset.GetY()), static_cast<float>(root_offset.GetZ()), dt);
    // #endregion
    ragdoll->DriveToPoseUsingKinematics(pose, dt);  // file rig often has no motors; Kinematics holds pose
  } else {
    // Procedural walk uses 12-joint indices (UpperLegL=8, etc.); use only when skeleton matches.
    const uint joint_count = skel->GetJointCount();
    if (joint_count == 12) {
      build_walk_pose(ragdoll, settings, state.walk_phase, config, pose);
      state.walk_phase += config.walk_speed * dt * TWO_PI;
      if (state.walk_phase >= TWO_PI) state.walk_phase -= TWO_PI;
      if (state.walk_phase < 0.f) state.walk_phase += TWO_PI;
    } else {
      build_neutral_pose(ragdoll, settings, pose);
    }
    ragdoll->DriveToPoseUsingMotors(pose);
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

void apply_jump(Ragdoll* ragdoll, BodyInterface& bi, ControllerState& state,
               const SimulatorConfig& config) {
  if (!state.jump_triggered)
    return;
  state.jump_triggered = false;
  BodyID root_id = ragdoll->GetBodyID(0);
  if (root_id.IsInvalid())
    return;
  bi.AddImpulse(root_id, Vec3(0, config.jump_impulse_y, 0));
  state.jump_frames_hold = 30;
  log("[Pose] Jump applied (rise then Ragdoll)");
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

  apply_jump(scene.ragdoll, bi, state, config);

  switch (state.mode) {
    case MotionMode::Standing:
      state.walk_time = 0.f;
      apply_standing_pose(scene.ragdoll, scene.ragdoll_settings,
                          scene.standing_anim.GetPtr(), dt, config);
      break;
    case MotionMode::StandingRaiseLeg:
      apply_standing_raise_leg_pose(scene.ragdoll, scene.ragdoll_settings, config);
      break;
    case MotionMode::Walking:
      apply_walking_pose(scene.ragdoll, scene.ragdoll_settings, state, config, dt,
                        scene.walking_anim.GetPtr());
      break;
    case MotionMode::Ragdoll:
      break;
  }
  if (state.jump_frames_hold > 0) {
    state.jump_frames_hold--;
    if (state.jump_frames_hold == 0) {
      state.mode = MotionMode::Ragdoll;
      log("[Pose] Jump hold done -> Ragdoll");
    }
  }
}

}  // namespace biomechanics
