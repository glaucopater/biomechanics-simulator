#include "biomechanics/Config.hpp"
#include "biomechanics/Log.hpp"
#include "biomechanics/PoseController.hpp"
#include "biomechanics/Ragdoll.hpp"
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <cmath>
#include <cstdio>

namespace biomechanics {

namespace {

using namespace JPH;

constexpr int BODYPART_COUNT = static_cast<int>(BodyPart::Count);
constexpr float TWO_PI = 6.28318530718f;
// Ragdoll joint order: 0=pelvis-spine, 1=spine-head, 2=left_shoulder, 3=left_elbow, 4=right_shoulder, 5=right_elbow, 6=left_hip, 7=left_knee, 8=right_hip, 9=right_knee
constexpr int WALK_ANIMATED_JOINTS[] = {2, 4, 6, 7, 8, 9};
constexpr int WALK_ANIMATED_JOINT_COUNT = static_cast<int>(sizeof(WALK_ANIMATED_JOINTS) / sizeof(WALK_ANIMATED_JOINTS[0]));

void capture_rest_pose_impl(const RagdollHandles& ragdoll, BodyInterface& bi, ControllerState& state) {
  state.rest_orientations.clear();
  state.rest_orientations.reserve(ragdoll.bodies.size());
  for (BodyID id : ragdoll.bodies) {
    if (!id.IsInvalid())
      state.rest_orientations.push_back(bi.GetRotation(id));
  }
  if (!ragdoll.bodies.empty() && !ragdoll.bodies[static_cast<int>(BodyPart::Pelvis)].IsInvalid())
    state.rest_pelvis_position = bi.GetPosition(ragdoll.bodies[static_cast<int>(BodyPart::Pelvis)]);
  state.rest_captured = true;
}

Quat rotation_y(float angle) {
  return Quat::sRotation(Vec3::sAxisY(), angle);
}

Quat rotation_x(float angle) {
  return Quat::sRotation(Vec3::sAxisX(), angle);
}

void apply_pd_torque(BodyInterface& bi, BodyID id,
                     const Quat& q_target, float stiffness, float damping, float max_torque) {
  Quat q_current = bi.GetRotation(id);
  Quat q_err = (q_target * q_current.Conjugated()).Normalized();

  float w = q_err.GetW();
  w = (w > 1.f) ? 1.f : ((w < -1.f) ? -1.f : w);
  float angle = 2.f * std::acos(w);

  const float eps = 1e-5f;
  if (angle < eps)
    return;

  Vec3 axis(q_err.GetX(), q_err.GetY(), q_err.GetZ());
  float sin_half = std::sqrt(1.f - w * w);
  if (sin_half > eps)
    axis /= sin_half;
  else
    axis = Vec3::sAxisX();

  Vec3 omega = bi.GetAngularVelocity(id);
  float scale = 1.f;
  if (angle > 1.5f)
    scale = 1.5f / angle;
  Vec3 torque = scale * (stiffness * angle * axis - damping * omega);

  float mag = torque.Length();
  if (mag > max_torque && mag > 1e-6f)
    torque *= max_torque / mag;

  bi.AddTorque(id, torque);
}

void apply_pelvis_position_hold(BodyInterface& bi, BodyID pelvis_id,
                                RVec3Arg rest_pos, float k_lin, float d_lin) {
  Vec3 pos = Vec3(bi.GetPosition(pelvis_id));
  Vec3 rest_v(rest_pos);
  Vec3 vel = bi.GetLinearVelocity(pelvis_id);
  Vec3 err = rest_v - pos;
  Vec3 force = k_lin * err - d_lin * vel;
  const float max_force = 800.f;
  float mag = force.Length();
  if (mag > max_force && mag > 1e-6f)
    force *= max_force / mag;
  bi.AddForce(pelvis_id, force);
}

void apply_standing(PhysicsSystem* /*physics*/,
                    const RagdollHandles& ragdoll, BodyInterface& bi,
                    ControllerState& state, const SimulatorConfig& config) {
  if (!state.rest_captured || state.rest_orientations.size() != ragdoll.bodies.size())
    capture_rest_pose_impl(ragdoll, bi, state);
  if (!ragdoll.bodies.empty() && state.jump_frames_hold <= 0) {
    BodyID pelvis_id = ragdoll.bodies[static_cast<int>(BodyPart::Pelvis)];
    if (!pelvis_id.IsInvalid())
      apply_pelvis_position_hold(bi, pelvis_id, state.rest_pelvis_position,
                                  config.pose_linear_stiffness, config.pose_linear_damping);
  }
}

void apply_walking(PhysicsSystem* /*physics*/,
                   const RagdollHandles& ragdoll, BodyInterface& bi,
                   ControllerState& state, const SimulatorConfig& config) {
  if (!state.rest_captured || state.rest_orientations.size() != ragdoll.bodies.size())
    capture_rest_pose_impl(ragdoll, bi, state);
  if (!ragdoll.bodies.empty() && state.jump_frames_hold <= 0) {
    BodyID pelvis_id = ragdoll.bodies[static_cast<int>(BodyPart::Pelvis)];
    if (!pelvis_id.IsInvalid())
      apply_pelvis_position_hold(bi, pelvis_id, state.rest_pelvis_position,
                                  config.pose_linear_stiffness, config.pose_linear_damping);
  }

  float phase = state.walk_phase;
  float hip_l = config.walk_hip_amplitude * std::sin(phase);
  float hip_r = config.walk_hip_amplitude * std::sin(phase + 3.14159265f);
  float knee_l = config.walk_knee_amplitude * (std::sin(phase) > 0 ? std::sin(phase) : 0.f);
  float knee_r = config.walk_knee_amplitude * (std::sin(phase + 3.14159265f) > 0 ? std::sin(phase + 3.14159265f) : 0.f);
  float arm_l = config.walk_arm_amplitude * std::sin(phase + 3.14159265f);
  float arm_r = config.walk_arm_amplitude * std::sin(phase);

  auto q_target = [&](int part) -> Quat {
    if (part < 0 || part >= static_cast<int>(state.rest_orientations.size()))
      return Quat::sIdentity();
    Quat rest = state.rest_orientations[part];
    switch (static_cast<BodyPart>(part)) {
      case BodyPart::Pelvis:
      case BodyPart::Spine:
      case BodyPart::Head:
        return rest;
      case BodyPart::LeftUpperLeg:
        return rest * rotation_x(hip_l);
      case BodyPart::RightUpperLeg:
        return rest * rotation_x(hip_r);
      case BodyPart::LeftLowerLeg:
        return rest * rotation_x(knee_l);
      case BodyPart::RightLowerLeg:
        return rest * rotation_x(knee_r);
      case BodyPart::LeftUpperArm:
        return rest * rotation_y(arm_l);
      case BodyPart::RightUpperArm:
        return rest * rotation_y(arm_r);
      case BodyPart::LeftLowerArm:
      case BodyPart::RightLowerArm:
      default:
        return rest;
    }
  };

  for (size_t i = 0; i < ragdoll.bodies.size(); ++i) {
    BodyID id = ragdoll.bodies[i];
    if (!id.IsInvalid())
      apply_pd_torque(bi, id, q_target(static_cast<int>(i)),
                      config.pose_stiffness, config.pose_damping, config.walk_max_torque);
  }

  static int s_walk_calls = 0;
  if (++s_walk_calls % 60 == 0 && ragdoll.bodies.size() > static_cast<size_t>(BodyPart::RightLowerLeg)) {
    const int left_upper_leg = static_cast<int>(BodyPart::LeftUpperLeg);
    BodyID id = ragdoll.bodies[left_upper_leg];
    if (!id.IsInvalid()) {
      Quat q_cur = bi.GetRotation(id);
      Quat q_tgt = q_target(left_upper_leg);
    Quat q_err = (q_tgt * q_cur.Conjugated()).Normalized();
      float w = q_err.GetW();
      w = (w > 1.f) ? 1.f : ((w < -1.f) ? -1.f : w);
      float angle_err = 2.f * std::acos(w);
      float pelvis_y = bi.GetPosition(ragdoll.bodies[static_cast<int>(BodyPart::Pelvis)]).GetY();
      float L_foot_y = bi.GetPosition(ragdoll.bodies[static_cast<int>(BodyPart::LeftLowerLeg)]).GetY();
      float R_foot_y = bi.GetPosition(ragdoll.bodies[static_cast<int>(BodyPart::RightLowerLeg)]).GetY();
      log("[Walk] phase=%.2f angle_err=%.3f pelvis_y=%.2f L_foot_y=%.2f R_foot_y=%.2f",
          phase, angle_err, float(pelvis_y), float(L_foot_y), float(R_foot_y));
    }
  }
}

void apply_jump(const RagdollHandles& ragdoll, BodyInterface& bi,
                ControllerState& state, const SimulatorConfig& config) {
  if (!state.jump_triggered)
    return;
  state.jump_triggered = false;
  if (ragdoll.bodies.empty())
    return;
  BodyID pelvis_id = ragdoll.bodies[static_cast<int>(BodyPart::Pelvis)];
  if (pelvis_id.IsInvalid())
    return;
  bi.AddImpulse(pelvis_id, Vec3(0, config.jump_impulse_y, 0));
  state.jump_frames_hold = 30;
  log("[Pose] Jump applied (rise then Ragdoll)");
}

void clamp_ragdoll_velocities_impl(RagdollHandles& ragdoll, BodyInterface& bi,
                                   float max_linear_speed, float max_angular_speed) {
  for (BodyID id : ragdoll.bodies) {
    if (id.IsInvalid())
      continue;
    Vec3 lin = bi.GetLinearVelocity(id);
    float lin_sq = lin.LengthSq();
    if (lin_sq > max_linear_speed * max_linear_speed) {
      float lin_len = std::sqrt(lin_sq);
      bi.SetLinearVelocity(id, lin * (max_linear_speed / lin_len));
    }
    Vec3 ang = bi.GetAngularVelocity(id);
    float ang_sq = ang.LengthSq();
    if (ang_sq > max_angular_speed * max_angular_speed) {
      float ang_len = std::sqrt(ang_sq);
      bi.SetAngularVelocity(id, ang * (max_angular_speed / ang_len));
    }
  }
}

}  // namespace

void clamp_ragdoll_velocities(RagdollHandles& ragdoll, BodyInterface& body_interface,
                              float max_linear_speed, float max_angular_speed) {
  clamp_ragdoll_velocities_impl(ragdoll, body_interface, max_linear_speed, max_angular_speed);
}

void zero_ragdoll_velocities(RagdollHandles& ragdoll, BodyInterface& body_interface) {
  for (BodyID id : ragdoll.bodies) {
    if (!id.IsInvalid()) {
      body_interface.SetLinearVelocity(id, Vec3::sZero());
      body_interface.SetAngularVelocity(id, Vec3::sZero());
    }
  }
}

void capture_rest_pose(const RagdollHandles& ragdoll, BodyInterface& body_interface, ControllerState& state) {
  capture_rest_pose_impl(ragdoll, body_interface, state);
}

void apply_pose_control(PhysicsSystem* physics,
                        const RagdollHandles& ragdoll,
                        ControllerState& state,
                        const SimulatorConfig& config) {
  if (!physics || ragdoll.bodies.empty())
    return;
  BodyInterface& bi = physics->GetBodyInterface();

  apply_jump(ragdoll, bi, state, config);

  switch (state.mode) {
    case MotionMode::Standing:
      apply_standing(physics, ragdoll, bi, state, config);
      break;
    case MotionMode::Walking:
      apply_walking(physics, ragdoll, bi, state, config);
      state.walk_phase += config.walk_speed * config.time_step * TWO_PI;
      if (state.walk_phase >= TWO_PI)
        state.walk_phase -= TWO_PI;
      if (state.walk_phase < 0)
        state.walk_phase += TWO_PI;
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
