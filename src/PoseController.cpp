#include "biomechanics/Config.hpp"
#include "biomechanics/Log.hpp"
#include "biomechanics/PoseController.hpp"
#include "biomechanics/Ragdoll.hpp"
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>
#include <cmath>
#include <cstdio>

namespace biomechanics {

namespace {

constexpr int BODYPART_COUNT = static_cast<int>(BodyPart::Count);
constexpr float STANDING_JOINT_SPRING_STIFFNESS = 150.f;
// Ragdoll joint order: 0=pelvis-spine, 1=spine-head, 2=left_shoulder, 3=left_elbow, 4=right_shoulder, 5=right_elbow, 6=left_hip, 7=left_knee, 8=right_hip, 9=right_knee
constexpr int WALK_ANIMATED_JOINTS[] = {2, 4, 6, 7, 8, 9};
constexpr int WALK_ANIMATED_JOINT_COUNT = static_cast<int>(sizeof(WALK_ANIMATED_JOINTS) / sizeof(WALK_ANIMATED_JOINTS[0]));

void set_joint_spring_stiffness_for_mode(const RagdollHandles& ragdoll,
                                         bool walking,
                                         float /*walk_stiffness*/) {
  static bool s_last_walking = false;
  int animated_count = 0;
  for (size_t i = 0; i < ragdoll.joints.size(); ++i) {
    btTypedConstraint* j = ragdoll.joints[i];
    if (!j)
      continue;
    auto* c = static_cast<btGeneric6DofSpring2Constraint*>(j);
    bool is_limb = false;
    if (walking) {
      for (int a = 0; a < WALK_ANIMATED_JOINT_COUNT; ++a)
        if (static_cast<size_t>(WALK_ANIMATED_JOINTS[a]) == i) {
          is_limb = true;
          ++animated_count;
          break;
        }
    }
    for (int axis = 3; axis <= 5; ++axis) {
      if (walking && is_limb) {
        c->enableSpring(axis, false);  // no spring during walk: PD torques move limbs
      } else {
        c->enableSpring(axis, true);
        c->setStiffness(axis, static_cast<btScalar>(STANDING_JOINT_SPRING_STIFFNESS), true);
      }
    }
  }
  if (s_last_walking != walking) {
    s_last_walking = walking;
    if (walking)
      log("[Pose] Walk mode: %d limb joints springs OFF (PD drives)", animated_count);
    else
      log("[Pose] Stand mode: all %zu joints springs ON -> %.0f", ragdoll.joints.size(), STANDING_JOINT_SPRING_STIFFNESS);
  }
}

void capture_rest_pose_impl(const RagdollHandles& ragdoll, ControllerState& state) {
  state.rest_orientations.clear();
  state.rest_orientations.reserve(ragdoll.bodies.size());
  for (btRigidBody* body : ragdoll.bodies) {
    state.rest_orientations.push_back(body->getWorldTransform().getRotation());
  }
  if (!ragdoll.bodies.empty()) {
    btTransform t;
    ragdoll.bodies[static_cast<int>(BodyPart::Pelvis)]->getMotionState()->getWorldTransform(t);
    state.rest_pelvis_position = t.getOrigin();
  }
  state.rest_captured = true;
}

btQuaternion rotation_y(btScalar angle) {
  return btQuaternion(btVector3(0, 1, 0), angle);
}

btQuaternion rotation_x(btScalar angle) {
  return btQuaternion(btVector3(1, 0, 0), angle);
}

void apply_pd_torque(btRigidBody* body,
                    const btQuaternion& q_target,
                    float stiffness,
                    float damping,
                    float max_torque) {
  btQuaternion q_current = body->getWorldTransform().getRotation();
  btQuaternion q_err = q_target * q_current.inverse();
  q_err.normalize();

  btScalar w = q_err.getW();
  w = (w > 1.f) ? 1.f : ((w < -1.f) ? -1.f : w);
  btScalar angle = 2.f * std::acos(w);

  const btScalar eps = 1e-5f;
  if (angle < eps)
    return;

  btVector3 axis(q_err.x(), q_err.y(), q_err.z());
  btScalar sin_half = std::sqrt(1.f - w * w);
  if (sin_half > eps)
    axis /= sin_half;
  else
    axis = btVector3(1, 0, 0);

  btVector3 omega = body->getAngularVelocity();
  btScalar k_p = static_cast<btScalar>(stiffness);
  btScalar k_d = static_cast<btScalar>(damping);
  btScalar max_angle = 3.f;
  if (angle > max_angle)
    angle = max_angle;
  btScalar scale = 1.f;
  if (angle > 1.5f)
    scale = 1.5f / angle;
  btVector3 torque = scale * (k_p * angle * axis - k_d * omega);

  btScalar mag = torque.length();
  if (mag > max_torque && mag > 1e-6f)
    torque *= max_torque / mag;

  body->applyTorque(torque);
}

void apply_pelvis_position_hold(btRigidBody* pelvis,
                                const btVector3& rest_pos,
                                float k_lin,
                                float d_lin) {
  btTransform t;
  pelvis->getMotionState()->getWorldTransform(t);
  btVector3 pos = t.getOrigin();
  btVector3 vel = pelvis->getLinearVelocity();
  btVector3 err = rest_pos - pos;
  btVector3 force = k_lin * err - d_lin * vel;
  btScalar max_force = 800.f;
  btScalar mag = force.length();
  if (mag > max_force && mag > 1e-6f)
    force *= max_force / mag;
  pelvis->applyCentralForce(force);
}

void apply_standing(btDynamicsWorld* /*world*/,
                    const RagdollHandles& ragdoll,
                    ControllerState& state,
                    const SimulatorConfig& config) {
  if (!state.rest_captured || state.rest_orientations.size() != ragdoll.bodies.size()) {
    capture_rest_pose_impl(ragdoll, state);
  }
  set_joint_spring_stiffness_for_mode(ragdoll, false, config.walk_joint_spring_stiffness);
  if (!ragdoll.bodies.empty() && state.jump_frames_hold <= 0) {
    btRigidBody* pelvis = ragdoll.bodies[static_cast<int>(BodyPart::Pelvis)];
    apply_pelvis_position_hold(pelvis, state.rest_pelvis_position,
                               config.pose_linear_stiffness, config.pose_linear_damping);
  }
}

void apply_walking(btDynamicsWorld* /*world*/,
                   const RagdollHandles& ragdoll,
                   ControllerState& state,
                   const SimulatorConfig& config) {
  if (!state.rest_captured || state.rest_orientations.size() != ragdoll.bodies.size()) {
    capture_rest_pose_impl(ragdoll, state);
  }
  set_joint_spring_stiffness_for_mode(ragdoll, true, config.walk_joint_spring_stiffness);
  if (!ragdoll.bodies.empty() && state.jump_frames_hold <= 0) {
    btRigidBody* pelvis = ragdoll.bodies[static_cast<int>(BodyPart::Pelvis)];
    apply_pelvis_position_hold(pelvis, state.rest_pelvis_position,
                               config.pose_linear_stiffness, config.pose_linear_damping);
  }

  float phase = state.walk_phase;
  float hip_l = static_cast<float>(config.walk_hip_amplitude * std::sin(phase));
  float hip_r = static_cast<float>(config.walk_hip_amplitude * std::sin(phase + 3.14159265f));
  float knee_l = static_cast<float>(config.walk_knee_amplitude * (std::sin(phase) > 0 ? std::sin(phase) : 0.f));
  float knee_r = static_cast<float>(config.walk_knee_amplitude * (std::sin(phase + 3.14159265f) > 0 ? std::sin(phase + 3.14159265f) : 0.f));
  float arm_l = static_cast<float>(config.walk_arm_amplitude * std::sin(phase + 3.14159265f));
  float arm_r = static_cast<float>(config.walk_arm_amplitude * std::sin(phase));

  // Hip swing: rotation around X (sagittal) so leg swings forward/back. Knee: X bend.
  auto q_target = [&](int part) -> btQuaternion {
    btQuaternion rest = state.rest_orientations[part];
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
        return rest;
      default:
        return rest;
    }
  };

  for (size_t i = 0; i < ragdoll.bodies.size(); ++i) {
    apply_pd_torque(ragdoll.bodies[i], q_target(static_cast<int>(i)),
                    config.pose_stiffness, config.pose_damping, config.walk_max_torque);
  }

  // Walk debug: log phase, angle error, torque, and limb Y so we can see tracking and foot height
  static int s_walk_calls = 0;
  if (++s_walk_calls % 60 == 0 && ragdoll.bodies.size() > static_cast<size_t>(BodyPart::RightLowerLeg)) {
    const int left_upper_leg = static_cast<int>(BodyPart::LeftUpperLeg);
    btRigidBody* body = ragdoll.bodies[left_upper_leg];
    btQuaternion q_cur = body->getWorldTransform().getRotation();
    btQuaternion q_tgt = q_target(left_upper_leg);
    btQuaternion q_err = q_tgt * q_cur.inverse();
    q_err.normalize();
    btScalar w = q_err.getW();
    w = (w > 1.f) ? 1.f : ((w < -1.f) ? -1.f : w);
    float angle_err = static_cast<float>(2. * std::acos(static_cast<double>(w)));
    btVector3 axis(q_err.x(), q_err.y(), q_err.z());
    float sin_half = std::sqrt(1.f - w * w);
    if (sin_half > 1e-5f) axis /= sin_half;
    btVector3 omega = body->getAngularVelocity();
    btVector3 torque = config.pose_stiffness * angle_err * axis - config.pose_damping * omega;
    float mag = static_cast<float>(torque.length());
    if (mag > config.walk_max_torque) mag = config.walk_max_torque;
    float pelvis_y = ragdoll.bodies[static_cast<int>(BodyPart::Pelvis)]->getWorldTransform().getOrigin().y();
    float L_foot_y = ragdoll.bodies[static_cast<int>(BodyPart::LeftLowerLeg)]->getWorldTransform().getOrigin().y();
    float R_foot_y = ragdoll.bodies[static_cast<int>(BodyPart::RightLowerLeg)]->getWorldTransform().getOrigin().y();
    log("[Walk] phase=%.2f angle_err=%.3f torque=%.0f pelvis_y=%.2f L_foot_y=%.2f R_foot_y=%.2f",
        phase, angle_err, mag, pelvis_y, L_foot_y, R_foot_y);
  }
}

void apply_jump(const RagdollHandles& ragdoll,
                ControllerState& state,
                const SimulatorConfig& config) {
  if (!state.jump_triggered)
    return;
  state.jump_triggered = false;
  if (ragdoll.bodies.empty())
    return;
  btRigidBody* pelvis = ragdoll.bodies[static_cast<int>(BodyPart::Pelvis)];
  btVector3 impulse(0, static_cast<btScalar>(config.jump_impulse_y), 0);
  pelvis->applyCentralImpulse(impulse);
  state.jump_frames_hold = 30;  // skip pelvis hold for ~0.5s so character rises, then go Ragdoll
  log("[Pose] Jump applied (rise then Ragdoll)");
}

void set_ragdoll_springs_off(const RagdollHandles& ragdoll) {
  for (size_t i = 0; i < ragdoll.joints.size(); ++i) {
    btTypedConstraint* j = ragdoll.joints[i];
    if (!j) continue;
    auto* c = static_cast<btGeneric6DofSpring2Constraint*>(j);
    for (int axis = 3; axis <= 5; ++axis)
      c->enableSpring(axis, false);
  }
}

void clamp_ragdoll_velocities_impl(RagdollHandles& ragdoll,
                                   btScalar max_linear_speed,
                                   btScalar max_angular_speed) {
  for (btRigidBody* body : ragdoll.bodies) {
    btVector3 lin = body->getLinearVelocity();
    btScalar lin_sq = lin.length2();
    if (lin_sq > max_linear_speed * max_linear_speed) {
      btScalar lin_len = std::sqrt(lin_sq);
      body->setLinearVelocity(lin * (max_linear_speed / lin_len));
    }
    btVector3 ang = body->getAngularVelocity();
    btScalar ang_sq = ang.length2();
    if (ang_sq > max_angular_speed * max_angular_speed) {
      btScalar ang_len = std::sqrt(ang_sq);
      body->setAngularVelocity(ang * (max_angular_speed / ang_len));
    }
  }
}

}  // namespace

constexpr float TWO_PI = 6.28318530718f;

void clamp_ragdoll_velocities(RagdollHandles& ragdoll,
                             btScalar max_linear_speed,
                             btScalar max_angular_speed) {
  clamp_ragdoll_velocities_impl(ragdoll, max_linear_speed, max_angular_speed);
}

void zero_ragdoll_velocities(RagdollHandles& ragdoll) {
  const btVector3 zero(0, 0, 0);
  for (btRigidBody* body : ragdoll.bodies) {
    body->setLinearVelocity(zero);
    body->setAngularVelocity(zero);
  }
}

void capture_rest_pose(const RagdollHandles& ragdoll, ControllerState& state) {
  capture_rest_pose_impl(ragdoll, state);
}

void apply_pose_control(btDynamicsWorld* world,
                        const RagdollHandles& ragdoll,
                        ControllerState& state,
                        const SimulatorConfig& config) {
  apply_jump(ragdoll, state, config);

  switch (state.mode) {
    case MotionMode::Standing:
      apply_standing(world, ragdoll, state, config);
      break;
    case MotionMode::Walking:
      apply_walking(world, ragdoll, state, config);
      state.walk_phase += config.walk_speed * config.time_step * TWO_PI;
      if (state.walk_phase >= TWO_PI)
        state.walk_phase -= TWO_PI;
      if (state.walk_phase < 0)
        state.walk_phase += TWO_PI;
      break;
    case MotionMode::Ragdoll:
      set_ragdoll_springs_off(ragdoll);  // no springs so character can fall
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
