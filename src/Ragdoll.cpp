#include "biomechanics/Ragdoll.hpp"
#include "biomechanics/JoltLayers.hpp"
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Body/Body.h>

namespace biomechanics {

namespace {

using namespace JPH;

enum JointIndex {
  JOINT_PELVIS_SPINE,
  JOINT_SPINE_HEAD,
  JOINT_LEFT_SHOULDER,
  JOINT_LEFT_ELBOW,
  JOINT_RIGHT_SHOULDER,
  JOINT_RIGHT_ELBOW,
  JOINT_LEFT_HIP,
  JOINT_LEFT_KNEE,
  JOINT_RIGHT_HIP,
  JOINT_RIGHT_KNEE,
  JOINT_COUNT
};

constexpr int BODYPART_COUNT = static_cast<int>(BodyPart::Count);

// Jolt CapsuleShape: halfHeight = half length along axis, radius = radius
BodyID create_body(BodyInterface& bi, const Shape* shape, const RVec3& pos, const Quat& rot, float linear_damping = 0.02f, float angular_damping = 0.15f) {
  BodyCreationSettings bcs(shape, pos, rot, EMotionType::Dynamic, Layers::MOVING);
  bcs.mLinearDamping = linear_damping;
  bcs.mAngularDamping = angular_damping;
  bcs.mAllowSleeping = false;
  bcs.mMaxLinearVelocity = 25.f;
  bcs.mMaxAngularVelocity = 15.f;
  Body* body = bi.CreateBody(bcs);
  if (!body)
    return BodyID();
  BodyID id = body->GetID();
  bi.AddBody(id, EActivation::Activate);
  return id;
}

void configure_six_dof(SixDOFConstraintSettings& s,
                       const Vec3& limit_min_rot, const Vec3& limit_max_rot) {
  s.MakeFixedAxis(SixDOFConstraintSettings::EAxis::TranslationX);
  s.MakeFixedAxis(SixDOFConstraintSettings::EAxis::TranslationY);
  s.MakeFixedAxis(SixDOFConstraintSettings::EAxis::TranslationZ);
  s.SetLimitedAxis(SixDOFConstraintSettings::EAxis::RotationX, limit_min_rot.GetX(), limit_max_rot.GetX());
  s.SetLimitedAxis(SixDOFConstraintSettings::EAxis::RotationY, limit_min_rot.GetY(), limit_max_rot.GetY());
  s.SetLimitedAxis(SixDOFConstraintSettings::EAxis::RotationZ, limit_min_rot.GetZ(), limit_max_rot.GetZ());
}

}  // namespace

void setup_ragdoll(PhysicsSystem* physics,
                   BodyInterface& body_interface,
                   const RVec3& position_offset,
                   float scale,
                   RagdollHandles& out) {
  if (!physics)
    return;

  BodyID bodies[BODYPART_COUNT];
  RefConst<Shape> shapes[BODYPART_COUNT];
  SixDOFConstraint* joints[JOINT_COUNT];

  // Capsule(halfHeight, radius). Bullet used (radius, fullHeight) so halfHeight = fullHeight/2.
  shapes[static_cast<int>(BodyPart::Pelvis)] = new CapsuleShape(scale * 0.10f, scale * 0.15f);
  shapes[static_cast<int>(BodyPart::Spine)] = new CapsuleShape(scale * 0.14f, scale * 0.15f);
  shapes[static_cast<int>(BodyPart::Head)] = new CapsuleShape(scale * 0.025f, scale * 0.10f);
  shapes[static_cast<int>(BodyPart::LeftUpperLeg)] = new CapsuleShape(scale * 0.225f, scale * 0.07f);
  shapes[static_cast<int>(BodyPart::LeftLowerLeg)] = new CapsuleShape(scale * 0.185f, scale * 0.05f);
  shapes[static_cast<int>(BodyPart::RightUpperLeg)] = new CapsuleShape(scale * 0.225f, scale * 0.07f);
  shapes[static_cast<int>(BodyPart::RightLowerLeg)] = new CapsuleShape(scale * 0.185f, scale * 0.05f);
  shapes[static_cast<int>(BodyPart::LeftUpperArm)] = new CapsuleShape(scale * 0.165f, scale * 0.05f);
  shapes[static_cast<int>(BodyPart::LeftLowerArm)] = new CapsuleShape(scale * 0.125f, scale * 0.04f);
  shapes[static_cast<int>(BodyPart::RightUpperArm)] = new CapsuleShape(scale * 0.165f, scale * 0.05f);
  shapes[static_cast<int>(BodyPart::RightLowerArm)] = new CapsuleShape(scale * 0.125f, scale * 0.04f);

  Vec3 offset_v(position_offset);
  bodies[static_cast<int>(BodyPart::Pelvis)] = create_body(body_interface, shapes[static_cast<int>(BodyPart::Pelvis)], position_offset + Vec3(0.f, scale * 1.f, 0.f), Quat::sIdentity());
  bodies[static_cast<int>(BodyPart::Spine)] = create_body(body_interface, shapes[static_cast<int>(BodyPart::Spine)], position_offset + Vec3(0.f, scale * 1.2f, 0.f), Quat::sIdentity());
  bodies[static_cast<int>(BodyPart::Head)] = create_body(body_interface, shapes[static_cast<int>(BodyPart::Head)], position_offset + Vec3(0.f, scale * 1.6f, 0.f), Quat::sIdentity());
  bodies[static_cast<int>(BodyPart::LeftUpperLeg)] = create_body(body_interface, shapes[static_cast<int>(BodyPart::LeftUpperLeg)], position_offset + Vec3(-0.18f * scale, 0.65f * scale, 0.f), Quat::sIdentity());
  bodies[static_cast<int>(BodyPart::LeftLowerLeg)] = create_body(body_interface, shapes[static_cast<int>(BodyPart::LeftLowerLeg)], position_offset + Vec3(-0.18f * scale, 0.2f * scale, 0.f), Quat::sIdentity());
  bodies[static_cast<int>(BodyPart::RightUpperLeg)] = create_body(body_interface, shapes[static_cast<int>(BodyPart::RightUpperLeg)], position_offset + Vec3(0.18f * scale, 0.65f * scale, 0.f), Quat::sIdentity());
  bodies[static_cast<int>(BodyPart::RightLowerLeg)] = create_body(body_interface, shapes[static_cast<int>(BodyPart::RightLowerLeg)], position_offset + Vec3(0.18f * scale, 0.2f * scale, 0.f), Quat::sIdentity());
  bodies[static_cast<int>(BodyPart::LeftUpperArm)] = create_body(body_interface, shapes[static_cast<int>(BodyPart::LeftUpperArm)], position_offset + Vec3(-0.2f * scale, (1.35f - 0.165f) * scale, 0.f), Quat::sIdentity());
  bodies[static_cast<int>(BodyPart::LeftLowerArm)] = create_body(body_interface, shapes[static_cast<int>(BodyPart::LeftLowerArm)], position_offset + Vec3(-0.2f * scale, (1.185f - 0.165f - 0.125f) * scale, 0.f), Quat::sIdentity());
  bodies[static_cast<int>(BodyPart::RightUpperArm)] = create_body(body_interface, shapes[static_cast<int>(BodyPart::RightUpperArm)], position_offset + Vec3(0.2f * scale, (1.35f - 0.165f) * scale, 0.f), Quat::sIdentity());
  bodies[static_cast<int>(BodyPart::RightLowerArm)] = create_body(body_interface, shapes[static_cast<int>(BodyPart::RightLowerArm)], position_offset + Vec3(0.2f * scale, (1.185f - 0.165f - 0.125f) * scale, 0.f), Quat::sIdentity());

  const BodyLockInterface& lock_iface = physics->GetBodyLockInterface();
  SixDOFConstraintSettings s;
  s.mSpace = EConstraintSpace::LocalToBodyCOM;

  auto add_constraint = [&](int i1, int i2, const SixDOFConstraintSettings& settings) -> SixDOFConstraint* {
    BodyLockRead lock1(lock_iface, bodies[i1]);
    BodyLockRead lock2(lock_iface, bodies[i2]);
    if (!lock1.Succeeded() || !lock2.Succeeded()) return nullptr;
    const Body* b1 = &lock1.GetBody();
    const Body* b2 = &lock2.GetBody();
    return new SixDOFConstraint(const_cast<Body&>(*b1), const_cast<Body&>(*b2), settings);
  };

  // Pelvis-Spine: spine rotated 90° around Y (capsule along X in local). Joint at top of pelvis / bottom of spine.
  s.mPosition1 = Vec3(0.f, 0.15f * scale, 0.f);
  s.mPosition2 = Vec3(0.f, -0.15f * scale, 0.f);
  s.mAxisX1 = Vec3(0, 0, -1);
  s.mAxisY1 = Vec3(0, 1, 0);
  s.mAxisX2 = Vec3(0, 0, -1);
  s.mAxisY2 = Vec3(0, 1, 0);
  configure_six_dof(s, Vec3(-JPH_PI * 0.2f, -1e-6f, -JPH_PI * 0.3f), Vec3(JPH_PI * 0.2f, 1e-6f, JPH_PI * 0.6f));
  joints[JOINT_PELVIS_SPINE] = add_constraint(static_cast<int>(BodyPart::Pelvis), static_cast<int>(BodyPart::Spine), s);
  if (joints[JOINT_PELVIS_SPINE]) physics->AddConstraint(joints[JOINT_PELVIS_SPINE]);

  // Spine-Head
  s.mPosition1 = Vec3(0.f, 0.30f * scale, 0.f);
  s.mPosition2 = Vec3(0.f, -0.14f * scale, 0.f);
  s.mAxisX1 = s.mAxisX2 = Vec3(1, 0, 0);
  s.mAxisY1 = s.mAxisY2 = Vec3(0, 1, 0);
  configure_six_dof(s, Vec3(-JPH_PI * 0.3f, -1e-6f, -JPH_PI * 0.3f), Vec3(JPH_PI * 0.5f, 1e-6f, JPH_PI * 0.3f));
  joints[JOINT_SPINE_HEAD] = add_constraint(static_cast<int>(BodyPart::Spine), static_cast<int>(BodyPart::Head), s);
  if (joints[JOINT_SPINE_HEAD]) physics->AddConstraint(joints[JOINT_SPINE_HEAD]);

  // Left hip
  s.mPosition1 = Vec3(-0.18f * scale, -0.10f * scale, 0.f);
  s.mPosition2 = Vec3(0.f, 0.225f * scale, 0.f);
  s.mAxisX1 = s.mAxisX2 = Vec3(1, 0, 0);
  s.mAxisY1 = s.mAxisY2 = Vec3(0, 1, 0);
  configure_six_dof(s, Vec3(-JPH_PI * 0.5f * 0.5f, -1e-6f, -1e-6f), Vec3(JPH_PI * 0.5f * 0.8f, 1e-6f, JPH_PI * 0.5f * 0.6f));
  joints[JOINT_LEFT_HIP] = add_constraint(static_cast<int>(BodyPart::Pelvis), static_cast<int>(BodyPart::LeftUpperLeg), s);
  if (joints[JOINT_LEFT_HIP]) physics->AddConstraint(joints[JOINT_LEFT_HIP]);

  // Left knee
  s.mPosition1 = Vec3(0.f, -0.225f * scale, 0.f);
  s.mPosition2 = Vec3(0.f, 0.185f * scale, 0.f);
  configure_six_dof(s, Vec3(-1e-6f, -1e-6f, -1e-6f), Vec3(JPH_PI * 0.7f, 1e-6f, 1e-6f));
  joints[JOINT_LEFT_KNEE] = add_constraint(static_cast<int>(BodyPart::LeftUpperLeg), static_cast<int>(BodyPart::LeftLowerLeg), s);
  if (joints[JOINT_LEFT_KNEE]) physics->AddConstraint(joints[JOINT_LEFT_KNEE]);

  // Right hip
  s.mPosition1 = Vec3(0.18f * scale, -0.10f * scale, 0.f);
  s.mPosition2 = Vec3(0.f, 0.225f * scale, 0.f);
  configure_six_dof(s, Vec3(-JPH_PI * 0.5f * 0.5f, -1e-6f, -JPH_PI * 0.5f * 0.6f), Vec3(JPH_PI * 0.5f * 0.8f, 1e-6f, 1e-6f));
  joints[JOINT_RIGHT_HIP] = add_constraint(static_cast<int>(BodyPart::Pelvis), static_cast<int>(BodyPart::RightUpperLeg), s);
  if (joints[JOINT_RIGHT_HIP]) physics->AddConstraint(joints[JOINT_RIGHT_HIP]);

  // Right knee
  s.mPosition1 = Vec3(0.f, -0.225f * scale, 0.f);
  s.mPosition2 = Vec3(0.f, 0.185f * scale, 0.f);
  configure_six_dof(s, Vec3(-1e-6f, -1e-6f, -1e-6f), Vec3(JPH_PI * 0.7f, 1e-6f, 1e-6f));
  joints[JOINT_RIGHT_KNEE] = add_constraint(static_cast<int>(BodyPart::RightUpperLeg), static_cast<int>(BodyPart::RightLowerLeg), s);
  if (joints[JOINT_RIGHT_KNEE]) physics->AddConstraint(joints[JOINT_RIGHT_KNEE]);

  // Left shoulder
  s.mPosition1 = Vec3(-0.2f * scale, 0.15f * scale, 0.f);
  s.mPosition2 = Vec3(0.f, 0.165f * scale, 0.f);
  configure_six_dof(s, Vec3(-JPH_PI * 0.8f, -1e-6f, -JPH_PI * 0.5f), Vec3(JPH_PI * 0.8f, 1e-6f, JPH_PI * 0.5f));
  joints[JOINT_LEFT_SHOULDER] = add_constraint(static_cast<int>(BodyPart::Spine), static_cast<int>(BodyPart::LeftUpperArm), s);
  if (joints[JOINT_LEFT_SHOULDER]) physics->AddConstraint(joints[JOINT_LEFT_SHOULDER]);

  // Left elbow
  s.mPosition1 = Vec3(0.f, -0.165f * scale, 0.f);
  s.mPosition2 = Vec3(0.f, 0.125f * scale, 0.f);
  configure_six_dof(s, Vec3(-1e-6f, -1e-6f, -1e-6f), Vec3(JPH_PI * 0.7f, 1e-6f, 1e-6f));
  joints[JOINT_LEFT_ELBOW] = add_constraint(static_cast<int>(BodyPart::LeftUpperArm), static_cast<int>(BodyPart::LeftLowerArm), s);
  if (joints[JOINT_LEFT_ELBOW]) physics->AddConstraint(joints[JOINT_LEFT_ELBOW]);

  // Right shoulder
  s.mPosition1 = Vec3(0.2f * scale, 0.15f * scale, 0.f);
  s.mPosition2 = Vec3(0.f, 0.165f * scale, 0.f);
  configure_six_dof(s, Vec3(-JPH_PI * 0.8f, -1e-6f, -JPH_PI * 0.5f), Vec3(JPH_PI * 0.8f, 1e-6f, JPH_PI * 0.5f));
  joints[JOINT_RIGHT_SHOULDER] = add_constraint(static_cast<int>(BodyPart::Spine), static_cast<int>(BodyPart::RightUpperArm), s);
  if (joints[JOINT_RIGHT_SHOULDER]) physics->AddConstraint(joints[JOINT_RIGHT_SHOULDER]);

  // Right elbow
  s.mPosition1 = Vec3(0.f, -0.165f * scale, 0.f);
  s.mPosition2 = Vec3(0.f, 0.125f * scale, 0.f);
  configure_six_dof(s, Vec3(-1e-6f, -1e-6f, -1e-6f), Vec3(JPH_PI * 0.7f, 1e-6f, 1e-6f));
  joints[JOINT_RIGHT_ELBOW] = add_constraint(static_cast<int>(BodyPart::RightUpperArm), static_cast<int>(BodyPart::RightLowerArm), s);
  if (joints[JOINT_RIGHT_ELBOW]) physics->AddConstraint(joints[JOINT_RIGHT_ELBOW]);

  for (int i = 0; i < BODYPART_COUNT; ++i) {
    out.bodies.push_back(bodies[i]);
    out.shapes.push_back(shapes[i]);
  }
  for (int i = 0; i < JOINT_COUNT; ++i)
    out.constraints.push_back(joints[i]);
}

void reset_ragdoll_pose(JPH::BodyInterface& body_interface,
                       RagdollHandles& handles,
                       const JPH::RVec3& position_offset,
                       float scale) {
  if (handles.bodies.size() != BODYPART_COUNT)
    return;
  using namespace JPH;
  const RVec3 pos[BODYPART_COUNT] = {
    position_offset + Vec3(0.f, scale * 1.f, 0.f),
    position_offset + Vec3(0.f, scale * 1.2f, 0.f),
    position_offset + Vec3(0.f, scale * 1.6f, 0.f),
    position_offset + Vec3(-0.18f * scale, 0.65f * scale, 0.f),
    position_offset + Vec3(-0.18f * scale, 0.2f * scale, 0.f),
    position_offset + Vec3(0.18f * scale, 0.65f * scale, 0.f),
    position_offset + Vec3(0.18f * scale, 0.2f * scale, 0.f),
    position_offset + Vec3(-0.2f * scale, (1.35f - 0.165f) * scale, 0.f),
    position_offset + Vec3(-0.2f * scale, (1.185f - 0.165f - 0.125f) * scale, 0.f),
    position_offset + Vec3(0.2f * scale, (1.35f - 0.165f) * scale, 0.f),
    position_offset + Vec3(0.2f * scale, (1.185f - 0.165f - 0.125f) * scale, 0.f),
  };
  const Quat identity = Quat::sIdentity();
  const Vec3 zero = Vec3::sZero();
  for (int i = 0; i < BODYPART_COUNT; ++i) {
    BodyID id = handles.bodies[static_cast<size_t>(i)];
    if (!id.IsInvalid())
      body_interface.SetPositionRotationAndVelocity(id, pos[i], identity, zero, zero);
  }
}

void destroy_ragdoll(PhysicsSystem* physics,
                     BodyInterface& body_interface,
                     RagdollHandles& handles) {
  // Remove constraints first (reverse order = always pop last, one ref released). Then bodies (constraints ref bodies).
  if (physics) {
    JPH::Constraints list = physics->GetConstraints();
    for (int i = static_cast<int>(list.size()) - 1; i >= 0; --i)
      physics->RemoveConstraint(list[i]);
  }
  handles.constraints.clear();
  for (BodyID id : handles.bodies) {
    if (!id.IsInvalid()) {
      body_interface.RemoveBody(id);
      body_interface.DestroyBody(id);
    }
  }
  handles.bodies.clear();
  handles.shapes.clear();
}

}  // namespace biomechanics
