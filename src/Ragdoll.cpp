#include "biomechanics/Ragdoll.hpp"
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>

namespace biomechanics {

namespace {

// Spring2 axis indices: 0,1,2 = linear; 3,4,5 = angular (see Bullet docs)
constexpr int ANGULAR_AXIS_START = 3;
constexpr int ANGULAR_AXIS_COUNT = 3;
constexpr btScalar JOINT_SPRING_STIFFNESS = 150.f;
constexpr btScalar JOINT_SPRING_DAMPING = 15.f;

void configure_spring2_joint(btGeneric6DofSpring2Constraint* c,
                              const btVector3& angular_lower,
                              const btVector3& angular_upper) {
  c->setLinearLowerLimit(btVector3(0.f, 0.f, 0.f));
  c->setLinearUpperLimit(btVector3(0.f, 0.f, 0.f));
  c->setAngularLowerLimit(angular_lower);
  c->setAngularUpperLimit(angular_upper);
  for (int i = 0; i < ANGULAR_AXIS_COUNT; ++i) {
    c->enableSpring(ANGULAR_AXIS_START + i, true);
    c->setStiffness(ANGULAR_AXIS_START + i, JOINT_SPRING_STIFFNESS, true);
    c->setDamping(ANGULAR_AXIS_START + i, JOINT_SPRING_DAMPING, true);
  }
  c->setEquilibriumPoint();
}

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

btRigidBody* create_rigid_body(btDynamicsWorld* world, btScalar mass,
                               const btTransform& start_transform,
                               btCollisionShape* shape) {
  bool is_dynamic = (mass != 0.f);
  btVector3 local_inertia(0, 0, 0);
  if (is_dynamic)
    shape->calculateLocalInertia(mass, local_inertia);

  auto* motion_state = new btDefaultMotionState(start_transform);
  btRigidBody::btRigidBodyConstructionInfo rb_info(mass, motion_state, shape, local_inertia);
  rb_info.m_additionalDamping = false;
  auto* body = new btRigidBody(rb_info);
  world->addRigidBody(body);
  return body;
}

}  // namespace

void setup_ragdoll(btDynamicsWorld* world,
                   const btVector3& position_offset,
                   btScalar scale,
                   RagdollHandles& out) {
  btCollisionShape* shapes[BODYPART_COUNT];
  btRigidBody* bodies[BODYPART_COUNT];
  btTypedConstraint* joints[JOINT_COUNT];

  shapes[static_cast<int>(BodyPart::Pelvis)] = new btCapsuleShape(scale * 0.15f, scale * 0.20f);
  shapes[static_cast<int>(BodyPart::Spine)] = new btCapsuleShape(scale * 0.15f, scale * 0.28f);
  shapes[static_cast<int>(BodyPart::Head)] = new btCapsuleShape(scale * 0.10f, scale * 0.05f);
  shapes[static_cast<int>(BodyPart::LeftUpperLeg)] = new btCapsuleShape(scale * 0.07f, scale * 0.45f);
  shapes[static_cast<int>(BodyPart::LeftLowerLeg)] = new btCapsuleShape(scale * 0.05f, scale * 0.37f);
  shapes[static_cast<int>(BodyPart::RightUpperLeg)] = new btCapsuleShape(scale * 0.07f, scale * 0.45f);
  shapes[static_cast<int>(BodyPart::RightLowerLeg)] = new btCapsuleShape(scale * 0.05f, scale * 0.37f);
  shapes[static_cast<int>(BodyPart::LeftUpperArm)] = new btCapsuleShape(scale * 0.05f, scale * 0.33f);
  shapes[static_cast<int>(BodyPart::LeftLowerArm)] = new btCapsuleShape(scale * 0.04f, scale * 0.25f);
  shapes[static_cast<int>(BodyPart::RightUpperArm)] = new btCapsuleShape(scale * 0.05f, scale * 0.33f);
  shapes[static_cast<int>(BodyPart::RightLowerArm)] = new btCapsuleShape(scale * 0.04f, scale * 0.25f);

  btTransform offset;
  offset.setIdentity();
  offset.setOrigin(position_offset);

  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(btVector3(0.f, scale * 1.f, 0.f));
  bodies[static_cast<int>(BodyPart::Pelvis)] = create_rigid_body(world, 1.f, offset * transform, shapes[static_cast<int>(BodyPart::Pelvis)]);

  transform.setOrigin(btVector3(0.f, scale * 1.2f, 0.f));
  bodies[static_cast<int>(BodyPart::Spine)] = create_rigid_body(world, 1.f, offset * transform, shapes[static_cast<int>(BodyPart::Spine)]);

  transform.setOrigin(btVector3(0.f, scale * 1.6f, 0.f));
  bodies[static_cast<int>(BodyPart::Head)] = create_rigid_body(world, 1.f, offset * transform, shapes[static_cast<int>(BodyPart::Head)]);

  transform.setOrigin(btVector3(-0.18f * scale, 0.65f * scale, 0.f));
  bodies[static_cast<int>(BodyPart::LeftUpperLeg)] = create_rigid_body(world, 1.f, offset * transform, shapes[static_cast<int>(BodyPart::LeftUpperLeg)]);

  transform.setOrigin(btVector3(-0.18f * scale, 0.2f * scale, 0.f));
  bodies[static_cast<int>(BodyPart::LeftLowerLeg)] = create_rigid_body(world, 1.f, offset * transform, shapes[static_cast<int>(BodyPart::LeftLowerLeg)]);

  transform.setOrigin(btVector3(0.18f * scale, 0.65f * scale, 0.f));
  bodies[static_cast<int>(BodyPart::RightUpperLeg)] = create_rigid_body(world, 1.f, offset * transform, shapes[static_cast<int>(BodyPart::RightUpperLeg)]);

  transform.setOrigin(btVector3(0.18f * scale, 0.2f * scale, 0.f));
  bodies[static_cast<int>(BodyPart::RightLowerLeg)] = create_rigid_body(world, 1.f, offset * transform, shapes[static_cast<int>(BodyPart::RightLowerLeg)]);

  // Arms at sides (standing pose): capsule along Y (identity). Shoulder y = 1.2+0.15 = 1.35; upper arm half 0.165, lower 0.125.
  transform.setIdentity();
  transform.setOrigin(btVector3(-0.2f * scale, (1.35f - 0.165f) * scale, 0.f));
  bodies[static_cast<int>(BodyPart::LeftUpperArm)] = create_rigid_body(world, 1.f, offset * transform, shapes[static_cast<int>(BodyPart::LeftUpperArm)]);

  transform.setIdentity();
  transform.setOrigin(btVector3(-0.2f * scale, (1.185f - 0.165f - 0.125f) * scale, 0.f));
  bodies[static_cast<int>(BodyPart::LeftLowerArm)] = create_rigid_body(world, 1.f, offset * transform, shapes[static_cast<int>(BodyPart::LeftLowerArm)]);

  transform.setIdentity();
  transform.setOrigin(btVector3(0.2f * scale, (1.35f - 0.165f) * scale, 0.f));
  bodies[static_cast<int>(BodyPart::RightUpperArm)] = create_rigid_body(world, 1.f, offset * transform, shapes[static_cast<int>(BodyPart::RightUpperArm)]);

  transform.setIdentity();
  transform.setOrigin(btVector3(0.2f * scale, (1.185f - 0.165f - 0.125f) * scale, 0.f));
  bodies[static_cast<int>(BodyPart::RightLowerArm)] = create_rigid_body(world, 1.f, offset * transform, shapes[static_cast<int>(BodyPart::RightLowerArm)]);

  for (int i = 0; i < BODYPART_COUNT; ++i) {
    bodies[i]->setDamping(0.02f, 0.15f);
    bodies[i]->setActivationState(DISABLE_DEACTIVATION);
  }

  btTransform local_a, local_b;

  local_a.setIdentity();
  local_b.setIdentity();
  local_a.getBasis().setEulerZYX(0, SIMD_HALF_PI, 0);
  local_a.setOrigin(btVector3(0.f, 0.15f * scale, 0.f));
  local_b.getBasis().setEulerZYX(0, SIMD_HALF_PI, 0);
  local_b.setOrigin(btVector3(0.f, -0.15f * scale, 0.f));
  auto* c_ps = new btGeneric6DofSpring2Constraint(
      *bodies[static_cast<int>(BodyPart::Pelvis)], *bodies[static_cast<int>(BodyPart::Spine)], local_a, local_b);
  configure_spring2_joint(c_ps,
      btVector3(-SIMD_PI * 0.2f, -SIMD_EPSILON, -SIMD_PI * 0.3f),
      btVector3(SIMD_PI * 0.2f, SIMD_EPSILON, SIMD_PI * 0.6f));
  joints[JOINT_PELVIS_SPINE] = c_ps;
  world->addConstraint(joints[JOINT_PELVIS_SPINE], true);

  local_a.setIdentity();
  local_b.setIdentity();
  local_a.setOrigin(btVector3(0.f, 0.30f * scale, 0.f));
  local_b.setOrigin(btVector3(0.f, -0.14f * scale, 0.f));
  auto* c_sh = new btGeneric6DofSpring2Constraint(
      *bodies[static_cast<int>(BodyPart::Spine)], *bodies[static_cast<int>(BodyPart::Head)], local_a, local_b);
  configure_spring2_joint(c_sh,
      btVector3(-SIMD_PI * 0.3f, -SIMD_EPSILON, -SIMD_PI * 0.3f),
      btVector3(SIMD_PI * 0.5f, SIMD_EPSILON, SIMD_PI * 0.3f));
  joints[JOINT_SPINE_HEAD] = c_sh;
  world->addConstraint(joints[JOINT_SPINE_HEAD], true);

  local_a.setIdentity();
  local_b.setIdentity();
  local_a.setOrigin(btVector3(-0.18f * scale, -0.10f * scale, 0.f));
  local_b.setOrigin(btVector3(0.f, 0.225f * scale, 0.f));
  auto* c_lh = new btGeneric6DofSpring2Constraint(
      *bodies[static_cast<int>(BodyPart::Pelvis)], *bodies[static_cast<int>(BodyPart::LeftUpperLeg)], local_a, local_b);
  configure_spring2_joint(c_lh,
      btVector3(-SIMD_HALF_PI * 0.5f, -SIMD_EPSILON, -SIMD_EPSILON),
      btVector3(SIMD_HALF_PI * 0.8f, SIMD_EPSILON, SIMD_HALF_PI * 0.6f));
  joints[JOINT_LEFT_HIP] = c_lh;
  world->addConstraint(joints[JOINT_LEFT_HIP], true);

  local_a.setIdentity();
  local_b.setIdentity();
  local_a.setOrigin(btVector3(0.f, -0.225f * scale, 0.f));
  local_b.setOrigin(btVector3(0.f, 0.185f * scale, 0.f));
  auto* c_lk = new btGeneric6DofSpring2Constraint(
      *bodies[static_cast<int>(BodyPart::LeftUpperLeg)], *bodies[static_cast<int>(BodyPart::LeftLowerLeg)], local_a, local_b);
  configure_spring2_joint(c_lk,
      btVector3(-SIMD_EPSILON, -SIMD_EPSILON, -SIMD_EPSILON),
      btVector3(SIMD_PI * 0.7f, SIMD_EPSILON, SIMD_EPSILON));
  joints[JOINT_LEFT_KNEE] = c_lk;
  world->addConstraint(joints[JOINT_LEFT_KNEE], true);

  local_a.setIdentity();
  local_b.setIdentity();
  local_a.setOrigin(btVector3(0.18f * scale, -0.10f * scale, 0.f));
  local_b.setOrigin(btVector3(0.f, 0.225f * scale, 0.f));
  auto* c_rh = new btGeneric6DofSpring2Constraint(
      *bodies[static_cast<int>(BodyPart::Pelvis)], *bodies[static_cast<int>(BodyPart::RightUpperLeg)], local_a, local_b);
  configure_spring2_joint(c_rh,
      btVector3(-SIMD_HALF_PI * 0.5f, -SIMD_EPSILON, -SIMD_HALF_PI * 0.6f),
      btVector3(SIMD_HALF_PI * 0.8f, SIMD_EPSILON, SIMD_EPSILON));
  joints[JOINT_RIGHT_HIP] = c_rh;
  world->addConstraint(joints[JOINT_RIGHT_HIP], true);

  local_a.setIdentity();
  local_b.setIdentity();
  local_a.setOrigin(btVector3(0.f, -0.225f * scale, 0.f));
  local_b.setOrigin(btVector3(0.f, 0.185f * scale, 0.f));
  auto* c_rk = new btGeneric6DofSpring2Constraint(
      *bodies[static_cast<int>(BodyPart::RightUpperLeg)], *bodies[static_cast<int>(BodyPart::RightLowerLeg)], local_a, local_b);
  configure_spring2_joint(c_rk,
      btVector3(-SIMD_EPSILON, -SIMD_EPSILON, -SIMD_EPSILON),
      btVector3(SIMD_PI * 0.7f, SIMD_EPSILON, SIMD_EPSILON));
  joints[JOINT_RIGHT_KNEE] = c_rk;
  world->addConstraint(joints[JOINT_RIGHT_KNEE], true);

  local_a.setIdentity();
  local_b.setIdentity();
  local_a.setOrigin(btVector3(-0.2f * scale, 0.15f * scale, 0.f));
  local_b.setOrigin(btVector3(0.f, 0.165f * scale, 0.f));
  auto* c_ls = new btGeneric6DofSpring2Constraint(
      *bodies[static_cast<int>(BodyPart::Spine)], *bodies[static_cast<int>(BodyPart::LeftUpperArm)], local_a, local_b);
  configure_spring2_joint(c_ls,
      btVector3(-SIMD_PI * 0.8f, -SIMD_EPSILON, -SIMD_PI * 0.5f),
      btVector3(SIMD_PI * 0.8f, SIMD_EPSILON, SIMD_PI * 0.5f));
  joints[JOINT_LEFT_SHOULDER] = c_ls;
  world->addConstraint(joints[JOINT_LEFT_SHOULDER], true);

  local_a.setIdentity();
  local_b.setIdentity();
  local_a.setOrigin(btVector3(0.f, -0.165f * scale, 0.f));
  local_b.setOrigin(btVector3(0.f, 0.125f * scale, 0.f));
  auto* c_le = new btGeneric6DofSpring2Constraint(
      *bodies[static_cast<int>(BodyPart::LeftUpperArm)], *bodies[static_cast<int>(BodyPart::LeftLowerArm)], local_a, local_b);
  configure_spring2_joint(c_le,
      btVector3(-SIMD_EPSILON, -SIMD_EPSILON, -SIMD_EPSILON),
      btVector3(SIMD_PI * 0.7f, SIMD_EPSILON, SIMD_EPSILON));
  joints[JOINT_LEFT_ELBOW] = c_le;
  world->addConstraint(joints[JOINT_LEFT_ELBOW], true);

  local_a.setIdentity();
  local_b.setIdentity();
  local_a.setOrigin(btVector3(0.2f * scale, 0.15f * scale, 0.f));
  local_b.setOrigin(btVector3(0.f, 0.165f * scale, 0.f));
  auto* c_rs = new btGeneric6DofSpring2Constraint(
      *bodies[static_cast<int>(BodyPart::Spine)], *bodies[static_cast<int>(BodyPart::RightUpperArm)], local_a, local_b);
  configure_spring2_joint(c_rs,
      btVector3(-SIMD_PI * 0.8f, -SIMD_EPSILON, -SIMD_PI * 0.5f),
      btVector3(SIMD_PI * 0.8f, SIMD_EPSILON, SIMD_PI * 0.5f));
  joints[JOINT_RIGHT_SHOULDER] = c_rs;
  world->addConstraint(joints[JOINT_RIGHT_SHOULDER], true);

  local_a.setIdentity();
  local_b.setIdentity();
  local_a.setOrigin(btVector3(0.f, -0.165f * scale, 0.f));
  local_b.setOrigin(btVector3(0.f, 0.125f * scale, 0.f));
  auto* c_re = new btGeneric6DofSpring2Constraint(
      *bodies[static_cast<int>(BodyPart::RightUpperArm)], *bodies[static_cast<int>(BodyPart::RightLowerArm)], local_a, local_b);
  configure_spring2_joint(c_re,
      btVector3(-SIMD_EPSILON, -SIMD_EPSILON, -SIMD_EPSILON),
      btVector3(SIMD_PI * 0.7f, SIMD_EPSILON, SIMD_EPSILON));
  joints[JOINT_RIGHT_ELBOW] = c_re;
  world->addConstraint(joints[JOINT_RIGHT_ELBOW], true);

  for (int i = 0; i < BODYPART_COUNT; ++i) {
    out.bodies.push_back(bodies[i]);
    out.shapes.push_back(shapes[i]);
  }
  for (int i = 0; i < JOINT_COUNT; ++i)
    out.joints.push_back(joints[i]);
}

void destroy_ragdoll(btDynamicsWorld* world, RagdollHandles& handles) {
  for (auto* j : handles.joints) {
    world->removeConstraint(j);
    delete j;
  }
  handles.joints.clear();
  for (auto* b : handles.bodies) {
    world->removeRigidBody(b);
    delete b->getMotionState();
    delete b;
  }
  handles.bodies.clear();
  for (auto* s : handles.shapes)
    delete s;
  handles.shapes.clear();
}

}  // namespace biomechanics
