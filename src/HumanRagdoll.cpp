#include "biomechanics/HumanRagdoll.hpp"
#include "biomechanics/JoltLayers.hpp"
#include "biomechanics/Log.hpp"
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Constraints/MotorSettings.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Math/Math.h>
#ifdef JPH_OBJECT_STREAM
#include <Jolt/ObjectStream/ObjectStreamIn.h>
#include <Jolt/Skeleton/SkeletalAnimation.h>
#include <filesystem>
#include <fstream>
#endif

namespace biomechanics {

using namespace JPH;

namespace {
const float MOTOR_FREQUENCY = 4.0f;
const float MOTOR_DAMPING = 1.0f;
const float MOTOR_TORQUE_LIMIT = 500.0f;

/** Match Jolt RagdollLoader: only motion type, layer, and init. Do NOT overwrite motor settings. */
void apply_loader_style(RagdollSettings* settings) {
  if (!settings) return;
  for (RagdollSettings::Part& part : settings->mParts) {
    part.mMotionType = EMotionType::Dynamic;
    part.mObjectLayer = Layers::MOVING;
  }
  if (settings->GetSkeleton())
    settings->GetSkeleton()->CalculateParentJointIndices();
  settings->Stabilize();
  settings->DisableParentChildCollisions();
  settings->CalculateBodyIndexToConstraintIndex();
}

void apply_motor_settings_and_layer(RagdollSettings* settings) {
  if (!settings) return;
  for (RagdollSettings::Part& part : settings->mParts) {
    part.mMotionType = EMotionType::Dynamic;
    part.mObjectLayer = Layers::MOVING;
    SwingTwistConstraintSettings* st = DynamicCast<SwingTwistConstraintSettings>(part.mToParent);
    if (st) {
      st->mSwingMotorSettings = MotorSettings(MOTOR_FREQUENCY, MOTOR_DAMPING, 0.0f, MOTOR_TORQUE_LIMIT);
      st->mTwistMotorSettings = MotorSettings(MOTOR_FREQUENCY, MOTOR_DAMPING, 0.0f, MOTOR_TORQUE_LIMIT);
    }
  }
  if (settings->GetSkeleton())
    settings->GetSkeleton()->CalculateParentJointIndices();
  settings->Stabilize();
  settings->DisableParentChildCollisions();
  settings->CalculateBodyIndexToConstraintIndex();
}
}  // namespace

#ifdef JPH_OBJECT_STREAM
JPH::RagdollSettings* load_human_ragdoll_from_file() {
  const char* base = get_assets_base_path();
  if (base && base[0] != '\0') {
    std::string path = (std::filesystem::path(base) / "Human.tof").string();
    RagdollSettings* settings = nullptr;
    if (ObjectStreamIn::sReadObject(path.c_str(), settings) && settings) {
      apply_loader_style(settings);  // match Jolt RagdollLoader: do not overwrite motor settings
      return settings;
    }
  }
  static const char* paths[] = {
    "assets/Human.tof",
    "Human.tof",
    "../../assets/Human.tof",
    "../assets/Human.tof",
  };
  for (const char* path : paths) {
    RagdollSettings* settings = nullptr;
    if (ObjectStreamIn::sReadObject(path, settings) && settings) {
      apply_loader_style(settings);  // match Jolt RagdollLoader: do not overwrite motor settings
      return settings;
    }
  }
  return nullptr;
}

void load_human_animations(Ref<SkeletalAnimation>& out_standing, Ref<SkeletalAnimation>& out_walking) {
  const char* base = get_assets_base_path();
  if (base && base[0] != '\0') {
    std::string neutral_path = (std::filesystem::path(base) / "Human" / "neutral.tof").string();
    std::string walk_path = (std::filesystem::path(base) / "Human" / "walk.tof").string();
    Ref<SkeletalAnimation> standing;
    Ref<SkeletalAnimation> walking;
    if (ObjectStreamIn::sReadObject(neutral_path.c_str(), standing) && standing)
      out_standing = standing;
    if (ObjectStreamIn::sReadObject(walk_path.c_str(), walking) && walking)
      out_walking = walking;
    if (out_standing.GetPtr() || out_walking.GetPtr())
      return;
  }
  static const char* neutral_paths[] = {
    "assets/Human/neutral.tof",
    "Human/neutral.tof",
    "../../assets/Human/neutral.tof",
    "../assets/Human/neutral.tof",
  };
  static const char* walk_paths[] = {
    "assets/Human/walk.tof",
    "Human/walk.tof",
    "../../assets/Human/walk.tof",
    "../assets/Human/walk.tof",
  };
  Ref<SkeletalAnimation> standing;
  Ref<SkeletalAnimation> walking;
  for (const char* path : neutral_paths) {
    if (ObjectStreamIn::sReadObject(path, standing) && standing) break;
  }
  for (const char* path : walk_paths) {
    if (ObjectStreamIn::sReadObject(path, walking) && walking) break;
  }
  if (standing) out_standing = standing;
  if (walking) out_walking = walking;
}
#else
JPH::RagdollSettings* load_human_ragdoll_from_file() {
  return nullptr;
}
void load_human_animations(JPH::Ref<JPH::SkeletalAnimation>& /*out_standing*/,
                          JPH::Ref<JPH::SkeletalAnimation>& /*out_walking*/) {}
#endif

JPH::RagdollSettings* create_human_ragdoll_settings() {
  Ref<Skeleton> skeleton = new Skeleton;
  uint lower_body = skeleton->AddJoint("LowerBody");
  uint mid_body = skeleton->AddJoint("MidBody", (int)lower_body);
  uint upper_body = skeleton->AddJoint("UpperBody", (int)mid_body);
  (void)skeleton->AddJoint("Head", (int)upper_body);
  uint upper_arm_l = skeleton->AddJoint("UpperArmL", (int)upper_body);
  uint upper_arm_r = skeleton->AddJoint("UpperArmR", (int)upper_body);
  (void)skeleton->AddJoint("LowerArmL", (int)upper_arm_l);
  (void)skeleton->AddJoint("LowerArmR", (int)upper_arm_r);
  uint upper_leg_l = skeleton->AddJoint("UpperLegL", (int)lower_body);
  uint upper_leg_r = skeleton->AddJoint("UpperLegR", (int)lower_body);
  (void)skeleton->AddJoint("LowerLegL", (int)upper_leg_l);
  (void)skeleton->AddJoint("LowerLegR", (int)upper_leg_r);

  Ref<Shape> shapes[] = {
    new CapsuleShape(0.15f, 0.10f),
    new CapsuleShape(0.15f, 0.10f),
    new CapsuleShape(0.15f, 0.10f),
    new CapsuleShape(0.075f, 0.10f),
    new CapsuleShape(0.15f, 0.06f),
    new CapsuleShape(0.15f, 0.06f),
    new CapsuleShape(0.15f, 0.05f),
    new CapsuleShape(0.15f, 0.05f),
    new CapsuleShape(0.2f, 0.075f),
    new CapsuleShape(0.2f, 0.075f),
    new CapsuleShape(0.2f, 0.06f),
    new CapsuleShape(0.2f, 0.06f),
  };

  RVec3 positions[] = {
    RVec3(0, 1.15f, 0),
    RVec3(0, 1.35f, 0),
    RVec3(0, 1.55f, 0),
    RVec3(0, 1.825f, 0),
    RVec3(-0.425f, 1.55f, 0),
    RVec3(0.425f, 1.55f, 0),
    RVec3(-0.8f, 1.55f, 0),
    RVec3(0.8f, 1.55f, 0),
    RVec3(-0.15f, 0.8f, 0),
    RVec3(0.15f, 0.8f, 0),
    RVec3(-0.15f, 0.3f, 0),
    RVec3(0.15f, 0.3f, 0),
  };

  Quat rotations[] = {
    Quat::sRotation(Vec3::sAxisZ(), 0.5f * JPH_PI),
    Quat::sRotation(Vec3::sAxisZ(), 0.5f * JPH_PI),
    Quat::sRotation(Vec3::sAxisZ(), 0.5f * JPH_PI),
    Quat::sIdentity(),
    Quat::sRotation(Vec3::sAxisZ(), 0.5f * JPH_PI),
    Quat::sRotation(Vec3::sAxisZ(), 0.5f * JPH_PI),
    Quat::sRotation(Vec3::sAxisZ(), 0.5f * JPH_PI),
    Quat::sRotation(Vec3::sAxisZ(), 0.5f * JPH_PI),
    Quat::sIdentity(),
    Quat::sIdentity(),
    Quat::sIdentity(),
    Quat::sIdentity(),
  };

  RVec3 constraint_positions[] = {
    RVec3::sZero(),
    RVec3(0, 1.25f, 0),
    RVec3(0, 1.45f, 0),
    RVec3(0, 1.65f, 0),
    RVec3(-0.225f, 1.55f, 0),
    RVec3(0.225f, 1.55f, 0),
    RVec3(-0.65f, 1.55f, 0),
    RVec3(0.65f, 1.55f, 0),
    RVec3(-0.15f, 1.05f, 0),
    RVec3(0.15f, 1.05f, 0),
    RVec3(-0.15f, 0.55f, 0),
    RVec3(0.15f, 0.55f, 0),
  };

  Vec3 twist_axis[] = {
    Vec3::sZero(),
    Vec3::sAxisY(),
    Vec3::sAxisY(),
    Vec3::sAxisY(),
    -Vec3::sAxisX(),
    Vec3::sAxisX(),
    -Vec3::sAxisX(),
    Vec3::sAxisX(),
    -Vec3::sAxisY(),
    -Vec3::sAxisY(),
    -Vec3::sAxisY(),
    -Vec3::sAxisY(),
  };

  float twist_angle[] = {
    0.0f, 5.0f, 5.0f, 90.0f, 45.0f, 45.0f, 45.0f, 45.0f,
    45.0f, 45.0f, 45.0f, 45.0f,
  };
  float normal_angle[] = {
    0.0f, 10.0f, 10.0f, 45.0f, 90.0f, 90.0f, 0.0f, 0.0f,
    45.0f, 45.0f, 0.0f, 0.0f,
  };
  float plane_angle[] = {
    0.0f, 10.0f, 10.0f, 45.0f, 45.0f, 45.0f, 90.0f, 90.0f,
    45.0f, 45.0f, 60.0f, 60.0f,
  };

  RagdollSettings* settings = new RagdollSettings;
  settings->mSkeleton = skeleton;
  settings->mParts.resize((size_t)skeleton->GetJointCount());
  for (int p = 0; p < skeleton->GetJointCount(); ++p) {
    RagdollSettings::Part& part = settings->mParts[(size_t)p];
    part.SetShape(shapes[(size_t)p]);
    part.mPosition = positions[(size_t)p];
    part.mRotation = rotations[(size_t)p];
    part.mMotionType = EMotionType::Dynamic;
    part.mObjectLayer = Layers::MOVING;

    if (p > 0) {
      SwingTwistConstraintSettings* constraint = new SwingTwistConstraintSettings;
      constraint->mDrawConstraintSize = 0.1f;
      constraint->mPosition1 = constraint->mPosition2 = constraint_positions[(size_t)p];
      constraint->mTwistAxis1 = constraint->mTwistAxis2 = twist_axis[(size_t)p];
      constraint->mPlaneAxis1 = constraint->mPlaneAxis2 = Vec3::sAxisZ();
      constraint->mTwistMinAngle = -DegreesToRadians(twist_angle[(size_t)p]);
      constraint->mTwistMaxAngle = DegreesToRadians(twist_angle[(size_t)p]);
      constraint->mNormalHalfConeAngle = DegreesToRadians(normal_angle[(size_t)p]);
      constraint->mPlaneHalfConeAngle = DegreesToRadians(plane_angle[(size_t)p]);
      // Motors for DriveToPoseUsingMotors: stiffness (frequency Hz) and torque limit so stance works
      const float motor_frequency = 4.0f;
      const float motor_damping = 1.0f;
      const float motor_torque_limit = 500.0f;
      constraint->mSwingMotorSettings = MotorSettings(motor_frequency, motor_damping, 0.0f, motor_torque_limit);
      constraint->mTwistMotorSettings = MotorSettings(motor_frequency, motor_damping, 0.0f, motor_torque_limit);
      part.mToParent = constraint;
    }
  }

  settings->Stabilize();
  settings->DisableParentChildCollisions();
  settings->CalculateBodyIndexToConstraintIndex();

  return settings;
}

}  // namespace biomechanics
