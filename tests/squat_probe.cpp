/**
 * Headless probe: find the joint rotation axes for a squat on the Human.tof rig.
 * Dumps the skeleton, then tries all hip/knee axis combinations (composed on the
 * neutral standing pose) and reports which keeps the feet closest to their
 * standing world position while the pelvis drops. Pure kinematics, no physics.
 * Run from repo root so assets/ resolves.
 */
#include "biomechanics/Config.hpp"
#include "biomechanics/SimulatorScene.hpp"
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Skeleton/SkeletonPose.h>
#include <Jolt/Skeleton/SkeletalAnimation.h>
#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

using namespace JPH;
namespace biomech = biomechanics;

namespace {

struct AxisOption {
  const char* name;
  Vec3 axis;
};

const AxisOption kAxes[] = {
    {"+X", Vec3::sAxisX()},  {"-X", -Vec3::sAxisX()},
    {"+Y", Vec3::sAxisY()},  {"-Y", -Vec3::sAxisY()},
    {"+Z", Vec3::sAxisZ()},  {"-Z", -Vec3::sAxisZ()},
};

void sample_base(const SkeletalAnimation* anim, const Skeleton* skel, float root_y,
                 SkeletonPose& pose) {
  pose.SetSkeleton(skel);
  anim->Sample(0.0f, pose);
  pose.GetJoint(0).mTranslation = Vec3::sZero();
  pose.SetRootOffset(RVec3(0, root_y, 0));
  pose.CalculateJointMatrices();
}

Vec3 joint_world(const SkeletonPose& pose, int idx) {
  return Vec3(pose.GetJointMatrix(idx).GetTranslation()) + Vec3(pose.GetRootOffset());
}

void compose(SkeletonPose& pose, int idx, Vec3Arg axis, float angle) {
  if (idx < 0) return;
  auto& j = pose.GetJoint(static_cast<uint>(idx));
  j.mRotation = Quat::sRotation(axis, angle) * j.mRotation;
}

}  // namespace

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  biomech::ensure_jolt_registered();

  biomech::SimulatorConfig config;
  biomech::SimulatorScene scene;
  biomech::create_simulator_scene(config, scene);
  if (!scene.ragdoll_settings || !scene.standing_anim.GetPtr()) {
    std::printf("PROBE FAIL: no ragdoll settings or standing anim (run from repo root)\n");
    return 1;
  }
  const Skeleton* skel = scene.ragdoll_settings->GetSkeleton();
  const SkeletalAnimation* anim = scene.standing_anim.GetPtr();

  // 1) Dump skeleton with standing world positions.
  SkeletonPose base;
  sample_base(anim, skel, config.standing_min_height, base);
  std::printf("=== Skeleton (%d joints), standing world positions ===\n", skel->GetJointCount());
  for (int i = 0; i < skel->GetJointCount(); ++i) {
    Vec3 w = joint_world(base, i);
    std::printf("  [%2d] %-18s parent=%2d  world=(%6.3f, %6.3f, %6.3f)\n", i,
                skel->GetJoint(i).mName.c_str(), skel->GetJoint(i).mParentJointIndex,
                w.GetX(), w.GetY(), w.GetZ());
  }

  const int r_thigh = skel->GetJointIndex("R_Leg_sjnt_0");
  const int r_shin = skel->GetJointIndex("R_Leg_sjnt_1");
  const int l_thigh = skel->GetJointIndex("L_Leg_sjnt_0");
  const int l_shin = skel->GetJointIndex("L_Leg_sjnt_1");
  const int r_foot = skel->GetJointIndex("R_Foot_sjnt_0");
  const int l_foot = skel->GetJointIndex("L_Foot_sjnt_0");
  if (r_thigh < 0 || r_shin < 0 || r_foot < 0) {
    std::printf("PROBE FAIL: leg joints not found\n");
    return 1;
  }

  const Vec3 rf0 = joint_world(base, r_foot);
  const Vec3 lf0 = joint_world(base, l_foot);
  const Vec3 rk0 = joint_world(base, r_shin);
  std::printf("Standing: R_foot=(%.3f, %.3f, %.3f) R_knee=(%.3f, %.3f, %.3f)\n",
              rf0.GetX(), rf0.GetY(), rf0.GetZ(), rk0.GetX(), rk0.GetY(), rk0.GetZ());

  // 2) Try all hip/knee axis combinations at the configured squat angles.
  struct Result {
    std::string desc;
    float foot_err;
    Vec3 rf_disp, knee_disp;
  };
  std::vector<Result> results;
  const float hip_angle = config.jump_crouch_hip;
  const float knee_angle = config.jump_crouch_knee;
  const float root_y = config.standing_min_height - config.jump_crouch_drop;

  for (const AxisOption& ha : kAxes) {
    for (const AxisOption& ka : kAxes) {
      SkeletonPose p;
      sample_base(anim, skel, root_y, p);
      compose(p, r_thigh, ha.axis, hip_angle);
      compose(p, l_thigh, ha.axis, hip_angle);
      compose(p, r_shin, ka.axis, knee_angle);
      compose(p, l_shin, ka.axis, knee_angle);
      p.CalculateJointMatrices();
      const Vec3 rf = joint_world(p, r_foot) - rf0;
      const Vec3 lf = joint_world(p, l_foot) - lf0;
      const Vec3 kn = joint_world(p, r_shin) - rk0;
      const float err = 0.5f * (rf.Length() + lf.Length());
      Result r;
      r.desc = std::string("hip ") + ha.name + " knee " + ka.name;
      r.foot_err = err;
      r.rf_disp = rf;
      r.knee_disp = kn;
      results.push_back(r);
    }
  }
  std::sort(results.begin(), results.end(),
            [](const Result& a, const Result& b) { return a.foot_err < b.foot_err; });
  std::printf("\n=== Best axis combos (hip=%.2f rad, knee=%.2f rad, drop=%.2f m) ===\n",
              hip_angle, knee_angle, config.jump_crouch_drop);
  std::printf("%-18s %-9s %-32s %s\n", "combo", "foot_err", "R_foot disp (x,y,z)", "R_knee disp (x,y,z)");
  for (size_t i = 0; i < results.size() && i < 10; ++i) {
    const Result& r = results[i];
    std::printf("%-18s %8.3f  (%6.3f, %6.3f, %6.3f)   (%6.3f, %6.3f, %6.3f)\n", r.desc.c_str(),
                r.foot_err, r.rf_disp.GetX(), r.rf_disp.GetY(), r.rf_disp.GetZ(),
                r.knee_disp.GetX(), r.knee_disp.GetY(), r.knee_disp.GetZ());
  }

  // 3) Fine grid search with the winning axes (hip -X, knee +X): for each fixed
  // knee depth, find hip/drop minimizing foot displacement, plus the ankle angle
  // that keeps the foot orientation unchanged (flat on ground).
  {
    const Quat rf_rot0 = base.GetJointMatrix(r_foot).GetQuaternion();
    std::printf("\n=== Fine search per knee depth (hip -X, knee +X, ankle +X) ===\n");
    std::printf("%-6s %-6s %-6s %-9s %-8s %s\n", "knee", "hip", "drop", "foot_err", "ankle", "R_foot disp (x,y,z)");
    for (float knee = 1.1f; knee <= 1.75f; knee += 0.1f) {
      float best_err = 1e9f;
      float best_hip = 0, best_drop = 0;
      for (float hip = 0.3f; hip <= 1.4f; hip += 0.02f) {
        for (float drop = 0.10f; drop <= 0.45f; drop += 0.01f) {
          SkeletonPose p;
          sample_base(anim, skel, config.standing_min_height - drop, p);
          compose(p, r_thigh, -Vec3::sAxisX(), hip);
          compose(p, l_thigh, -Vec3::sAxisX(), hip);
          compose(p, r_shin, Vec3::sAxisX(), knee);
          compose(p, l_shin, Vec3::sAxisX(), knee);
          p.CalculateJointMatrices();
          const float err = 0.5f * ((joint_world(p, r_foot) - rf0).Length() +
                                    (joint_world(p, l_foot) - lf0).Length());
          if (err < best_err) {
            best_err = err;
            best_hip = hip;
            best_drop = drop;
          }
        }
      }
      // Ankle compensation: find angle minimizing foot world-orientation change.
      float best_ankle = 0.f, best_rot_err = 1e9f;
      Vec3 fd;
      for (float ankle = -1.2f; ankle <= 0.4f; ankle += 0.02f) {
        SkeletonPose p;
        sample_base(anim, skel, config.standing_min_height - best_drop, p);
        compose(p, r_thigh, -Vec3::sAxisX(), best_hip);
        compose(p, l_thigh, -Vec3::sAxisX(), best_hip);
        compose(p, r_shin, Vec3::sAxisX(), knee);
        compose(p, l_shin, Vec3::sAxisX(), knee);
        compose(p, r_foot, Vec3::sAxisX(), ankle);
        compose(p, l_foot, Vec3::sAxisX(), ankle);
        p.CalculateJointMatrices();
        const Quat rot = p.GetJointMatrix(r_foot).GetQuaternion();
        const float rot_err = 2.f * std::acos(std::min(1.f, std::abs(rot.Dot(rf_rot0))));
        if (rot_err < best_rot_err) {
          best_rot_err = rot_err;
          best_ankle = ankle;
          fd = joint_world(p, r_foot) - rf0;
        }
      }
      std::printf("%-6.2f %-6.2f %-6.2f %8.3f  %6.2f   (%6.3f, %6.3f, %6.3f)\n", knee, best_hip,
                  best_drop, best_err, best_ankle, fd.GetX(), fd.GetY(), fd.GetZ());
    }
  }

  // 4) Arm axis probe: rotate shoulders by +0.9 and report wrist displacement.
  // Character faces -Z, so "arms back" = wrist +Z & down stays low, "arms up/forward" = -Z & up.
  const int l_arm = skel->GetJointIndex("L_Arm_sjnt_0");
  const int r_arm = skel->GetJointIndex("R_Arm_sjnt_0");
  const int l_wrist = skel->GetJointIndex("L_Wrist_sjnt_0");
  const int r_wrist = skel->GetJointIndex("R_Wrist_sjnt_0");
  if (l_arm >= 0 && r_wrist >= 0) {
    const Vec3 rw0 = joint_world(base, r_wrist);
    const Vec3 lw0 = joint_world(base, l_wrist);
    std::printf("\n=== Arm probe (shoulder +0.9 rad, wrist displacement) ===\n");
    std::printf("standing R_wrist=(%.3f, %.3f, %.3f)\n", rw0.GetX(), rw0.GetY(), rw0.GetZ());
    for (const AxisOption& a : kAxes) {
      SkeletonPose p;
      sample_base(anim, skel, config.standing_min_height, p);
      compose(p, l_arm, a.axis, 0.9f);
      compose(p, r_arm, a.axis, 0.9f);
      p.CalculateJointMatrices();
      const Vec3 rd = joint_world(p, r_wrist) - rw0;
      const Vec3 ld = joint_world(p, l_wrist) - lw0;
      std::printf("  axis %s: R_wrist disp=(%6.3f, %6.3f, %6.3f)  L_wrist disp=(%6.3f, %6.3f, %6.3f)\n",
                  a.name, rd.GetX(), rd.GetY(), rd.GetZ(), ld.GetX(), ld.GetY(), ld.GetZ());
    }
  }

  // 5) Torso lean probe: rotate spine joints by +0.3, report head displacement (forward = -Z).
  const int spine0 = skel->GetJointIndex("C_Spine_sjnt_0");
  const int head = skel->GetJointIndex("C_Head_sjnt_0");
  if (spine0 >= 0 && head >= 0) {
    const Vec3 h0 = joint_world(base, head);
    std::printf("\n=== Torso probe (spine +0.3 rad, head displacement; forward = -Z) ===\n");
    for (const AxisOption& a : kAxes) {
      SkeletonPose p;
      sample_base(anim, skel, config.standing_min_height, p);
      compose(p, spine0, a.axis, 0.3f);
      p.CalculateJointMatrices();
      const Vec3 hd = joint_world(p, head) - h0;
      std::printf("  axis %s: head disp=(%6.3f, %6.3f, %6.3f)\n", a.name, hd.GetX(), hd.GetY(),
                  hd.GetZ());
    }
  }

  // Skip destroy_simulator_scene: teardown has a pre-existing heap issue; probe is read-only.
  return 0;
}
