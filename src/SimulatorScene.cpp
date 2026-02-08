#include "biomechanics/SimulatorScene.hpp"
#include "biomechanics/HumanRagdoll.hpp"
#include "biomechanics/Log.hpp"
#include <RegisterTypes.h>
#include <Core/Memory.h>
#include <Core/Factory.h>
#include <Physics/Collision/Shape/BoxShape.h>
#include <Physics/Body/BodyCreationSettings.h>
#include <Physics/Body/Body.h>
#include <Skeleton/SkeletonPose.h>
#include <thread>
#include <fstream>

namespace biomechanics {

void ensure_jolt_registered() {
  static bool done = false;
  if (done)
    return;
  JPH::RegisterDefaultAllocator();
  if (!JPH::Factory::sInstance)
    JPH::Factory::sInstance = new JPH::Factory();
  JPH::RegisterTypes();
  done = true;
}

void create_simulator_scene(const SimulatorConfig& config, SimulatorScene& out) {
  ensure_jolt_registered();

  constexpr JPH::uint cMaxBodies = 64;
  constexpr JPH::uint cNumBodyMutexes = 0;
  constexpr JPH::uint cMaxBodyPairs = 1024;
  constexpr JPH::uint cMaxContactConstraints = 512;

  unsigned num_threads = std::thread::hardware_concurrency();
  if (num_threads > 0)
    num_threads -= 1;  // leave one for main thread
  out.job_system = new JPH::JobSystemThreadPool(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, num_threads);
  out.temp_allocator = new JPH::TempAllocatorImpl(10 * 1024 * 1024);
  out.physics = new JPH::PhysicsSystem();
  out.physics->Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints,
                   out.bp_layer_interface, out.object_vs_bp_filter, out.object_layer_pair_filter);
  out.physics->SetGravity(JPH::Vec3(0.f, config.gravity_y, 0.f));

  JPH::BodyInterface& bi = out.physics->GetBodyInterface();

  JPH::BoxShapeSettings box_settings(JPH::Vec3(50.f, 0.5f, 50.f));
  JPH::ShapeSettings::ShapeResult box_result = box_settings.Create();
  JPH::ShapeRefC ground_shape = box_result.Get();
  JPH::BodyCreationSettings ground_bcs(ground_shape, JPH::RVec3(0.f, -0.5f, 0.f), JPH::Quat::sIdentity(), JPH::EMotionType::Static, Layers::NON_MOVING);
  JPH::Body* ground_body = bi.CreateBody(ground_bcs);
  if (ground_body) {
    out.ground_id = ground_body->GetID();
    if (!out.ground_id.IsInvalid())
      bi.AddBody(out.ground_id, JPH::EActivation::DontActivate);
  }

  // Prefer JoltPhysics Human.tof model when available (copy from JoltPhysics/Assets/Human.tof to assets/)
  out.ragdoll_settings = load_human_ragdoll_from_file();
  bool from_file = (out.ragdoll_settings != nullptr);
  if (!out.ragdoll_settings)
    out.ragdoll_settings = create_human_ragdoll_settings();
  out.ragdoll = out.ragdoll_settings->CreateRagdoll(0, 0, out.physics);
  if (from_file)
    load_human_animations(out.standing_anim, out.walking_anim);
  if (out.ragdoll) {
    out.ragdoll->AddToPhysicsSystem(JPH::EActivation::Activate);
    JPH::SkeletonPose pose;
    pose.SetSkeleton(out.ragdoll_settings->GetSkeleton());
    out.ragdoll->GetPose(pose);
    pose.SetRootOffset(pose.GetRootOffset() + JPH::RVec3(0.f, config.ragdoll_height, 0.f));
    out.ragdoll->SetPose(pose);
    out.ragdoll->ResetWarmStart();
  }
}

void destroy_simulator_scene(SimulatorScene& scene) {
  if (!scene.physics)
    return;
  JPH::BodyInterface& bi = scene.physics->GetBodyInterface();
  if (scene.ragdoll) {
    scene.ragdoll->RemoveFromPhysicsSystem();
    delete scene.ragdoll;
    scene.ragdoll = nullptr;
  }
  if (scene.ragdoll_settings) {
    delete scene.ragdoll_settings;
    scene.ragdoll_settings = nullptr;
  }
  scene.standing_anim = nullptr;
  scene.walking_anim = nullptr;
  if (!scene.ground_id.IsInvalid()) {
    bi.RemoveBody(scene.ground_id);
    bi.DestroyBody(scene.ground_id);
  }
  delete scene.physics;
  scene.physics = nullptr;
  delete scene.temp_allocator;
  scene.temp_allocator = nullptr;
  delete scene.job_system;
  scene.job_system = nullptr;
  scene.ground_id = JPH::BodyID();
}

}  // namespace biomechanics
