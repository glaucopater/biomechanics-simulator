#pragma once

#include "biomechanics/Config.hpp"
#include "biomechanics/JoltLayers.hpp"
#include <Jolt.h>
#include <Physics/Body/BodyID.h>
#include <Physics/PhysicsSystem.h>
#include <Physics/Ragdoll/Ragdoll.h>
#include <Skeleton/SkeletalAnimation.h>
#include <Core/Reference.h>
#include <Core/TempAllocator.h>
#include <Core/JobSystemThreadPool.h>
#include <Math/Quat.h>
#include <Math/Vec3.h>
#include <Math/Real.h>
#include <vector>

namespace biomechanics {

/** Owned resources for a simulator scene (physics system, ground, Jolt human ragdoll). Call destroy_simulator_scene when done. */
struct SimulatorScene {
  BPLayerInterfaceImpl              bp_layer_interface;
  ObjectVsBPLayerFilterImpl         object_vs_bp_filter;
  ObjectLayerPairFilterImpl         object_layer_pair_filter;
  JPH::JobSystemThreadPool*         job_system = nullptr;
  JPH::TempAllocator*               temp_allocator = nullptr;
  JPH::PhysicsSystem*               physics = nullptr;
  JPH::BodyID                       ground_id;
  JPH::RagdollSettings*             ragdoll_settings = nullptr;  // owned; created by create_human_ragdoll_settings
  JPH::Ragdoll*                     ragdoll = nullptr;           // owned; created from ragdoll_settings
  JPH::Ref<JPH::SkeletalAnimation>  standing_anim;                // optional; when set, standing uses this
  JPH::Ref<JPH::SkeletalAnimation>  walking_anim;                 // optional; when set, walking uses this
  /** Initial standing pose (limb positions at start); used to reset Ragdoll->Standing to same pose in space. */
  JPH::RVec3                        initial_standing_root_offset{0, 0, 0};
  JPH::Quat                         initial_standing_root_rotation = JPH::Quat::sIdentity();
  std::vector<JPH::Quat>            initial_standing_joint_rotations;
  std::vector<JPH::Vec3>            initial_standing_joint_translations;
};

/** Call once per process before using Jolt (allocator + type registration). Safe to call multiple times. */
void ensure_jolt_registered();

/** Create physics world, ground plane, and ragdoll. Caller must call destroy_simulator_scene. */
void create_simulator_scene(const SimulatorConfig& config, SimulatorScene& out);

/** Remove ragdoll, ground, and delete all scene resources. */
void destroy_simulator_scene(SimulatorScene& scene);

}  // namespace biomechanics
