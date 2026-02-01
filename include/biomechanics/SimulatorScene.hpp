#pragma once

#include "biomechanics/Config.hpp"
#include "biomechanics/JoltLayers.hpp"
#include "biomechanics/Ragdoll.hpp"
#include <Jolt.h>
#include <Physics/Body/BodyID.h>
#include <Physics/PhysicsSystem.h>
#include <Core/TempAllocator.h>
#include <Core/JobSystemThreadPool.h>

namespace biomechanics {

/** Owned resources for a simulator scene (physics system, ground, ragdoll). Call destroy_simulator_scene when done. */
struct SimulatorScene {
  BPLayerInterfaceImpl              bp_layer_interface;
  ObjectVsBPLayerFilterImpl         object_vs_bp_filter;
  ObjectLayerPairFilterImpl         object_layer_pair_filter;
  JPH::JobSystemThreadPool*         job_system = nullptr;
  JPH::TempAllocator*               temp_allocator = nullptr;
  JPH::PhysicsSystem*               physics = nullptr;
  JPH::BodyID                       ground_id;
  RagdollHandles                    ragdoll;
};

/** Call once per process before using Jolt (allocator + type registration). Safe to call multiple times. */
void ensure_jolt_registered();

/** Create physics world, ground plane, and ragdoll. Caller must call destroy_simulator_scene. */
void create_simulator_scene(const SimulatorConfig& config, SimulatorScene& out);

/** Remove ragdoll, ground, and delete all scene resources. */
void destroy_simulator_scene(SimulatorScene& scene);

}  // namespace biomechanics
