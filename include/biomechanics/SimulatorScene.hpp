#pragma once

#include "biomechanics/Config.hpp"
#include "biomechanics/Ragdoll.hpp"
#include <btBulletDynamicsCommon.h>

namespace biomechanics {

/** Owned resources for a simulator scene (world, ground, ragdoll). Call destroy_simulator_scene when done. */
struct SimulatorScene {
  btDefaultCollisionConfiguration* collision_config = nullptr;
  btCollisionDispatcher* dispatcher = nullptr;
  btBroadphaseInterface* broadphase = nullptr;
  btSequentialImpulseConstraintSolver* solver = nullptr;
  btDiscreteDynamicsWorld* world = nullptr;
  btRigidBody* ground_body = nullptr;
  btCollisionShape* ground_shape = nullptr;
  RagdollHandles ragdoll;
};

/** Create world, ground plane, and ragdoll. Returns filled scene; caller must call destroy_simulator_scene. */
void create_simulator_scene(const SimulatorConfig& config, SimulatorScene& out);

/** Remove ragdoll, ground, world, and delete all scene resources. */
void destroy_simulator_scene(SimulatorScene& scene);

}  // namespace biomechanics
