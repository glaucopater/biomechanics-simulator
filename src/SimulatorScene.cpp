#include "biomechanics/SimulatorScene.hpp"

namespace biomechanics {

void create_simulator_scene(const SimulatorConfig& config, SimulatorScene& out) {
  out.collision_config = new btDefaultCollisionConfiguration();
  out.dispatcher = new btCollisionDispatcher(out.collision_config);
  out.broadphase = new btDbvtBroadphase();
  out.solver = new btSequentialImpulseConstraintSolver();
  out.world = new btDiscreteDynamicsWorld(out.dispatcher, out.broadphase, out.solver, out.collision_config);
  out.world->setGravity(btVector3(0, config.gravity_y, 0));

  out.ground_shape = new btBoxShape(btVector3(50, 0.5f, 50));
  btTransform ground_transform;
  ground_transform.setIdentity();
  ground_transform.setOrigin(btVector3(0, -0.5f, 0));
  btScalar ground_mass(0.);
  btVector3 ground_inertia(0, 0, 0);
  out.ground_shape->calculateLocalInertia(ground_mass, ground_inertia);
  auto* ground_motion = new btDefaultMotionState(ground_transform);
  btRigidBody::btRigidBodyConstructionInfo ground_rb_info(ground_mass, ground_motion, out.ground_shape, ground_inertia);
  out.ground_body = new btRigidBody(ground_rb_info);
  out.world->addRigidBody(out.ground_body);

  setup_ragdoll(out.world, btVector3(0, config.ragdoll_height, 0), config.ragdoll_scale, out.ragdoll);
}

void destroy_simulator_scene(SimulatorScene& scene) {
  destroy_ragdoll(scene.world, scene.ragdoll);
  if (scene.ground_body && scene.world) {
    scene.world->removeRigidBody(scene.ground_body);
    delete scene.ground_body->getMotionState();
    delete scene.ground_body;
  }
  delete scene.ground_shape;
  delete scene.world;
  delete scene.solver;
  delete scene.broadphase;
  delete scene.dispatcher;
  delete scene.collision_config;
  scene = SimulatorScene{};
}

}  // namespace biomechanics
