/**
 * Smoke test: create a minimal Bullet world and step once.
 */
#include <btBulletDynamicsCommon.h>

int main() {
  btDefaultCollisionConfiguration config;
  btCollisionDispatcher dispatcher(&config);
  btDbvtBroadphase broadphase;
  btSequentialImpulseConstraintSolver solver;
  btDiscreteDynamicsWorld world(&dispatcher, &broadphase, &solver, &config);
  world.setGravity(btVector3(0, -10, 0));
  world.stepSimulation(1.f / 60.f, 1);
  return 0;
}
