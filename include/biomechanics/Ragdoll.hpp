#pragma once

#include <btBulletDynamicsCommon.h>
#include <vector>

namespace biomechanics {

enum class BodyPart : int {
  Pelvis,
  Spine,
  Head,
  LeftUpperLeg,
  LeftLowerLeg,
  RightUpperLeg,
  RightLowerLeg,
  LeftUpperArm,
  LeftLowerArm,
  RightUpperArm,
  RightLowerArm,
  Count
};

/** Ragdoll handles: bodies, shapes, and joints to be removed/deleted by the caller. */
struct RagdollHandles {
  std::vector<btRigidBody*>       bodies;
  std::vector<btCollisionShape*> shapes;
  std::vector<btTypedConstraint*> joints;
};

/**
 * Create a humanoid ragdoll in the given world and append to the output vectors.
 * Caller owns the returned bodies, shapes, and joints and must remove/delete them.
 */
void setup_ragdoll(btDynamicsWorld* world,
                   const btVector3& position_offset,
                   btScalar scale,
                   RagdollHandles& out);

/** Remove ragdoll from world and free bodies, shapes, and joints. */
void destroy_ragdoll(btDynamicsWorld* world, RagdollHandles& handles);

}  // namespace biomechanics
