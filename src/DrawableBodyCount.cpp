#include "biomechanics/DrawableBodyCount.hpp"
#include "biomechanics/ShapeWireframeSupport.hpp"
#include <Jolt/Physics/Body/BodyLock.h>

namespace biomechanics {

using namespace JPH;

int count_drawable_bodies(JPH::PhysicsSystem* physics,
                         JPH::Ragdoll* ragdoll,
                         JPH::BodyID ground_id) {
  if (!physics) return 0;
  const BodyLockInterface& lock_iface = physics->GetBodyLockInterface();
  int count = 0;
  auto count_body = [&](BodyID id) {
    if (id.IsInvalid()) return;
    BodyLockRead lock(lock_iface, id);
    if (!lock.Succeeded()) return;
    const Body* b = &lock.GetBody();
    const Shape* shape = b->GetShape();
    if (shape_wireframe::is_drawable_shape(shape))
      count++;
  };
  if (ragdoll)
    for (BodyID id : ragdoll->GetBodyIDs())
      count_body(id);
  count_body(ground_id);
  return count;
}

}  // namespace biomechanics
