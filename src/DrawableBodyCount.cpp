#include "biomechanics/DrawableBodyCount.hpp"
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>

namespace biomechanics {

using namespace JPH;

namespace {

bool is_drawable_shape(const Shape* shape) {
  if (!shape) return false;
  switch (shape->GetSubType()) {
    case EShapeSubType::Sphere:
    case EShapeSubType::Capsule:
    case EShapeSubType::Box:
    case EShapeSubType::TaperedCapsule:
      return true;
    case EShapeSubType::RotatedTranslated: {
      const RotatedTranslatedShape* rt = static_cast<const RotatedTranslatedShape*>(shape);
      return is_drawable_shape(rt->GetInnerShape());
    }
    case EShapeSubType::Scaled: {
      const ScaledShape* sc = static_cast<const ScaledShape*>(shape);
      return is_drawable_shape(sc->GetInnerShape());
    }
    case EShapeSubType::OffsetCenterOfMass: {
      const OffsetCenterOfMassShape* oc = static_cast<const OffsetCenterOfMassShape*>(shape);
      return is_drawable_shape(oc->GetInnerShape());
    }
    default:
      return false;
  }
}

}  // namespace

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
    if (is_drawable_shape(shape))
      count++;
  };
  if (ragdoll)
    for (BodyID id : ragdoll->GetBodyIDs())
      count_body(id);
  count_body(ground_id);
  return count;
}

}  // namespace biomechanics
