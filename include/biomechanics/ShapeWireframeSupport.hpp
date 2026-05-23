#pragma once

#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>

namespace biomechanics {
namespace shape_wireframe {

using namespace JPH;

inline bool is_drawable_shape(const Shape* shape) {
  if (!shape)
    return false;
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

inline Vec3 component_mul(Vec3Arg a, Vec3Arg b) {
  return Vec3(a.GetX() * b.GetX(), a.GetY() * b.GetY(), a.GetZ() * b.GetZ());
}

inline float average_scale(Vec3Arg scale) {
  return (scale.GetX() + scale.GetY() + scale.GetZ()) / 3.f;
}

/** Recursively draw supported wireframe shapes. DrawFn must provide sphere/capsule/tapered_capsule/box methods. */
template<typename DrawFn>
bool draw_wireframe_shape(const Shape* shape, Vec3 pos, Quat rot, Vec3 scale, DrawFn&& drawFn) {
  if (!shape)
    return false;
  switch (shape->GetSubType()) {
    case EShapeSubType::Sphere: {
      const SphereShape* sp = static_cast<const SphereShape*>(shape);
      float r = sp->GetRadius() * average_scale(scale);
      drawFn.sphere(pos, rot, r);
      return true;
    }
    case EShapeSubType::Capsule: {
      const CapsuleShape* cap = static_cast<const CapsuleShape*>(shape);
      float r = cap->GetRadius() * average_scale(scale);
      float h = cap->GetHalfHeightOfCylinder() * scale.GetY();
      drawFn.capsule(pos, rot, h, r);
      return true;
    }
    case EShapeSubType::TaperedCapsule: {
      const TaperedCapsuleShape* tc = static_cast<const TaperedCapsuleShape*>(shape);
      float r = tc->GetInnerRadius() * average_scale(scale);
      float half_h = tc->GetLocalBounds().GetExtent().GetY() * 0.5f * scale.GetY();
      drawFn.tapered_capsule(pos, rot, half_h, r);
      return true;
    }
    case EShapeSubType::Box: {
      const BoxShape* box = static_cast<const BoxShape*>(shape);
      Vec3 half = box->GetHalfExtent();
      drawFn.box(pos, rot, Vec3(half.GetX() * scale.GetX(), half.GetY() * scale.GetY(), half.GetZ() * scale.GetZ()));
      return true;
    }
    case EShapeSubType::RotatedTranslated: {
      const RotatedTranslatedShape* rt = static_cast<const RotatedTranslatedShape*>(shape);
      return draw_wireframe_shape(rt->GetInnerShape(), pos + rot * rt->GetPosition(), rot * rt->GetRotation(), scale, drawFn);
    }
    case EShapeSubType::Scaled: {
      const ScaledShape* sc = static_cast<const ScaledShape*>(shape);
      return draw_wireframe_shape(sc->GetInnerShape(), pos, rot, component_mul(scale, sc->GetScale()), drawFn);
    }
    case EShapeSubType::OffsetCenterOfMass: {
      const OffsetCenterOfMassShape* oc = static_cast<const OffsetCenterOfMassShape*>(shape);
      return draw_wireframe_shape(oc->GetInnerShape(), pos + rot * oc->GetOffset(), rot, scale, drawFn);
    }
    default:
      return false;
  }
}

}  // namespace shape_wireframe
}  // namespace biomechanics
