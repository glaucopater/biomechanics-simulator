#include "biomechanics/OpenGLDebugDrawer.hpp"
#include "biomechanics/ShapeWireframeSupport.hpp"
#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Physics/Body/BodyLock.h>
#include <Math/Quat.h>
#include <Math/Vec3.h>
#include <cmath>

namespace biomechanics {

namespace {

using namespace JPH;

void draw_line(float x0, float y0, float z0, float x1, float y1, float z1) {
  glVertex3f(x0, y0, z0);
  glVertex3f(x1, y1, z1);
}

void draw_capsule_wireframe(const Vec3& pos, const Quat& rot, float half_height, float radius) {
  Vec3 axis = rot.RotateAxisY();
  Vec3 p0 = pos - axis * half_height;
  Vec3 p1 = pos + axis * half_height;
  draw_line(p0.GetX(), p0.GetY(), p0.GetZ(), p1.GetX(), p1.GetY(), p1.GetZ());
  const int n = 8;
  Vec3 perp = rot.RotateAxisX();
  Vec3 perp2 = rot.RotateAxisZ();
  for (int i = 0; i < n; ++i) {
    float a = (float)i * (2.f * 3.14159265f / (float)n);
    float c = std::cos(a), s = std::sin(a);
    Vec3 o0 = p0 + (perp * (radius * c) + perp2 * (radius * s));
    Vec3 o1 = p0 + (perp * (radius * std::cos(a + 3.14159265f / n)) + perp2 * (radius * std::sin(a + 3.14159265f / n)));
    draw_line(o0.GetX(), o0.GetY(), o0.GetZ(), o1.GetX(), o1.GetY(), o1.GetZ());
    Vec3 o2 = p1 + (perp * (radius * c) + perp2 * (radius * s));
    Vec3 o3 = p1 + (perp * (radius * std::cos(a + 3.14159265f / n)) + perp2 * (radius * std::sin(a + 3.14159265f / n)));
    draw_line(o2.GetX(), o2.GetY(), o2.GetZ(), o3.GetX(), o3.GetY(), o3.GetZ());
  }
}

void draw_box_wireframe(const Vec3& pos, const Quat& rot, const Vec3& half_extent) {
  Vec3 ax = rot.RotateAxisX();
  Vec3 ay = rot.RotateAxisY();
  Vec3 az = rot.RotateAxisZ();
  Vec3 c[8];
  for (int i = 0; i < 8; ++i) {
    float sx = (i & 1) ? 1.f : -1.f;
    float sy = (i & 2) ? 1.f : -1.f;
    float sz = (i & 4) ? 1.f : -1.f;
    c[i] = pos + ax * (half_extent.GetX() * sx) + ay * (half_extent.GetY() * sy) + az * (half_extent.GetZ() * sz);
  }
  auto edge = [&](int a, int b) {
    draw_line(c[a].GetX(), c[a].GetY(), c[a].GetZ(), c[b].GetX(), c[b].GetY(), c[b].GetZ());
  };
  edge(0, 1); edge(2, 3); edge(4, 5); edge(6, 7);
  edge(0, 2); edge(1, 3); edge(4, 6); edge(5, 7);
  edge(0, 4); edge(1, 5); edge(2, 6); edge(3, 7);
}

struct WireframeDrawFn {
  void sphere(Vec3 pos, Quat rot, float radius) const {
    draw_capsule_wireframe(pos, rot, radius * 0.001f, radius);
  }
  void capsule(Vec3 pos, Quat rot, float half_height, float radius) const {
    draw_capsule_wireframe(pos, rot, half_height, radius);
  }
  void tapered_capsule(Vec3 pos, Quat rot, float half_height, float radius) const {
    draw_capsule_wireframe(pos, rot, half_height, radius);
  }
  void box(Vec3 pos, Quat rot, Vec3 half_extent) const {
    draw_box_wireframe(pos, rot, half_extent);
  }
};

}  // namespace

void OpenGLDebugDrawer::draw_bodies(JPH::PhysicsSystem* physics,
                                    JPH::Ragdoll* ragdoll,
                                    JPH::BodyID ground_id) {
  if (!physics)
    return;
  const BodyLockInterface& lock_iface = physics->GetBodyLockInterface();
  glColor3f(0.6f, 0.85f, 0.9f);

  WireframeDrawFn drawFn;
  auto draw_body = [&](BodyID id, bool always_draw) {
    if (id.IsInvalid())
      return;
    BodyLockRead lock(lock_iface, id);
    if (!lock.Succeeded()) return;
    const Body* b = &lock.GetBody();
    if (!always_draw && !b->IsActive())
      return;
    Vec3 pos = Vec3(b->GetPosition());
    Quat rot = b->GetRotation();
    const Shape* shape = b->GetShape();
    if (!shape)
      return;
    shape_wireframe::draw_wireframe_shape(shape, pos, rot, Vec3::sReplicate(1.f), drawFn);
  };

  if (ragdoll)
    for (BodyID id : ragdoll->GetBodyIDs())
      draw_body(id, true);   // always draw ragdoll so it stays visible when bodies sleep
  draw_body(ground_id, true);  // always draw ground (static bodies are inactive)
}

}  // namespace biomechanics
