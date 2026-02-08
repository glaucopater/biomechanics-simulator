#pragma once

#include <Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>

namespace biomechanics {

/** Draw Jolt physics bodies as OpenGL wireframe (capsules and box). */
class OpenGLDebugDrawer {
public:
  OpenGLDebugDrawer() = default;

  /** Draw all ragdoll bodies and the ground body. Call between glBegin(GL_LINES) and glEnd(). */
  void draw_bodies(JPH::PhysicsSystem* physics,
                   JPH::Ragdoll* ragdoll,
                   JPH::BodyID ground_id);
};

}  // namespace biomechanics
