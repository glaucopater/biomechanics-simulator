#pragma once

#include "biomechanics/Ragdoll.hpp"
#include <Jolt.h>
#include <Physics/Body/BodyID.h>
#include <Physics/PhysicsSystem.h>

namespace biomechanics {

/** Draw Jolt physics bodies as OpenGL wireframe (capsules and box). */
class OpenGLDebugDrawer {
public:
  OpenGLDebugDrawer() = default;

  /** Draw all ragdoll bodies and the ground body. Call between glBegin(GL_LINES) and glEnd(). */
  void draw_bodies(JPH::PhysicsSystem* physics,
                   const RagdollHandles& ragdoll,
                   JPH::BodyID ground_id);
};

}  // namespace biomechanics
