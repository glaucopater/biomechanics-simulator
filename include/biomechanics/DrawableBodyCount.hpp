#pragma once

#include <Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>

namespace biomechanics {

/**
 * Count how many bodies would be drawn by OpenGLDebugDrawer::draw_bodies (no GL).
 * Used by E2E tests to verify "what will be rendered": every ragdoll body + ground
 * must be drawable (Box, Capsule, or RotatedTranslated wrapping Capsule/Box).
 * Returns the same value that would be bodies_drawn after draw_bodies.
 */
int count_drawable_bodies(JPH::PhysicsSystem* physics,
                         JPH::Ragdoll* ragdoll,
                         JPH::BodyID ground_id);

}  // namespace biomechanics
