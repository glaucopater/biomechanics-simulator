#pragma once

#include <Jolt.h>
#include <Physics/Body/BodyID.h>
#include <Physics/Body/BodyInterface.h>
#include <Physics/PhysicsSystem.h>
#include <Physics/Collision/Shape/Shape.h>
#include <Physics/Constraints/Constraint.h>
#include <Core/Reference.h>
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

/** Ragdoll handles: body IDs, shapes (kept alive), and constraints. Caller must destroy via destroy_ragdoll. */
struct RagdollHandles {
  std::vector<JPH::BodyID>              bodies;
  std::vector<JPH::RefConst<JPH::Shape>> shapes;
  std::vector<JPH::Constraint*>         constraints;
};

/**
 * Create a humanoid ragdoll in the given physics system and append to the output.
 * Caller owns the returned data and must call destroy_ragdoll when done.
 */
void setup_ragdoll(JPH::PhysicsSystem* physics,
                   JPH::BodyInterface& body_interface,
                   const JPH::RVec3& position_offset,
                   float scale,
                   RagdollHandles& out);

/** Remove ragdoll from physics system and free constraints and bodies. */
void destroy_ragdoll(JPH::PhysicsSystem* physics,
                     JPH::BodyInterface& body_interface,
                     RagdollHandles& handles);

/** Reset ragdoll to standing pose and zero velocities in place (no destroy). Use for Reset button. */
void reset_ragdoll_pose(JPH::BodyInterface& body_interface,
                       RagdollHandles& handles,
                       const JPH::RVec3& position_offset,
                       float scale);

}  // namespace biomechanics
