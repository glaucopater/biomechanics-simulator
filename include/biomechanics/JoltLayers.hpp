#pragma once

#include <Jolt.h>
#include <Physics/Collision/ObjectLayer.h>
#include <Physics/Collision/BroadPhase/BroadPhaseLayer.h>

namespace biomechanics {

/** Object layers: non-moving (ground) and moving (ragdoll). */
namespace Layers {
static constexpr JPH::ObjectLayer NON_MOVING = 0;
static constexpr JPH::ObjectLayer MOVING = 1;
static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
}

/** Broad phase layers for Jolt PhysicsSystem::Init. */
class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
  static constexpr JPH::BroadPhaseLayer NON_MOVING { 0 };
  static constexpr JPH::BroadPhaseLayer MOVING { 1 };
  static constexpr JPH::uint NUM_LAYERS = 2;

  JPH::uint GetNumBroadPhaseLayers() const override { return NUM_LAYERS; }
  JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
    return inLayer == Layers::NON_MOVING ? NON_MOVING : MOVING;
  }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
  const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override {
    return inLayer == NON_MOVING ? "NON_MOVING" : "MOVING";
  }
#endif
};

/** Filter: object layer vs broad phase layer (allows collision with both). */
class ObjectVsBPLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
  bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override {
    (void)inLayer1;
    (void)inLayer2;
    return true;
  }
};

/** Filter: which object layer pairs collide (moving vs non-moving, moving vs moving). */
class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter {
public:
  bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::ObjectLayer inLayer2) const override {
    (void)inLayer1;
    (void)inLayer2;
    return true;
  }
};

}  // namespace biomechanics
