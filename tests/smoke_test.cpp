/**
 * Smoke test: create a minimal Jolt physics world and step once.
 */
#include <Jolt.h>
#include <RegisterTypes.h>
#include <Core/Memory.h>
#include <Core/Factory.h>
#include <Core/TempAllocator.h>
#include <Core/JobSystemThreadPool.h>
#include <Physics/PhysicsSystem.h>
#include <Physics/Body/Body.h>
#include <Physics/Body/BodyCreationSettings.h>
#include <Physics/Collision/Shape/BoxShape.h>
#include <Physics/Collision/ObjectLayer.h>
#include <Physics/Collision/BroadPhase/BroadPhaseLayer.h>

namespace {
class BPLayerImpl : public JPH::BroadPhaseLayerInterface {
public:
  uint GetNumBroadPhaseLayers() const override { return 1; }
  JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override { return 0; }
};
class ObjectVsBPImpl : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
  bool ShouldCollide(JPH::ObjectLayer, JPH::BroadPhaseLayer) const override { return true; }
};
class ObjectLayerPairImpl : public JPH::ObjectLayerPairFilter {
public:
  bool ShouldCollide(JPH::ObjectLayer, JPH::ObjectLayer) const override { return true; }
};
}  // namespace

int main() {
  JPH::RegisterDefaultAllocator();
  JPH::Factory::sRegisterAllTypes();

  BPLayerImpl bp_layer;
  ObjectVsBPImpl object_vs_bp;
  ObjectLayerPairImpl object_pair;

  JPH::PhysicsSystem physics;
  physics.Init(64, 0, 1024, 512, bp_layer, object_vs_bp, object_pair);
  physics.SetGravity(JPH::Vec3(0, -10, 0));

  JPH::BoxShapeSettings box(JPH::Vec3(1, 1, 1));
  JPH::ShapeSettings::ShapeResult res = box.Create();
  JPH::BodyCreationSettings bcs(res.Get(), JPH::RVec3(0, 0, 0), JPH::Quat::sIdentity(), JPH::EMotionType::Static, 0);
  JPH::Body* body = physics.GetBodyInterface().CreateBody(bcs);
  if (body) {
    physics.GetBodyInterface().AddBody(body->GetID(), JPH::EActivation::DontActivate);
    physics.GetBodyInterface().RemoveBody(body->GetID());
    physics.GetBodyInterface().DestroyBody(body->GetID());
  }

  JPH::JobSystemThreadPool job_system(1, 1, 0);
  JPH::TempAllocatorImpl temp_alloc(1024 * 1024);
  physics.Update(1.f / 60.f, 1, &temp_alloc, &job_system);
  return 0;
}
