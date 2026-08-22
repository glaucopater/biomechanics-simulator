#include "biomechanics/SimulatorScene.hpp"
#include "biomechanics/Config.hpp"
#include "biomechanics/HumanRagdoll.hpp"
#include "biomechanics/PoseController.hpp"
#include "biomechanics/Log.hpp"
#include <Jolt/Physics/PhysicsSystem.h>

namespace biomechanics
{

SimulatorScene::SimulatorScene()
    : physicsSystem(nullptr)
    , ragdoll(nullptr)
    , poseController(nullptr)
{
}

SimulatorScene::~SimulatorScene()
{
    delete poseController;
    delete ragdoll;
    delete physicsSystem;
}

void SimulatorScene::Initialize()
{
    LOG_INFO("SimulatorScene::Initialize()");

    // Initialize physics system
    physicsSystem = new JPH::PhysicsSystem();
    physicsSystem->Init(
        cMaxBodies,
        cNumBodyMutexes,
        cMaxBodyPairs,
        cMaxConstraints
    );

    // Create ragdoll
    ragdoll = new HumanRagdoll(physicsSystem);
    ragdoll->Create();

    // Create pose controller
    poseController = new PoseController(ragdoll, physicsSystem);

    LOG_INFO("SimulatorScene initialized successfully");
}

void SimulatorScene::Update(float deltaTime)
{
    // Get physics settings from config for stability
    auto settings = config::getPhysicsSettings();

    // Substepped physics update for better ragdoll stability
    float subStepDt = deltaTime / static_cast<float>(settings.numSubSteps);

    for (int i = 0; i < settings.numSubSteps; ++i)
    {
        physicsSystem->Update(
            subStepDt,
            settings.velocityIterations,
            settings.positionIterations,
            tempAllocator,
            jobSystem
        );
    }

    // Update pose controller (actions)
    if (poseController)
    {
        poseController->Update(deltaTime);
    }
}

void SimulatorScene::Reset()
{
    LOG_INFO("SimulatorScene::Reset()");

    if (ragdoll)
    {
        ragdoll->Reset();
    }

    if (poseController)
    {
        delete poseController;
        poseController = new PoseController(ragdoll, physicsSystem);
    }
}

void SimulatorScene::StartAction(PoseController::ActionType action)
{
    if (poseController)
    {
        poseController->StartAction(action);
    }
}

HumanRagdoll* SimulatorScene::GetRagdoll() const
{
    return ragdoll;
}

PoseController* SimulatorScene::GetPoseController() const
{
    return poseController;
}

JPH::PhysicsSystem* SimulatorScene::GetPhysicsSystem() const
{
    return physicsSystem;
}

} // namespace biomechanics
