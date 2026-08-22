#include "biomechanics/PoseController.hpp"
#include "biomechanics/HumanRagdoll.hpp"
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyInterface.h>

namespace biomechanics
{

PoseController::PoseController(HumanRagdoll* ragdoll, JPH::PhysicsSystem* physicsSystem)
    : ragdoll(ragdoll), physicsSystem(physicsSystem)
{
}

PoseController::~PoseController()
{
}

void PoseController::StartAction(ActionType action)
{
    if (currentAction == ActionType::None && state == State::Idle)
    {
        currentAction = action;
        state = State::Prepare;
        timer = 0.0f;
    }
}

void PoseController::Update(float deltaTime)
{
    switch (currentAction)
    {
    case ActionType::Jump:
        UpdateJump(deltaTime);
        break;
    case ActionType::RaiseArms:
        UpdateRaiseArms(deltaTime);
        break;
    case ActionType::LowerArms:
        UpdateLowerArms(deltaTime);
        break;
    case ActionType::None:
    default:
        break;
    }
}

void PoseController::UpdateJump(float deltaTime)
{
    timer += deltaTime;

    switch (state)
    {
    case State::Prepare:
    {
        ragdoll->SetJointTargetAngle("hip_left", -0.5f);
        ragdoll->SetJointTargetAngle("hip_right", -0.5f);
        ragdoll->SetJointTargetAngle("knee_left", -0.8f);
        ragdoll->SetJointTargetAngle("knee_right", -0.8f);

        if (timer >= prepareDuration)
        {
            state = State::Execute;
            timer = 0.0f;
        }
        break;
    }
    case State::Execute:
    {
        ragdoll->SetJointTargetAngle("hip_left", 0.0f);
        ragdoll->SetJointTargetAngle("hip_right", 0.0f);
        ragdoll->SetJointTargetAngle("knee_left", 0.0f);
        ragdoll->SetJointTargetAngle("knee_right", 0.0f);

        auto* rootBody = ragdoll->GetRootBody();
        if (rootBody && rootBody->IsActive())
        {
            JPH::BodyInterface& bodyInterface = physicsSystem->GetBodyInterface();
            bodyInterface.AddImpulse(rootBody->GetID(), JPH::Vec3(0, 8.0f, 0));
        }

        if (timer >= executeDuration)
        {
            state = State::Recover;
            timer = 0.0f;
        }
        break;
    }
    case State::Recover:
    {
        ragdoll->SetJointTargetAngle("hip_left", 0.0f);
        ragdoll->SetJointTargetAngle("hip_right", 0.0f);
        ragdoll->SetJointTargetAngle("knee_left", 0.0f);
        ragdoll->SetJointTargetAngle("knee_right", 0.0f);

        if (timer >= recoverDuration)
        {
            currentAction = ActionType::None;
            state = State::Idle;
            timer = 0.0f;
        }
        break;
    }
    default:
        break;
    }
}

void PoseController::UpdateRaiseArms(float deltaTime)
{
    timer += deltaTime;

    switch (state)
    {
    case State::Prepare:
    {
        ragdoll->SetJointTargetAngle("hip_left", -0.1f);
        ragdoll->SetJointTargetAngle("hip_right", -0.1f);

        if (timer >= prepareDuration)
        {
            state = State::Execute;
            timer = 0.0f;
        }
        break;
    }
    case State::Execute:
    {
        ragdoll->SetJointTargetAngle("shoulder_left", 2.5f);
        ragdoll->SetJointTargetAngle("shoulder_right", 2.5f);

        if (timer >= executeDuration)
        {
            state = State::Recover;
            timer = 0.0f;
        }
        break;
    }
    case State::Recover:
    {
        ragdoll->SetJointTargetAngle("shoulder_left", 0.0f);
        ragdoll->SetJointTargetAngle("shoulder_right", 0.0f);
        ragdoll->SetJointTargetAngle("hip_left", 0.0f);
        ragdoll->SetJointTargetAngle("hip_right", 0.0f);

        if (timer >= recoverDuration)
        {
            currentAction = ActionType::None;
            state = State::Idle;
            timer = 0.0f;
        }
        break;
    }
    default:
        break;
    }
}

void PoseController::UpdateLowerArms(float deltaTime)
{
    timer += deltaTime;

    switch (state)
    {
    case State::Prepare:
    {
        if (timer >= prepareDuration)
        {
            state = State::Execute;
            timer = 0.0f;
        }
        break;
    }
    case State::Execute:
    {
        ragdoll->SetJointTargetAngle("shoulder_left", 0.0f);
        ragdoll->SetJointTargetAngle("shoulder_right", 0.0f);

        if (timer >= executeDuration)
        {
            state = State::Recover;
            timer = 0.0f;
        }
        break;
    }
    case State::Recover:
    {
        if (timer >= recoverDuration)
        {
            currentAction = ActionType::None;
            state = State::Idle;
            timer = 0.0f;
        }
        break;
    }
    default:
        break;
    }
}

} // namespace biomechanics
