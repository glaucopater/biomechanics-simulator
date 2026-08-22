#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <vector>

namespace biomechanics
{

class HumanRagdoll;

class PoseController
{
public:
    enum class ActionType
    {
        None,
        Jump,
        RaiseArms,
        LowerArms,
        KickRightLeg,
        KickLeftLeg
    };

    enum class State
    {
        Idle,
        Prepare,
        Execute,
        Recover
    };

    PoseController(HumanRagdoll* ragdoll, JPH::PhysicsSystem* physicsSystem);
    ~PoseController();

    void StartAction(ActionType action);
    void Update(float deltaTime);

    ActionType GetCurrentAction() const { return currentAction; }
    State GetState() const { return state; }

    void SetPrepareDuration(float duration) { prepareDuration = duration; }
    void SetExecuteDuration(float duration) { executeDuration = duration; }
    void SetRecoverDuration(float duration) { recoverDuration = duration; }

private:
    HumanRagdoll* ragdoll;
    JPH::PhysicsSystem* physicsSystem;

    ActionType currentAction = ActionType::None;
    State state = State::Idle;

    float prepareDuration = 0.2f;
    float executeDuration = 0.4f;
    float recoverDuration = 0.3f;

    float timer = 0.0f;

    // Action queue for sequential execution
    std::vector<ActionType> actionQueue;

    void UpdateJump(float deltaTime);
    void UpdateRaiseArms(float deltaTime);
    void UpdateLowerArms(float deltaTime);
    void UpdateKickRightLeg(float deltaTime);
    void UpdateKickLeftLeg(float deltaTime);

    void EnqueueAction(ActionType action);
    ActionType DequeueAction();
    bool HasQueuedActions() const { return !actionQueue.empty(); }
};

} // namespace biomechanics
