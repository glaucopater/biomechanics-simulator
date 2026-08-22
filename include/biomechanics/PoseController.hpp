#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <vector>
#include <string>

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

    // Start an action (if another is running, it's queued)
    void StartAction(ActionType action);

    // Update the controller (call every frame with delta time)
    void Update(float deltaTime);

    // Get current action
    ActionType GetCurrentAction() const { return currentAction; }

    // Get current state
    State GetState() const { return state; }

    // Set action durations (in seconds)
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

    // Internal state machine handlers
    void UpdateJump(float deltaTime);
    void UpdateRaiseArms(float deltaTime);
    void UpdateLowerArms(float deltaTime);
    void UpdateKickRightLeg(float deltaTime);
    void UpdateKickLeftLeg(float deltaTime);

    // Queue management
    void UpdateQueue(float deltaTime);
    void EnqueueAction(ActionType action);
    ActionType DequeueAction();
    bool HasQueuedActions() const { return !actionQueue.empty(); }
};

} // namespace biomechanics
