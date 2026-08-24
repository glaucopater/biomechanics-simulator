#include "biomechanics/HumanRagdoll.hpp"
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace biomechanics {

using namespace JPH;

HumanRagdoll::HumanRagdoll(Ragdoll* ragdoll) : ragdoll(ragdoll) {
    BuildJointMap();
}

HumanRagdoll::~HumanRagdoll() {
    // Ragdoll is owned by SimulatorScene, don't delete
}

void HumanRagdoll::BuildJointMap() {
    if (!ragdoll || !ragdoll->GetSkeleton()) return;
    
    const Skeleton* skeleton = ragdoll->GetSkeleton();
    for (int i = 0; i < skeleton->GetJointCount(); ++i) {
        const char* jointName = skeleton->GetJointName(i);
        if (jointName && jointName[0] != '\0') {
            Ragdoll::Constraint* constraint = ragdoll->GetConstraint(i);
            if (constraint) {
                SwingTwistConstraint* st = DynamicCast<SwingTwistConstraint>(constraint);
                if (st) {
                    jointMap[std::string(jointName)] = st;
                }
            }
        }
    }
}

void HumanRagdoll::SetJointTargetAngle(const std::string& jointName, float targetAngle) {
    auto it = jointMap.find(jointName);
    if (it == jointMap.end()) return;
    
    SwingTwistConstraint* constraint = it->second;
    if (!constraint || !constraint->IsActive()) return;
    
    // Set twist motor target angle
    constraint->SetTwistMotorTargetAngle(targetAngle);
}

Body* HumanRagdoll::GetRootBody() const {
    if (!ragdoll || ragdoll->GetBodyIDs().empty()) return nullptr;
    BodyID rootBodyID = ragdoll->GetBodyIDs()[0];
    // Note: Getting the actual Body* requires access to PhysicsSystem's BodyInterface
    // This should be called from a context where the BodyInterface is available
    return nullptr;
}

} // namespace biomechanics
