#pragma once

#include "biomechanics/JoltLayers.hpp"
#include <Jolt.h>
#include <Physics/Ragdoll/Ragdoll.h>
#include <Skeleton/SkeletalAnimation.h>
#include <Core/Reference.h>
#include <string>
#include <unordered_map>

namespace JPH {
    class Ragdoll;
    class SwingTwistConstraint;
    class Body;
}

namespace biomechanics {

JPH::RagdollSettings* load_human_ragdoll_from_file();
void load_human_animations(JPH::Ref<JPH::SkeletalAnimation>& out_standing,
                          JPH::Ref<JPH::SkeletalAnimation>& out_walking);
JPH::RagdollSettings* create_human_ragdoll_settings();

class HumanRagdoll {
public:
    HumanRagdoll(JPH::Ragdoll* ragdoll);
    ~HumanRagdoll();

    void SetJointTargetAngle(const std::string& jointName, float targetAngle);
    JPH::Ragdoll* GetRagdoll() const { return ragdoll; }
    JPH::Body* GetRootBody() const;

private:
    JPH::Ragdoll* ragdoll;
    std::unordered_map<std::string, JPH::SwingTwistConstraint*> jointMap;
    void BuildJointMap();
};

} // namespace biomechanics
