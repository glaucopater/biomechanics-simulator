#pragma once

#include "biomechanics/JoltLayers.hpp"
#include <Jolt.h>
#include <Physics/Ragdoll/Ragdoll.h>
#include <Skeleton/SkeletalAnimation.h>
#include <Core/Reference.h>

namespace biomechanics {

/**
 * Load human ragdoll from JoltPhysics Human.tof (when JPH_OBJECT_STREAM is enabled).
 * Tries assets/Human.tof, Human.tof, then ../../assets/Human.tof, ../assets/Human.tof.
 * Returns nullptr if file not found or load fails; caller owns the returned pointer.
 */
JPH::RagdollSettings* load_human_ragdoll_from_file();

/**
 * Load standing (neutral) and walking animations for the Human rig.
 * Tries assets/Human/neutral.tof, assets/Human/walk.tof (and ../assets/ variants).
 * On success, out_standing and out_walking are set and walking is set to looping.
 * On failure, refs are left unchanged. Only available when JPH_OBJECT_STREAM is enabled.
 */
void load_human_animations(JPH::Ref<JPH::SkeletalAnimation>& out_standing,
                          JPH::Ref<JPH::SkeletalAnimation>& out_walking);

/**
 * Create Jolt human ragdoll settings in code (same structure as JoltPhysics RagdollLoader::sCreate).
 * Used when Human.tof is not available. Caller owns the returned pointer.
 */
JPH::RagdollSettings* create_human_ragdoll_settings();

}  // namespace biomechanics
