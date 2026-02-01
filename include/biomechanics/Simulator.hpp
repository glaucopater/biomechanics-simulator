#pragma once

#include "biomechanics/Config.hpp"

namespace biomechanics {

/** Run the default ragdoll demo (create world, ragdoll, step, cleanup). */
void run_demo(const SimulatorConfig& config = {});

}  // namespace biomechanics
