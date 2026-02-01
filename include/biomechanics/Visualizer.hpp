#pragma once

#include "biomechanics/Config.hpp"

namespace biomechanics {

/** Run the ragdoll demo with a visualizer window (GLFW + OpenGL). */
void run_demo_visual(const SimulatorConfig& config = {});

}  // namespace biomechanics
