#pragma once

#include "biomechanics/Config.hpp"

namespace biomechanics {

/** Run the ragdoll demo with a visualizer window (GLFW + OpenGL). If http_port > 0, enables GET /status and PATCH /stance on 127.0.0.1:port. */
void run_demo_visual(const SimulatorConfig& config = {}, int http_port = 0);

}  // namespace biomechanics
