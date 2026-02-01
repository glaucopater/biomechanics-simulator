/**
 * Biomechanics Simulator (C++)
 * Entry point: default runs with visualizer window; use --headless for console-only; --walk = headless in walking mode (writes walk log).
 */

#include "biomechanics/Config.hpp"
#include "biomechanics/Log.hpp"
#include "biomechanics/Simulator.hpp"
#include "biomechanics/Visualizer.hpp"
#include <cstring>

int main(int argc, char* argv[]) {
  biomechanics::init_log_path();
  bool headless = false;
  bool walk_mode = false;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--headless") == 0)
      headless = true;
    else if (std::strcmp(argv[i], "--walk") == 0)
      walk_mode = true;
  }
  biomechanics::SimulatorConfig config;
  if (walk_mode) {
    headless = true;
    config.default_motion_mode = 1;  // Walking
    config.num_steps = 360;           // ~6 s of walk samples
  }
  if (headless)
    biomechanics::run_demo(config);
  else
    biomechanics::run_demo_visual(config);
  return 0;
}
