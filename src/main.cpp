/**
 * Biomechanics Simulator (C++)
 * Entry point: default runs with visualizer window; use --headless for console-only; --walk = headless in walking mode (writes walk log).
 */

#include "biomechanics/Config.hpp"
#include "biomechanics/Log.hpp"
#include "biomechanics/Simulator.hpp"
#include "biomechanics/SimulatorScene.hpp"
#include "biomechanics/Visualizer.hpp"
#include <cstdlib>
#include <cstring>

int main(int argc, char* argv[]) {
#ifdef BIOMECH_AGENT_DEBUG
  biomechanics::debug_instrument("main.cpp:13", "main_entry", "H1");
#endif
  biomechanics::ensure_jolt_registered();
#ifdef BIOMECH_AGENT_DEBUG
  biomechanics::debug_instrument("main.cpp:17", "after_ensure_jolt_main", "H2");
#endif
  biomechanics::init_log_path();
#ifdef BIOMECH_AGENT_DEBUG
  biomechanics::debug_instrument("main.cpp:18", "after_init_log_path", "H1");
#endif
  bool headless = false;
  bool walk_mode = false;
  int http_port = 0;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--headless") == 0)
      headless = true;
    else if (std::strcmp(argv[i], "--walk") == 0)
      walk_mode = true;
    else if (std::strcmp(argv[i], "--http-port") == 0 && i + 1 < argc) {
      http_port = std::atoi(argv[i + 1]);
      ++i;
    }
  }
  biomechanics::SimulatorConfig config;
  if (walk_mode) {
    headless = true;
    config.default_motion_mode = 1;  // Walking
    config.num_steps = 360;           // ~6 s of walk samples
  }
#ifdef BIOMECH_AGENT_DEBUG
  biomechanics::debug_instrument("main.cpp:28", headless ? "calling_run_demo" : "calling_run_demo_visual", "H4");
#endif
  if (headless)
    biomechanics::run_demo(config);
  else
    biomechanics::run_demo_visual(config, http_port);
#ifdef BIOMECH_AGENT_DEBUG
  biomechanics::debug_instrument("main.cpp:35", "main_exit", "H1");
#endif
  return 0;
}
