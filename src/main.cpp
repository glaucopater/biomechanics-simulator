/**
 * Biomechanics Simulator (C++)
 * Entry point: default runs with visualizer window; use --headless for console-only; --walk = headless in walking mode (writes walk log).
 */

#include "biomechanics/Config.hpp"
#include "biomechanics/Log.hpp"
#include "biomechanics/Simulator.hpp"
#include "biomechanics/SimulatorScene.hpp"
#include "biomechanics/Visualizer.hpp"
#include <cstring>

int main(int argc, char* argv[]) {
  // #region agent log
  biomechanics::debug_instrument("main.cpp:13", "main_entry", "H1");
  // #endregion
  biomechanics::ensure_jolt_registered();
  // #region agent log
  biomechanics::debug_instrument("main.cpp:17", "after_ensure_jolt_main", "H2");
  // #endregion
  biomechanics::init_log_path();
  // #region agent log
  biomechanics::debug_instrument("main.cpp:18", "after_init_log_path", "H1");
  // #endregion
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
  // #region agent log
  biomechanics::debug_instrument("main.cpp:28", headless ? "calling_run_demo" : "calling_run_demo_visual", "H4");
  // #endregion
  if (headless)
    biomechanics::run_demo(config);
  else
    biomechanics::run_demo_visual(config);
  // #region agent log
  biomechanics::debug_instrument("main.cpp:35", "main_exit", "H1");
  // #endregion
  return 0;
}
