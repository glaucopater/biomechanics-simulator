#pragma once

#include "biomechanics/PoseController.hpp"
#include "biomechanics/SimulatorScene.hpp"
#include <string>
#include <vector>

namespace biomechanics {

/** Snapshot of simulator state for GET /status (written by main thread). */
struct HttpStatusSnapshot {
  std::string stance;   // "standing" | "walking" | "ragdoll"
  double root_x = 0.0;
  double root_y = 0.0;
  double root_z = 0.0;
  double root_vx = 0.0, root_vy = 0.0, root_vz = 0.0;  // root linear velocity
  double root_wx = 0.0, root_wy = 0.0, root_wz = 0.0;  // root angular velocity (spin)
  int body_count = 0;
  float walk_time = 0.f;
  float walk_phase = 0.f;  // procedural walk phase [0, 2*PI)
  std::vector<std::string> log_tail;
};

/**
 * Start the HTTP control server on 127.0.0.1:port (background thread).
 * GET /status returns current snapshot; PATCH /stance with {"stance":"standing"|"walking"|"ragdoll"} queues a stance change.
 * No-op if port <= 0. Call from main thread before entering the visualizer loop.
 */
void http_control_start(int port);

/** Stop the server and join the thread. Call from main thread on exit. */
void http_control_stop();

/**
 * Apply any pending stance from the HTTP thread to ctrl_state (main thread only).
 * Call once per frame at start of frame before apply_pose_control.
 */
void http_control_apply_pending_stance(ControllerState& ctrl_state);

/**
 * Consume pending actions (main thread only). Call once per frame when HTTP is enabled.
 * Returns true if the action was requested so the caller can perform it.
 */
bool http_control_consume_pending_jump();
bool http_control_consume_pending_test_float();
bool http_control_consume_pending_reset();

/**
 * If a position was requested via PATCH /position, fills x,y,z and returns true (clears pending).
 * Main thread only. Use to snap root to the given position and zero velocities.
 */
bool http_control_consume_pending_position(double& x, double& y, double& z);

/**
 * Update the status snapshot from current scene and ctrl_state (main thread only).
 * Call once per frame when HTTP is enabled (e.g. after physics step).
 */
void http_control_update_snapshot(const SimulatorScene& scene, const ControllerState& ctrl_state);

}  // namespace biomechanics
