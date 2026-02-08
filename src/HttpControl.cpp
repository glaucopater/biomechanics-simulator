#include "biomechanics/HttpControl.hpp"
#include "biomechanics/Log.hpp"
#include "httplib.h"
#include <cstdlib>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>
#include <atomic>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace biomechanics {

namespace {

std::mutex g_mutex;
int g_pending_stance = -1;  // -1 = none, 0 = Standing, 1 = Walking, 2 = Ragdoll, 3 = StandingRaiseLeg
bool g_pending_jump = false;
bool g_pending_test_float = false;
bool g_pending_reset = false;
bool g_pending_position_valid = false;
double g_pending_position_x = 0.0, g_pending_position_y = 0.0, g_pending_position_z = 0.0;
HttpStatusSnapshot g_snapshot;

std::unique_ptr<httplib::Server> g_server;
std::thread g_server_thread;
std::atomic<bool> g_running{false};

const char* stance_to_string(MotionMode m) {
  switch (m) {
    case MotionMode::Standing: return "standing";
    case MotionMode::Walking: return "walking";
    case MotionMode::Ragdoll: return "ragdoll";
    case MotionMode::StandingRaiseLeg: return "standing_raise_leg";
    default: return "standing";
  }
}

/** Returns 0=standing, 1=walking, 2=ragdoll, 3=standing_raise_leg, or -1 if body invalid. */
int parse_stance_from_body(const std::string& body) {
  if (body.find("\"standing_raise_leg\"") != std::string::npos) return 3;
  if (body.find("\"standing\"") != std::string::npos) return 0;
  if (body.find("\"walking\"") != std::string::npos) return 1;
  if (body.find("\"ragdoll\"") != std::string::npos) return 2;
  return -1;
}

std::string snapshot_to_json() {
  std::ostringstream out;
  out << "{\"stance\":\"" << g_snapshot.stance << "\""
      << ",\"root_x\":" << g_snapshot.root_x
      << ",\"root_y\":" << g_snapshot.root_y
      << ",\"root_z\":" << g_snapshot.root_z
      << ",\"root_vx\":" << g_snapshot.root_vx << ",\"root_vy\":" << g_snapshot.root_vy << ",\"root_vz\":" << g_snapshot.root_vz
      << ",\"root_wx\":" << g_snapshot.root_wx << ",\"root_wy\":" << g_snapshot.root_wy << ",\"root_wz\":" << g_snapshot.root_wz
      << ",\"body_count\":" << g_snapshot.body_count
      << ",\"walk_time\":" << g_snapshot.walk_time
      << ",\"walk_phase\":" << g_snapshot.walk_phase;
  out << ",\"log_tail\":[";
  for (size_t i = 0; i < g_snapshot.log_tail.size(); ++i) {
    if (i) out << ",";
    out << "\"";
    for (char c : g_snapshot.log_tail[i]) {
      if (c == '"' || c == '\\') out << '\\';
      out << c;
    }
    out << "\"";
  }
  out << "]}";
  return out.str();
}

void escape_json_string(std::ostringstream& out, const std::string& s) {
  for (char c : s) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c == '\n') out << "\\n";
    else if (c == '\r') out << "\\r";
    else if (c == '\t') out << "\\t";
    else out << c;
  }
}

/** Try to load OpenAPI spec from docs/openapi.yaml (relative to cwd or parent). */
std::string load_openapi_spec() {
  const char* paths[] = {"docs/openapi.yaml", "../docs/openapi.yaml"};
  for (const char* p : paths) {
    std::ifstream f(p);
    if (f) {
      std::ostringstream out;
      out << f.rdbuf();
      return out.str();
    }
  }
  return {};
}

}  // namespace

void http_control_start(int port) {
  if (port <= 0) return;
  if (g_running.exchange(true)) return;

  g_server = std::make_unique<httplib::Server>();

  g_server->Get("/status", [](const httplib::Request&, httplib::Response& res) {
    std::lock_guard<std::mutex> lock(g_mutex);
    res.set_content(snapshot_to_json(), "application/json");
  });

  g_server->Get("/openapi.yaml", [](const httplib::Request&, httplib::Response& res) {
    std::string spec = load_openapi_spec();
    if (spec.empty()) {
      res.status = 404;
      res.set_content("{\"error\":\"openapi.yaml not found (run from repo root or build dir)\"}", "application/json");
    } else {
      res.set_content(spec, "application/x-yaml");
    }
  });

  g_server->Get("/log", [](const httplib::Request& req, httplib::Response& res) {
    int64_t from_ts = 0;
    int64_t to_ts = 9223372036854775807LL;  // max int64
    if (req.has_param("from")) {
      const std::string& v = req.get_param_value("from");
      char* end = nullptr;
      long long n = std::strtoll(v.c_str(), &end, 10);
      if (end && *end == '\0') from_ts = static_cast<int64_t>(n);
    }
    if (req.has_param("to")) {
      const std::string& v = req.get_param_value("to");
      char* end = nullptr;
      long long n = std::strtoll(v.c_str(), &end, 10);
      if (end && *end == '\0') to_ts = static_cast<int64_t>(n);
    }
    std::vector<LogEntry> entries = get_log_entries();
    std::ostringstream out;
    out << "{\"entries\":[";
    bool first = true;
    for (const auto& e : entries) {
      if (e.timestamp_ms < from_ts || e.timestamp_ms > to_ts) continue;
      if (!first) out << ",";
      first = false;
      out << "{\"ts\":" << e.timestamp_ms << ",\"message\":\"";
      escape_json_string(out, e.message);
      out << "\"}";
    }
    out << "]}";
    res.set_content(out.str(), "application/json");
  });

  g_server->Patch("/stance", [](const httplib::Request& req, httplib::Response& res) {
    int stance = parse_stance_from_body(req.body);
    std::lock_guard<std::mutex> lock(g_mutex);
    if (stance >= 0) {
      g_pending_stance = stance;
      res.set_content("{\"stance\":\"" + std::string(stance_to_string(static_cast<MotionMode>(stance))) + "\"}", "application/json");
    } else {
      res.status = 400;
      res.set_content("{\"error\":\"bad body: use {\\\"stance\\\": \\\"standing\\\" | \\\"standing_raise_leg\\\" | \\\"walking\\\" | \\\"ragdoll\\\"}\"}", "application/json");
    }
  });

  auto set_ok = [](httplib::Response& res, const std::string& key, const std::string& value) {
    res.set_content("{\"" + key + "\":\"" + value + "\"}", "application/json");
  };
  g_server->Post("/stance/standing", [set_ok](const httplib::Request&, httplib::Response& res) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_pending_stance = 0;
    set_ok(res, "stance", "standing");
  });
  g_server->Post("/stance/walking", [set_ok](const httplib::Request&, httplib::Response& res) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_pending_stance = 1;
    set_ok(res, "stance", "walking");
  });
  g_server->Post("/stance/ragdoll", [set_ok](const httplib::Request&, httplib::Response& res) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_pending_stance = 2;
    set_ok(res, "stance", "ragdoll");
  });
  g_server->Post("/stance/standing_raise_leg", [set_ok](const httplib::Request&, httplib::Response& res) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_pending_stance = 3;
    set_ok(res, "stance", "standing_raise_leg");
  });
  g_server->Post("/jump", [set_ok](const httplib::Request&, httplib::Response& res) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_pending_jump = true;
    set_ok(res, "action", "jump");
  });
  g_server->Post("/test-float", [set_ok](const httplib::Request&, httplib::Response& res) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_pending_test_float = true;
    set_ok(res, "action", "test-float");
  });
  g_server->Post("/reset", [set_ok](const httplib::Request&, httplib::Response& res) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_pending_reset = true;
    set_ok(res, "action", "reset");
  });

  g_server->Patch("/position", [](const httplib::Request& req, httplib::Response& res) {
    double x = 0.0, y = 0.0, z = 0.0;
    const std::string& body = req.body;
    if (body.find("\"x\"") != std::string::npos) {
      size_t i = body.find("\"x\"");
      i = body.find(':', i); if (i != std::string::npos) x = std::atof(body.c_str() + i + 1);
    }
    if (body.find("\"y\"") != std::string::npos) {
      size_t i = body.find("\"y\"");
      i = body.find(':', i); if (i != std::string::npos) y = std::atof(body.c_str() + i + 1);
    }
    if (body.find("\"z\"") != std::string::npos) {
      size_t i = body.find("\"z\"");
      i = body.find(':', i); if (i != std::string::npos) z = std::atof(body.c_str() + i + 1);
    }
    std::lock_guard<std::mutex> lock(g_mutex);
    g_pending_position_x = x;
    g_pending_position_y = y;
    g_pending_position_z = z;
    g_pending_position_valid = true;
    res.set_content("{\"position\":[" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "]}", "application/json");
  });

  g_server->set_default_headers({{"Access-Control-Allow-Origin", "*"}});

  g_server_thread = std::thread([port]() {
    g_server->listen("127.0.0.1", port);
  });
}

void http_control_stop() {
  if (!g_running.exchange(false)) return;
  if (g_server) g_server->stop();
  if (g_server_thread.joinable()) g_server_thread.join();
  g_server.reset();
}

void http_control_apply_pending_stance(ControllerState& ctrl_state) {
  std::lock_guard<std::mutex> lock(g_mutex);
  if (g_pending_stance >= 0 && g_pending_stance <= 3) {
    MotionMode old_mode = ctrl_state.mode;
    MotionMode new_mode = static_cast<MotionMode>(g_pending_stance);
    ctrl_state.mode = new_mode;
    g_pending_stance = -1;
    log("[HTTP] Stance applied: %s -> %s", stance_to_string(old_mode), stance_to_string(new_mode));
  }
}

bool http_control_consume_pending_jump() {
  std::lock_guard<std::mutex> lock(g_mutex);
  if (!g_pending_jump) return false;
  g_pending_jump = false;
  return true;
}

bool http_control_consume_pending_test_float() {
  std::lock_guard<std::mutex> lock(g_mutex);
  if (!g_pending_test_float) return false;
  g_pending_test_float = false;
  return true;
}

bool http_control_consume_pending_reset() {
  std::lock_guard<std::mutex> lock(g_mutex);
  if (!g_pending_reset) return false;
  g_pending_reset = false;
  return true;
}

bool http_control_consume_pending_position(double& x, double& y, double& z) {
  std::lock_guard<std::mutex> lock(g_mutex);
  if (!g_pending_position_valid) return false;
  x = g_pending_position_x;
  y = g_pending_position_y;
  z = g_pending_position_z;
  g_pending_position_valid = false;
  return true;
}

void http_control_update_snapshot(const SimulatorScene& scene, const ControllerState& ctrl_state) {
  std::lock_guard<std::mutex> lock(g_mutex);
  g_snapshot.stance = stance_to_string(ctrl_state.mode);
  g_snapshot.walk_time = ctrl_state.walk_time;
  g_snapshot.walk_phase = ctrl_state.walk_phase;
  g_snapshot.body_count = 0;
  g_snapshot.root_x = g_snapshot.root_y = g_snapshot.root_z = 0.0;
  g_snapshot.root_vx = g_snapshot.root_vy = g_snapshot.root_vz = 0.0;
  g_snapshot.root_wx = g_snapshot.root_wy = g_snapshot.root_wz = 0.0;

  if (scene.ragdoll) {
    g_snapshot.body_count = static_cast<int>(scene.ragdoll->GetBodyIDs().size());
    JPH::RVec3 root_pos;
    JPH::Quat root_rot;
    scene.ragdoll->GetRootTransform(root_pos, root_rot, true);
    g_snapshot.root_x = static_cast<double>(root_pos.GetX());
    g_snapshot.root_y = static_cast<double>(root_pos.GetY());
    g_snapshot.root_z = static_cast<double>(root_pos.GetZ());
    if (scene.physics) {
      JPH::BodyInterface& bi = scene.physics->GetBodyInterface();
      JPH::BodyID root_id = scene.ragdoll->GetBodyID(0);
      if (!root_id.IsInvalid()) {
        JPH::Vec3 lin = bi.GetLinearVelocity(root_id);
        JPH::Vec3 ang = bi.GetAngularVelocity(root_id);
        g_snapshot.root_vx = static_cast<double>(lin.GetX());
        g_snapshot.root_vy = static_cast<double>(lin.GetY());
        g_snapshot.root_vz = static_cast<double>(lin.GetZ());
        g_snapshot.root_wx = static_cast<double>(ang.GetX());
        g_snapshot.root_wy = static_cast<double>(ang.GetY());
        g_snapshot.root_wz = static_cast<double>(ang.GetZ());
      }
    }
  }

  const auto& lines = get_log_lines();
  const size_t n = (lines.size() > 20) ? 20 : lines.size();
  g_snapshot.log_tail.clear();
  for (size_t i = lines.size() - n; i < lines.size(); ++i)
    g_snapshot.log_tail.push_back(lines[i]);
}

}  // namespace biomechanics
