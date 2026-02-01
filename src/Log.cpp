#include "biomechanics/Log.hpp"
#include <cstdio>
#include <chrono>
#include <fstream>
#include <filesystem>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

namespace biomechanics {

namespace {

constexpr size_t MAX_LOG_LINES = 200;
constexpr size_t LOG_BUF_SIZE = 512;
std::vector<std::string> s_log_lines;
std::string s_log_path = ".cursor/debug.log";
bool s_log_path_inited = false;

void ensure_log_dir() {
  if (s_log_path.find('/') == std::string::npos && s_log_path.find('\\') == std::string::npos)
    return;
  std::error_code ec;
  std::filesystem::create_directories(std::filesystem::path(s_log_path).parent_path(), ec);
}

/** Resolve log path to repo .cursor/debug.log so it's the same file no matter where the app is run from. */
void resolve_log_path() {
  if (s_log_path_inited)
    return;
  s_log_path_inited = true;
  std::error_code ec;
  std::filesystem::path exe_dir;
#ifdef _WIN32
  char buf[MAX_PATH];
  if (GetModuleFileNameA(nullptr, buf, sizeof(buf)) == 0)
    return;
  exe_dir = std::filesystem::path(buf).parent_path();
#else
  exe_dir = std::filesystem::current_path(ec);
#endif
  std::filesystem::path p = exe_dir;
  for (int up = 0; up < 10 && !p.empty(); ++up) {
    if (std::filesystem::exists(p / "CMakeLists.txt", ec) || std::filesystem::exists(p / ".git", ec)) {
      s_log_path = (p / ".cursor" / "debug.log").string();
      return;
    }
    p = p.parent_path();
  }
}

}  // namespace

void init_log_path() {
  resolve_log_path();
}

void log(const char* fmt, ...) {
  resolve_log_path();
  char buf[LOG_BUF_SIZE];
  va_list ap;
  va_start(ap, fmt);
  int n = std::vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0)
    return;
  std::string line(buf, static_cast<size_t>(n) >= sizeof(buf) ? sizeof(buf) - 1 : static_cast<size_t>(n));
  s_log_lines.push_back(line);
  if (s_log_lines.size() > MAX_LOG_LINES)
    s_log_lines.erase(s_log_lines.begin());
  ensure_log_dir();
  std::ofstream f(s_log_path, std::ios::app);
  if (!f) {
    s_log_path = "debug.log";
    f.open(s_log_path, std::ios::app);
  }
  if (f) {
    f << line << "\n";
    f.flush();
  }
}

const std::vector<std::string>& get_log_lines() {
  return s_log_lines;
}

/** Return the path where the log file is written (for display to user). */
const char* get_log_path() {
  return s_log_path.c_str();
}

void clear_log(bool clear_file) {
  resolve_log_path();
  s_log_lines.clear();
  if (clear_file) {
    ensure_log_dir();
    std::ofstream f(s_log_path, std::ios::trunc);
    if (f) {
      f << "[log cleared]\n";
      f.flush();
      f.close();
    }
  }
}

// #region agent log
static const char* k_debug_log_path = "c:/Users/glauc/github/biomechanics-simulator/.cursor/debug.log";

void debug_instrument(const char* location, const char* message, const char* hypothesis_id, int data_val) {
  std::error_code ec;
  std::filesystem::path p(k_debug_log_path);
  std::filesystem::create_directories(p.parent_path(), ec);
  auto t = std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
  std::ofstream f(k_debug_log_path, std::ios::app);
  if (!f) return;
  f << "{\"timestamp\":" << t << ",\"location\":\"" << location << "\",\"message\":\"" << message
    << "\",\"hypothesisId\":\"" << hypothesis_id << "\",\"sessionId\":\"debug-session\",\"runId\":\"run1\"";
  if (data_val != -999) f << ",\"data\":{\"val\":" << data_val << "}";
  f << "}\n";
  f.flush();
}
// #endregion

}  // namespace biomechanics
