#include "biomechanics/Log.hpp"
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <mutex>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

namespace biomechanics {

namespace {

constexpr size_t MAX_LOG_LINES = 200;
constexpr size_t LOG_BUF_SIZE = 512;
constexpr size_t LOG_ROTATE_SIZE = 1024 * 1024;  // 1 MB
constexpr int LOG_ROTATE_KEEP = 2;               // keep .1, .2
std::mutex s_log_mutex;
std::vector<LogEntry> s_log_entries;
std::vector<std::string> s_log_lines_cache;  // "[ts] msg" built from s_log_entries
std::string s_log_path = ".cursor/debug.log";
bool s_log_path_inited = false;
std::string s_assets_base;
bool s_assets_base_inited = false;

void ensure_log_dir() {
  if (s_log_path.find('/') == std::string::npos && s_log_path.find('\\') == std::string::npos)
    return;
  std::error_code ec;
  std::filesystem::create_directories(std::filesystem::path(s_log_path).parent_path(), ec);
}

/** Find repo root by walking up from exe dir; returns empty if not found. */
static std::filesystem::path find_repo_root() {
  std::error_code ec;
  std::filesystem::path exe_dir;
#ifdef _WIN32
  char buf[MAX_PATH];
  if (GetModuleFileNameA(nullptr, buf, sizeof(buf)) == 0)
    return {};
  exe_dir = std::filesystem::path(buf).parent_path();
#else
  exe_dir = std::filesystem::current_path(ec);
#endif
  std::filesystem::path p = exe_dir;
  for (int up = 0; up < 10 && !p.empty(); ++up) {
    if (std::filesystem::exists(p / "CMakeLists.txt", ec) || std::filesystem::exists(p / ".git", ec))
      return p;
    p = p.parent_path();
  }
  return {};
}

/** Resolve log path to repo .cursor/debug.log so it's the same file no matter where the app is run from. */
void resolve_log_path() {
  if (s_log_path_inited)
    return;
  s_log_path_inited = true;
  std::filesystem::path root = find_repo_root();
  if (!root.empty())
    s_log_path = (root / ".cursor" / "debug.log").string();
}

void resolve_assets_path() {
  if (s_assets_base_inited)
    return;
  s_assets_base_inited = true;
  std::filesystem::path root = find_repo_root();
  if (!root.empty())
    s_assets_base = (root / "assets").string();
}

/** If path exists and exceeds LOG_ROTATE_SIZE, rotate: .log -> .log.1, .log.1 -> .log.2, then truncate. */
void rotate_log_if_needed(const std::string& path) {
  std::error_code ec;
  std::filesystem::path p(path);
  if (!std::filesystem::is_regular_file(p, ec) || ec)
    return;
  auto size = std::filesystem::file_size(p, ec);
  if (ec || size <= LOG_ROTATE_SIZE)
    return;
  std::filesystem::path p1(path + ".1");
  std::filesystem::path p2(path + ".2");
  if (LOG_ROTATE_KEEP >= 2 && std::filesystem::exists(p2, ec))
    std::filesystem::remove(p2, ec);
  if (LOG_ROTATE_KEEP >= 1 && std::filesystem::exists(p1, ec))
    std::filesystem::rename(p1, p2, ec);
  std::filesystem::rename(p, p1, ec);
}

}  // namespace

void init_log_path() {
  resolve_log_path();
  ensure_log_dir();
  rotate_log_if_needed(s_log_path);
}

static int64_t now_ms() {
  return static_cast<int64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
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
  std::string message(buf, static_cast<size_t>(n) >= sizeof(buf) ? sizeof(buf) - 1 : static_cast<size_t>(n));
  int64_t ts = now_ms();
  {
    std::lock_guard<std::mutex> lock(s_log_mutex);
    s_log_entries.push_back({ts, message});
    if (s_log_entries.size() > MAX_LOG_LINES)
      s_log_entries.erase(s_log_entries.begin());
    s_log_lines_cache.clear();
    s_log_lines_cache.reserve(s_log_entries.size());
    for (const auto& e : s_log_entries)
      s_log_lines_cache.push_back("[" + std::to_string(e.timestamp_ms) + "] " + e.message);
  }
  ensure_log_dir();
  std::ofstream f(s_log_path, std::ios::app);
  if (!f) {
    s_log_path = "debug.log";
    f.open(s_log_path, std::ios::app);
  }
  if (f) {
    f << "[" << ts << "] " << message << "\n";
    f.flush();
  }
}

const std::vector<std::string>& get_log_lines() {
  std::lock_guard<std::mutex> lock(s_log_mutex);
  if (s_log_lines_cache.size() != s_log_entries.size()) {
    s_log_lines_cache.clear();
    s_log_lines_cache.reserve(s_log_entries.size());
    for (const auto& e : s_log_entries)
      s_log_lines_cache.push_back("[" + std::to_string(e.timestamp_ms) + "] " + e.message);
  }
  return s_log_lines_cache;
}

std::vector<LogEntry> get_log_entries() {
  std::lock_guard<std::mutex> lock(s_log_mutex);
  return s_log_entries;
}

/** Return the path where the log file is written (for display to user). */
const char* get_log_path() {
  return s_log_path.c_str();
}

const char* get_assets_base_path() {
  resolve_assets_path();
  return s_assets_base.empty() ? "" : s_assets_base.c_str();
}

void clear_log(bool clear_file) {
  resolve_log_path();
  {
    std::lock_guard<std::mutex> lock(s_log_mutex);
    s_log_entries.clear();
    s_log_lines_cache.clear();
  }
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
void debug_instrument(const char* location, const char* message, const char* hypothesis_id, int data_val) {
  resolve_log_path();
  std::error_code ec;
  std::filesystem::path p(s_log_path);
  std::filesystem::create_directories(p.parent_path(), ec);
  auto t = std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
  std::ofstream f(s_log_path, std::ios::app);
  if (!f) return;
  f << "{\"timestamp\":" << t << ",\"location\":\"" << location << "\",\"message\":\"" << message
    << "\",\"hypothesisId\":\"" << hypothesis_id << "\",\"sessionId\":\"debug-session\",\"runId\":\"run1\"";
  if (data_val != -999) f << ",\"data\":{\"val\":" << data_val << "}";
  f << "}\n";
  f.flush();
}
// #endregion

}  // namespace biomechanics
