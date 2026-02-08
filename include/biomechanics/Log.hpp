#pragma once

#include <cstdarg>
#include <cstdint>
#include <string>
#include <vector>

namespace biomechanics {

/** Single log entry with timestamp (ms since Unix epoch). */
struct LogEntry {
  int64_t timestamp_ms = 0;
  std::string message;
};

/** Call once at startup so log file is written to repo .cursor/debug.log (not cwd). */
void init_log_path();

/** Append a line to the log (file + in-memory buffer). Thread-unsafe. Adds current time. */
void log(const char* fmt, ...);

/** Get the last N log lines for display (each string is "[timestamp_ms] message"). */
const std::vector<std::string>& get_log_lines();

/** Get a copy of log entries with timestamps (thread-safe; for time-window queries). */
std::vector<LogEntry> get_log_entries();

/** Path where the log file is written (e.g. ".cursor/debug.log"). */
const char* get_log_path();

/** Clear in-memory buffer and optionally the log file. */
void clear_log(bool clear_file = false);

/** Write one NDJSON line to .cursor/debug.log for debug instrumentation. */
void debug_instrument(const char* location, const char* message, const char* hypothesis_id, int data_val = -999);

/** Path to project assets directory (repo root + "assets"), resolved from exe location. Empty if repo not found. */
const char* get_assets_base_path();

}  // namespace biomechanics
