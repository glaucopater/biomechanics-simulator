#pragma once

#include <cstdarg>
#include <string>
#include <vector>

namespace biomechanics {

/** Call once at startup so log file is written to repo .cursor/debug.log (not cwd). */
void init_log_path();

/** Append a line to the log (file + in-memory buffer). Thread-unsafe. */
void log(const char* fmt, ...);

/** Get the last N log lines for display (e.g. ImGui). */
const std::vector<std::string>& get_log_lines();

/** Path where the log file is written (e.g. ".cursor/debug.log"). */
const char* get_log_path();

/** Clear in-memory buffer and optionally the log file. */
void clear_log(bool clear_file = false);

}  // namespace biomechanics
