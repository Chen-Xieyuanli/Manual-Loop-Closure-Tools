#include "common/runtime_logger.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

namespace {
std::mutex g_logger_mutex;
std::ofstream g_latest_stream;
bool g_logger_initialized = false;
std::string g_latest_path;
} // namespace

void RuntimeLogger::Init(const std::string &file_path) {
  std::lock_guard<std::mutex> lock(g_logger_mutex);
  if (!file_path.empty() && file_path == g_latest_path && g_logger_initialized) {
    return;
  }

  if (g_latest_stream.is_open()) {
    g_latest_stream.close();
  }

  std::filesystem::path target(file_path);
  std::error_code ec;
  std::filesystem::create_directories(target.parent_path(), ec);
  if (ec) {
    return;
  }

  g_latest_stream.open(file_path, std::ios::out | std::ios::trunc);
  if (!g_latest_stream.is_open()) {
    g_logger_initialized = false;
    return;
  }

  g_logger_initialized = true;
  g_latest_path = file_path;

  auto now = std::chrono::system_clock::now();
  auto tt = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
#ifdef _WIN32
  localtime_s(&tm_buf, &tt);
#else
  localtime_r(&tt, &tm_buf);
#endif
  g_latest_stream << "=== MS-Mapping Runtime Log (latest) ===\n";
  g_latest_stream << "Start Time: " << std::put_time(&tm_buf, "%F %T") << "\n";
  g_latest_stream << "Log Path: " << file_path << "\n";
  g_latest_stream << "======================================\n";
  g_latest_stream.flush();
}

void RuntimeLogger::Log(const std::string &level, const char *function,
                        const std::string &message) {
  std::lock_guard<std::mutex> lock(g_logger_mutex);
  if (!g_logger_initialized || !g_latest_stream.is_open()) {
    return;
  }

  std::string sanitized = message;
  for (char &ch : sanitized) {
    unsigned char value = static_cast<unsigned char>(ch);
    if (value == 0) {
      ch = ' ';
    } else if (value < 0x20 && ch != '\n' && ch != '\r' && ch != '\t') {
      ch = ' ';
    }
  }

  auto now = std::chrono::system_clock::now();
  auto tt = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) %
            1000;
  std::tm tm_buf{};
#ifdef _WIN32
  localtime_s(&tm_buf, &tt);
#else
  localtime_r(&tt, &tm_buf);
#endif

  g_latest_stream << std::put_time(&tm_buf, "%F %T") << "."
                  << std::setfill('0') << std::setw(3) << ms.count() << " ["
                  << level << "] (" << function << ") " << sanitized << "\n";
  g_latest_stream.flush();
}

void RuntimeLogger::Snapshot(const std::string &file_path) {
  std::lock_guard<std::mutex> lock(g_logger_mutex);
  if (!g_logger_initialized || !g_latest_stream.is_open()) {
    return;
  }

  g_latest_stream.flush();

  std::filesystem::path target(file_path);
  std::error_code ec;
  std::filesystem::create_directories(target.parent_path(), ec);
  if (ec) {
    return;
  }

  std::ifstream src(g_latest_path, std::ios::in);
  if (!src.is_open()) {
    return;
  }

  std::ofstream dst(file_path, std::ios::out | std::ios::trunc);
  if (!dst.is_open()) {
    return;
  }

  dst << src.rdbuf();
  dst.flush();
}
