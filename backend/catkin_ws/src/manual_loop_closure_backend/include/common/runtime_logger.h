#pragma once

#include <string>

class RuntimeLogger {
public:
  static void Init(const std::string &file_path);
  static void Log(const std::string &level, const char *function,
                  const std::string &message);
  static void Snapshot(const std::string &file_path);
};

#include <sstream>

#define RUNTIME_LOG_STREAM(level, message)                                     \
  do {                                                                         \
    std::ostringstream _runtime_log_stream__;                                  \
    _runtime_log_stream__ << message;                                          \
    RuntimeLogger::Log(level, __FUNCTION__, _runtime_log_stream__.str());      \
  } while (0)

#define LOG_INFO_STREAM(message)                                               \
  do {                                                                         \
    ROS_INFO_STREAM(message);                                                  \
    RUNTIME_LOG_STREAM("INFO", message);                                       \
  } while (0)

#define LOG_WARN_STREAM(message)                                               \
  do {                                                                         \
    ROS_WARN_STREAM(message);                                                  \
    RUNTIME_LOG_STREAM("WARN", message);                                       \
  } while (0)

#define LOG_INFO_STREAM_ONCE(message)                                         \
  do {                                                                        \
    ROS_INFO_STREAM_ONCE(message);                                            \
    RUNTIME_LOG_STREAM("INFO", message);                                      \
  } while (0)

#define LOG_WARN_STREAM_ONCE(message)                                         \
  do {                                                                        \
    ROS_WARN_STREAM_ONCE(message);                                            \
    RUNTIME_LOG_STREAM("WARN", message);                                      \
  } while (0)

#define LOG_DEBUG_STREAM(message)                                              \
  do {                                                                         \
    ROS_DEBUG_STREAM(message);                                                 \
    RUNTIME_LOG_STREAM("DEBUG", message);                                      \
  } while (0)

#define LOG_ERROR_STREAM(message)                                              \
  do {                                                                         \
    ROS_ERROR_STREAM(message);                                                 \
    RUNTIME_LOG_STREAM("ERROR", message);                                      \
  } while (0)

#define LOG_FATAL_STREAM(message)                                              \
  do {                                                                         \
    ROS_FATAL_STREAM(message);                                                 \
    RUNTIME_LOG_STREAM("FATAL", message);                                      \
  } while (0)

#define LOG_INFO_STREAM_THROTTLE(period, message)                              \
  do {                                                                         \
    ROS_INFO_STREAM_THROTTLE(period, message);                                 \
    RUNTIME_LOG_STREAM("INFO", message);                                       \
  } while (0)

#define LOG_WARN_STREAM_THROTTLE(period, message)                              \
  do {                                                                         \
    ROS_WARN_STREAM_THROTTLE(period, message);                                 \
    RUNTIME_LOG_STREAM("WARN", message);                                       \
  } while (0)

#define LOG_ERROR_STREAM_THROTTLE(period, message)                             \
  do {                                                                         \
    ROS_ERROR_STREAM_THROTTLE(period, message);                                \
    RUNTIME_LOG_STREAM("ERROR", message);                                      \
  } while (0)
