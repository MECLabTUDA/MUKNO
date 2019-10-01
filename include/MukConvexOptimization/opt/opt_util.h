#pragma once

namespace gris
{
  namespace opt
  {
    enum LogLevel 
    {
      LevelFatal = 0,
      LevelError = 1,
      LevelWarn = 2,
      LevelInfo = 3,
      LevelDebug = 4,
      LevelTrace = 5
    };

    static LogLevel gLogLevel = LevelInfo;
    inline LogLevel GetLogLevel() {return gLogLevel;}

#define FATAL_PREFIX "\x1b[31m[FATAL] "
#define ERROR_PREFIX "\x1b[31m[ERROR] "
#define WARN_PREFIX "\x1b[33m[WARN] "
#define INFO_PREFIX "[INFO] "
#define DEBUG_PREFIX "\x1b[32m[DEBUG] "
#define TRACE_PREFIX "\x1b[34m[TRACE] "
#define LOG_SUFFIX "\x1b[0m\n"

#define LOG_FATAL(msg, ...) if (GetLogLevel() >= LevelFatal) {std::printf(FATAL_PREFIX); std::printf(msg, ##__VA_ARGS__); std::printf(LOG_SUFFIX);}
#define LOG_ERROR(msg, ...) if (GetLogLevel() >= LevelError) {std::printf(ERROR_PREFIX); std::printf(msg, ##__VA_ARGS__); std::printf(LOG_SUFFIX);}
#define LOG_WARN(msg, ...)  if (GetLogLevel() >= LevelWarn)  {std::printf(WARN_PREFIX);  std::printf(msg, ##__VA_ARGS__); std::printf(LOG_SUFFIX);}
#define LOG_INFO(msg, ...)  if (GetLogLevel() >= LevelInfo)  {std::printf(INFO_PREFIX);  std::printf(msg, ##__VA_ARGS__); std::printf(LOG_SUFFIX);}
#define LOG_DEBUG(msg, ...) if (GetLogLevel() >= LevelDebug) {std::printf(DEBUG_PREFIX); std::printf(msg, ##__VA_ARGS__); std::printf(LOG_SUFFIX);}
#define LOG_TRACE(msg, ...) if (GetLogLevel() >= LevelTrace) {std::printf(TRACE_PREFIX); std::printf(msg, ##__VA_ARGS__); std::printf(LOG_SUFFIX);}
    
  }
}
