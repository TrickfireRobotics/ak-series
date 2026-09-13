#ifndef __AK_SERIES_LOGGING
#pragma once
#ifndef LOGGING_LEVEL
#define LOGGING_LEVEL 3
#endif
#include <cstdio>
#include <cstring>
#include <filesystem>

enum class LoggingLevel : uint8_t { Debug = 0, Info, Warning, Error, Critical, Unknown };

namespace {
constexpr const char *enumToStr(LoggingLevel l) {
  switch (l) {
  case (LoggingLevel::Debug):
    return "[Debug]: ";
  case (LoggingLevel::Info):
    return "[Info]: ";
  case (LoggingLevel::Warning):
    return "[Warning]: ";
  case (LoggingLevel::Error):
    return "[Error]: ";
  case (LoggingLevel::Critical):
    return "[Critical]: ";
  default:
    return "[Unknown]: ";
  };
}
}; // namespace

// Quiet dumps error and above, Verbose dumps warning and above
// VV dumps Info and above VVV dumps debug and above
namespace Logging {
#define Quiet 3
#define Verbose 2
#define VeryVerbose 1
#define VeryVeryVeryVerbose 0

}; // namespace Logging
class Logger {
  static inline FILE *m_file;
  // Logger will manage itself
  explicit Logger(const char *filename = "stderr") {
    if (std::strcmp(filename, "stderr") == 0) {
      Logger::m_file = stderr;
      return;
    }
    auto res = std::fopen(filename, "r");
    if (res == NULL) {
      std::fprintf(stderr, "file passed  doesnt exist, defaulting to `stderr`");
      Logger::m_file = stderr;
      return;
    }

    Logger::m_file = std::fopen(filename, "w");
  }

  static inline Logger *activeLogger{nullptr};

public:
  // By defualt will dump to stderr of current shell, however will write to logging file as needed
  template <typename... Args>
  static const void log(LoggingLevel level, const char *error, Args &&...args) {
    if (activeLogger == nullptr) {
      activeLogger = new Logger{};
    }

    if (static_cast<uint8_t>(level) >= LOGGING_LEVEL) {
      return;
    }

    std::fprintf(Logger::m_file, "%s", enumToStr(level));
    std::fprintf(Logger::m_file, error, std::forward<Args>(args)...);
    std::fprintf(Logger::m_file, "\n");
  };
};
#endif
