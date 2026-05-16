#pragma once

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace logger {

enum class LogLevel { TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL, OFF };

class Logger {
 public:
  static Logger& GetInstance();

  void Init(const LogLevel _console_level = LogLevel::INFO,
            const LogLevel _file_level = LogLevel::DEBUG,
            const std::string& _filename = "",
            const size_t _max_size = 5 * 1024 * 1024,
            const size_t _max_files = 3);

  void AddSink(spdlog::sink_ptr _sink, const LogLevel _level = LogLevel::INFO);

  void SetLevel(const LogLevel _level);

  std::shared_ptr<spdlog::logger> GetLogger();

 private:
  Logger();
  ~Logger() = default;
  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;

  spdlog::level::level_enum ConvertLevel(LogLevel _level);

 private:
  std::vector<spdlog::sink_ptr> sinks_;
  std::shared_ptr<spdlog::logger> logger_;
};

#define LOG_TRACE(...)                                                      \
  SPDLOG_LOGGER_TRACE(logger::Logger::GetInstance().GetLogger(), \
                      __VA_ARGS__)
#define LOG_DEBUG(...)                                                      \
  SPDLOG_LOGGER_DEBUG(logger::Logger::GetInstance().GetLogger(), \
                      __VA_ARGS__)
#define LOG_INFO(...)                                                      \
  SPDLOG_LOGGER_INFO(logger::Logger::GetInstance().GetLogger(), \
                     __VA_ARGS__)
#define LOG_WARN(...)                                                      \
  SPDLOG_LOGGER_WARN(logger::Logger::GetInstance().GetLogger(), \
                     __VA_ARGS__)
#define LOG_ERROR(...)                                                      \
  SPDLOG_LOGGER_ERROR(logger::Logger::GetInstance().GetLogger(), \
                      __VA_ARGS__)
#define LOG_CRITICAL(...)                                                      \
  SPDLOG_LOGGER_CRITICAL(logger::Logger::GetInstance().GetLogger(), \
                         __VA_ARGS__)

}  // namespace logger
