#include "logger.hpp"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace logger {

Logger& Logger::GetInstance() {
  static Logger instance;
  return instance;
}

Logger::Logger() : logger_(nullptr) {}

void Logger::Init(const LogLevel _console_level, const LogLevel _file_level,
                  const std::string& _filename, const size_t _max_size,
                  const size_t _max_files) {
  sinks_.clear();

  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(ConvertLevel(_console_level));
  console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%l] [%s:%#]%$ %v");
  sinks_.push_back(console_sink);

  if (!_filename.empty()) {
    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        _filename, _max_size, _max_files);
    file_sink->set_level(ConvertLevel(_file_level));
    file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%s:%#] %v");
    sinks_.push_back(file_sink);
  }

  logger_ = std::make_shared<spdlog::logger>("hy_common", sinks_.begin(),
                                             sinks_.end());
  logger_->set_level(spdlog::level::trace);
  spdlog::register_logger(logger_);
}

void Logger::AddSink(spdlog::sink_ptr _sink, const LogLevel _level) {
  if (!_sink) return;
  _sink->set_level(ConvertLevel(_level));
  sinks_.push_back(_sink);
  if (logger_) {
    logger_->sinks().push_back(_sink);
  }
}

void Logger::SetLevel(const LogLevel _level) {
  if (!logger_) return;
  logger_->set_level(ConvertLevel(_level));
}

std::shared_ptr<spdlog::logger> Logger::GetLogger() {
  if (!logger_) {
    Init();
  }
  return logger_;
}

spdlog::level::level_enum Logger::ConvertLevel(LogLevel _level) {
  switch (_level) {
    case LogLevel::TRACE:
      return spdlog::level::trace;
    case LogLevel::DEBUG:
      return spdlog::level::debug;
    case LogLevel::INFO:
      return spdlog::level::info;
    case LogLevel::WARN:
      return spdlog::level::warn;
    case LogLevel::ERROR:
      return spdlog::level::err;
    case LogLevel::CRITICAL:
      return spdlog::level::critical;
    case LogLevel::OFF:
    default:
      return spdlog::level::off;
  }
}

}  // namespace logger
