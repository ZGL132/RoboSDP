#include "core/logging/Logger.h"

namespace RoboSDP::Logging
{

Logger& Logger::Instance()
{
    static Logger logger;
    return logger;
}

Logger::Logger()
    : m_backend(&m_defaultBackend)
{
}

void Logger::SetBackend(ILogger* backend)
{
    m_backend = (backend != nullptr) ? backend : &m_defaultBackend;
}

void Logger::ResetToDefaultBackend()
{
    m_backend = &m_defaultBackend;
}

void Logger::Log(const LogMessage& logMessage)
{
    // 统一门面只负责把结构化日志转发给当前后端，不在这里叠加业务逻辑。
    ActiveBackend()->Log(
        logMessage.level,
        logMessage.message,
        logMessage.error_info.code,
        logMessage.context);
}

void Logger::Debug(
    const QString& message,
    RoboSDP::Errors::ErrorCode errorCode,
    const LogContext& context)
{
    LogWithLevel(LogLevel::Debug, message, errorCode, context);
}

void Logger::Info(
    const QString& message,
    RoboSDP::Errors::ErrorCode errorCode,
    const LogContext& context)
{
    LogWithLevel(LogLevel::Info, message, errorCode, context);
}

void Logger::Warning(
    const QString& message,
    RoboSDP::Errors::ErrorCode errorCode,
    const LogContext& context)
{
    LogWithLevel(LogLevel::Warning, message, errorCode, context);
}

void Logger::Error(
    const QString& message,
    RoboSDP::Errors::ErrorCode errorCode,
    const LogContext& context)
{
    LogWithLevel(LogLevel::Error, message, errorCode, context);
}

ILogger* Logger::ActiveBackend() const
{
    return (m_backend != nullptr) ? m_backend : const_cast<ConsoleLogger*>(&m_defaultBackend);
}

void Logger::LogWithLevel(
    LogLevel level,
    const QString& message,
    RoboSDP::Errors::ErrorCode errorCode,
    const LogContext& context)
{
    LogMessage logMessage;
    logMessage.level = level;
    logMessage.message = message;
    logMessage.error_info = RoboSDP::Errors::ToErrorInfo(
        errorCode,
        level == LogLevel::Error ? QStringLiteral("error") :
        level == LogLevel::Warning ? QStringLiteral("warning") :
        QStringLiteral("info"));
    logMessage.context = context;
    Log(logMessage);
}

} // namespace RoboSDP::Logging
