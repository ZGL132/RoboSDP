#pragma once

#include "core/logging/ConsoleLogger.h"
#include "core/logging/LogMessage.h"

namespace RoboSDP::Logging
{

/**
 * @brief 统一日志门面。
 * 本轮不重写现有业务模块，只提供一个可复用的统一入口，
 * 让后续模块逐步通过该门面输出结构化日志。
 */
class Logger final
{
public:
    static Logger& Instance();

    /// 设置外部日志实现，便于桌面端、测试或文件日志后续接入。
    void SetBackend(ILogger* backend);

    /// 恢复到默认控制台日志实现，保证没有外部注入时仍可工作。
    void ResetToDefaultBackend();

    /// 输出一条完整日志消息对象，作为统一收口入口。
    void Log(const LogMessage& logMessage);

    /// 便捷输出调试日志。
    void Debug(
        const QString& message,
        RoboSDP::Errors::ErrorCode errorCode = RoboSDP::Errors::ErrorCode::Ok,
        const LogContext& context = {});

    /// 便捷输出信息日志。
    void Info(
        const QString& message,
        RoboSDP::Errors::ErrorCode errorCode = RoboSDP::Errors::ErrorCode::Ok,
        const LogContext& context = {});

    /// 便捷输出警告日志。
    void Warning(
        const QString& message,
        RoboSDP::Errors::ErrorCode errorCode = RoboSDP::Errors::ErrorCode::Ok,
        const LogContext& context = {});

    /// 便捷输出错误日志。
    void Error(
        const QString& message,
        RoboSDP::Errors::ErrorCode errorCode = RoboSDP::Errors::ErrorCode::UnknownError,
        const LogContext& context = {});

private:
    Logger();

    ILogger* ActiveBackend() const;
    void LogWithLevel(
        LogLevel level,
        const QString& message,
        RoboSDP::Errors::ErrorCode errorCode,
        const LogContext& context);

    ConsoleLogger m_defaultBackend;
    ILogger* m_backend = nullptr;
};

} // namespace RoboSDP::Logging
