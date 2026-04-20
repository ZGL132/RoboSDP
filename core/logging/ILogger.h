#pragma once

#include "core/errors/ErrorCode.h"

#include <QString>

namespace RoboSDP::Logging
{

/**
 * @brief 日志级别枚举。
 * 当前先统一最小日志等级口径，供控制台日志、文件日志和测试日志共用。
 */
enum class LogLevel
{
    Debug = 0,
    Info,
    Warning,
    Error
};

/**
 * @brief 结构化日志上下文。
 * 该结构用于携带模块、动作和位置提示，避免日志文本和上下文信息耦合。
 */
struct LogContext
{
    QString moduleName;
    QString actionName;
    QString locationHint;
};

/// 将日志级别转换为稳定字符串，供输出和筛选使用。
QString ToString(LogLevel logLevel);

/**
 * @brief 统一日志接口。
 * 所有基础设施层和后续业务层都应依赖该接口，而不是直接输出裸字符串。
 */
class ILogger
{
public:
    virtual ~ILogger() = default;

    /// 输出一条结构化日志。
    virtual void Log(
        LogLevel level,
        const QString& message,
        RoboSDP::Errors::ErrorCode errorCode = RoboSDP::Errors::ErrorCode::Ok,
        const LogContext& context = {}) = 0;
};

} // namespace RoboSDP::Logging
