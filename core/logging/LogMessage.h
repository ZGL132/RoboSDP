#pragma once

#include "core/errors/ErrorInfo.h"
#include "core/logging/ILogger.h"

#include <QDateTime>
#include <QString>

namespace RoboSDP::Logging
{

/**
 * @brief 统一日志消息对象。
 * 该对象用于在日志门面和底层日志实现之间传递稳定结构，
 * 避免各层重复拼接时间戳、错误信息和上下文字段。
 */
struct LogMessage
{
    QDateTime timestamp = QDateTime::currentDateTime();
    LogLevel level = LogLevel::Info;
    QString message;
    RoboSDP::Errors::ErrorInfo error_info = RoboSDP::Errors::ToErrorInfo(RoboSDP::Errors::ErrorCode::Ok, QStringLiteral("info"));
    LogContext context;
};

} // namespace RoboSDP::Logging
