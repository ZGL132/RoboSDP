#pragma once

#include <QString>

namespace RoboSDP::Schema
{

/// 统一 validator 与 schema 检查使用的严重级别。
enum class ValidationSeverity
{
    Info = 0,
    Warning,
    Error
};

/// 将严重级别转成稳定字符串，便于测试、日志和后续 UI 复用。
inline QString ToString(ValidationSeverity severity)
{
    switch (severity)
    {
    case ValidationSeverity::Info:
        return QStringLiteral("INFO");
    case ValidationSeverity::Warning:
        return QStringLiteral("WARNING");
    case ValidationSeverity::Error:
        return QStringLiteral("ERROR");
    }

    return QStringLiteral("ERROR");
}

/**
 * @brief 统一错误输出结构。
 *
 * 本轮按约束统一为：
 * 1. field：字段路径或文档路径；
 * 2. code：稳定错误码；
 * 3. message_zh：中文说明；
 * 4. severity：严重级别。
 */
struct ValidationMessage
{
    QString field;
    QString code;
    QString message_zh;
    ValidationSeverity severity = ValidationSeverity::Error;
};

} // namespace RoboSDP::Schema
