#pragma once

#include "core/errors/ErrorCode.h"

#include <QString>

namespace RoboSDP::Errors
{

/**
 * @brief 统一错误信息结构。
 * 该结构用于把错误码、稳定英文编码、中文说明和严重级别收口到同一个对象，
 * 便于日志、配置加载和后续校验结果复用统一错误输出格式。
 */
struct ErrorInfo
{
    ErrorCode code = ErrorCode::Ok;
    QString code_string = QStringLiteral("OK");
    QString message_zh = QString::fromWCharArray(L"\u64CD\u4F5C\u6210\u529F");
    QString severity = QStringLiteral("info");
};

/// 将错误码转换为统一错误信息对象，避免各层重复拼装中文消息。
ErrorInfo ToErrorInfo(
    ErrorCode errorCode,
    const QString& severity = QStringLiteral("error"));

} // namespace RoboSDP::Errors
