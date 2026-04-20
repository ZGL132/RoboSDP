#pragma once

#include <QString>

namespace RoboSDP::Errors
{

/**
 * @brief 统一公共错误码。
 *
 * 错误码采用英文技术标识，中文说明由辅助函数统一提供，
 * 避免 UI、日志与基础设施层出现分散定义。
 */
enum class ErrorCode
{
    Ok = 0,
    UnknownError,
    InvalidArgument,
    UiStartupFailed,
    ConfigFileNotFound,
    ConfigParseFailed,
    RepositoryNotInitialized,
    RepositoryRootNotFound,
    RepositoryDocumentNotFound,
    RepositoryReadFailed,
    RepositoryWriteFailed,
    JsonFormatInvalid,
    IoError
};

/// 将错误码转换为稳定的英文代码字符串，供日志和调试使用。
QString ToCodeString(ErrorCode errorCode);

/// 返回面向用户与开发人员的中文说明文本。
QString ToChineseMessage(ErrorCode errorCode);

} // namespace RoboSDP::Errors
