#include "core/errors/ErrorCode.h"

#include "core/errors/ErrorInfo.h"

namespace RoboSDP::Errors
{

namespace
{

QString Zh(const wchar_t* text)
{
    return QString::fromWCharArray(text);
}

} // namespace

QString ToCodeString(ErrorCode errorCode)
{
    switch (errorCode)
    {
    case ErrorCode::Ok:
        return QStringLiteral("OK");
    case ErrorCode::UnknownError:
        return QStringLiteral("UNKNOWN_ERROR");
    case ErrorCode::InvalidArgument:
        return QStringLiteral("INVALID_ARGUMENT");
    case ErrorCode::UiStartupFailed:
        return QStringLiteral("UI_STARTUP_FAILED");
    case ErrorCode::ConfigFileNotFound:
        return QStringLiteral("CONFIG_FILE_NOT_FOUND");
    case ErrorCode::ConfigParseFailed:
        return QStringLiteral("CONFIG_PARSE_FAILED");
    case ErrorCode::RepositoryNotInitialized:
        return QStringLiteral("REPOSITORY_NOT_INITIALIZED");
    case ErrorCode::RepositoryRootNotFound:
        return QStringLiteral("REPOSITORY_ROOT_NOT_FOUND");
    case ErrorCode::RepositoryDocumentNotFound:
        return QStringLiteral("REPOSITORY_DOCUMENT_NOT_FOUND");
    case ErrorCode::RepositoryReadFailed:
        return QStringLiteral("REPOSITORY_READ_FAILED");
    case ErrorCode::RepositoryWriteFailed:
        return QStringLiteral("REPOSITORY_WRITE_FAILED");
    case ErrorCode::JsonFormatInvalid:
        return QStringLiteral("JSON_FORMAT_INVALID");
    case ErrorCode::IoError:
        return QStringLiteral("IO_ERROR");
    }

    return QStringLiteral("UNKNOWN_ERROR");
}

QString ToChineseMessage(ErrorCode errorCode)
{
    switch (errorCode)
    {
    case ErrorCode::Ok:
        return Zh(L"\u64CD\u4F5C\u6210\u529F");
    case ErrorCode::UnknownError:
        return Zh(L"\u53D1\u751F\u672A\u5206\u7C7B\u9519\u8BEF");
    case ErrorCode::InvalidArgument:
        return Zh(L"\u8F93\u5165\u53C2\u6570\u65E0\u6548");
    case ErrorCode::UiStartupFailed:
        return Zh(L"\u754C\u9762\u542F\u52A8\u5931\u8D25");
    case ErrorCode::ConfigFileNotFound:
        return Zh(L"\u914D\u7F6E\u6587\u4EF6\u4E0D\u5B58\u5728");
    case ErrorCode::ConfigParseFailed:
        return Zh(L"\u914D\u7F6E\u6587\u4EF6\u89E3\u6790\u5931\u8D25");
    case ErrorCode::RepositoryNotInitialized:
        return Zh(L"\u4ED3\u50A8\u5C1A\u672A\u521D\u59CB\u5316\u9879\u76EE\u6839\u76EE\u5F55");
    case ErrorCode::RepositoryRootNotFound:
        return Zh(L"\u9879\u76EE\u6839\u76EE\u5F55\u4E0D\u5B58\u5728");
    case ErrorCode::RepositoryDocumentNotFound:
        return Zh(L"\u76EE\u6807 JSON \u6587\u6863\u4E0D\u5B58\u5728");
    case ErrorCode::RepositoryReadFailed:
        return Zh(L"JSON \u6587\u6863\u8BFB\u53D6\u5931\u8D25");
    case ErrorCode::RepositoryWriteFailed:
        return Zh(L"JSON \u6587\u6863\u5199\u5165\u5931\u8D25");
    case ErrorCode::JsonFormatInvalid:
        return Zh(L"JSON \u683C\u5F0F\u65E0\u6548");
    case ErrorCode::IoError:
        return Zh(L"\u6587\u4EF6\u8BFB\u5199\u5931\u8D25");
    }

    return Zh(L"\u53D1\u751F\u672A\u5206\u7C7B\u9519\u8BEF");
}

ErrorInfo ToErrorInfo(ErrorCode errorCode, const QString& severity)
{
    ErrorInfo info;
    info.code = errorCode;
    info.code_string = ToCodeString(errorCode);
    info.message_zh = ToChineseMessage(errorCode);
    info.severity = severity;
    return info;
}

} // namespace RoboSDP::Errors
