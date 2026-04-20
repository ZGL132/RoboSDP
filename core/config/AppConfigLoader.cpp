#include "core/config/AppConfigLoader.h"

#include "core/errors/ErrorInfo.h"
#include "core/logging/Logger.h"

#include <QDir>
#include <QFileInfo>

namespace RoboSDP::Config
{

namespace
{

QString Zh(const wchar_t* text)
{
    return QString::fromWCharArray(text);
}

} // namespace

QString AppConfigLoader::DefaultAppConfigRelativePath() const
{
    return QStringLiteral("config/app-config.json");
}

ConfigLoadResult AppConfigLoader::LoadFromApplicationRoot(const QString& applicationRootPath) const
{
    if (applicationRootPath.trimmed().isEmpty())
    {
        ConfigLoadResult result = m_configLoader.LoadDefaults();
        result.errorCode = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.errorInfo = RoboSDP::Errors::ToErrorInfo(
            RoboSDP::Errors::ErrorCode::InvalidArgument,
            QStringLiteral("warning"));
        result.message = Zh(L"\u5E94\u7528\u6839\u76EE\u5F55\u4E3A\u7A7A\uFF0C\u5DF2\u56DE\u9000\u5230\u9ED8\u8BA4\u914D\u7F6E");
        RoboSDP::Logging::Logger::Instance().Warning(
            result.message,
            result.errorCode,
            {QStringLiteral("config"), QStringLiteral("load_app_config"), applicationRootPath});
        return result;
    }

    const QString configFilePath =
        QDir(applicationRootPath).filePath(DefaultAppConfigRelativePath());
    if (!QFileInfo::exists(configFilePath))
    {
        // 当前阶段把缺少应用配置文件视为可接受情况，直接回退默认配置。
        ConfigLoadResult result = m_configLoader.LoadDefaults();
        result.errorCode = RoboSDP::Errors::ErrorCode::Ok;
        result.errorInfo = RoboSDP::Errors::ToErrorInfo(
            RoboSDP::Errors::ErrorCode::Ok,
            QStringLiteral("info"));
        result.message = Zh(L"\u672A\u53D1\u73B0\u5E94\u7528\u914D\u7F6E\u6587\u4EF6\uFF0C\u5DF2\u4F7F\u7528\u9ED8\u8BA4\u914D\u7F6E");
        RoboSDP::Logging::Logger::Instance().Info(
            result.message,
            result.errorCode,
            {QStringLiteral("config"), QStringLiteral("load_app_config"), configFilePath});
        return result;
    }

    RoboSDP::Logging::Logger::Instance().Info(
        Zh(L"\u5F00\u59CB\u8BFB\u53D6\u5E94\u7528\u914D\u7F6E\u6587\u4EF6"),
        RoboSDP::Errors::ErrorCode::Ok,
        {QStringLiteral("config"), QStringLiteral("load_app_config"), configFilePath});
    return m_configLoader.LoadFromFile(configFilePath);
}

} // namespace RoboSDP::Config
