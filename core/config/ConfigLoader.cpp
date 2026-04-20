#include "core/config/ConfigLoader.h"

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>

namespace RoboSDP::Config
{

namespace
{

QString Zh(const wchar_t* text)
{
    return QString::fromWCharArray(text);
}

} // namespace

ConfigLoadResult ConfigLoader::LoadDefaults() const
{
    ConfigLoadResult result;
    result.errorInfo = RoboSDP::Errors::ToErrorInfo(
        RoboSDP::Errors::ErrorCode::Ok,
        QStringLiteral("info"));
    result.message = Zh(L"\u5DF2\u52A0\u8F7D\u9ED8\u8BA4\u914D\u7F6E");
    return result;
}

ConfigLoadResult ConfigLoader::LoadFromFile(const QString& filePath) const
{
    QFile file(filePath);
    if (!file.exists())
    {
        return {
            RoboSDP::Errors::ErrorCode::ConfigFileNotFound,
            RoboSDP::Errors::ToErrorInfo(
                RoboSDP::Errors::ErrorCode::ConfigFileNotFound,
                QStringLiteral("error")),
            Zh(L"\u914D\u7F6E\u6587\u4EF6\u4E0D\u5B58\u5728\uFF1A%1").arg(filePath),
            {}
        };
    }

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        return {
            RoboSDP::Errors::ErrorCode::IoError,
            RoboSDP::Errors::ToErrorInfo(
                RoboSDP::Errors::ErrorCode::IoError,
                QStringLiteral("error")),
            Zh(L"\u914D\u7F6E\u6587\u4EF6\u6253\u5F00\u5931\u8D25\uFF1A%1").arg(filePath),
            {}
        };
    }

    const QByteArray rawData = file.readAll();
    file.close();

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(rawData, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        return {
            RoboSDP::Errors::ErrorCode::ConfigParseFailed,
            RoboSDP::Errors::ToErrorInfo(
                RoboSDP::Errors::ErrorCode::ConfigParseFailed,
                QStringLiteral("error")),
            Zh(L"\u914D\u7F6E\u6587\u4EF6\u89E3\u6790\u5931\u8D25\uFF1A%1").arg(parseError.errorString()),
            {}
        };
    }

    ConfigLoadResult result;
    result.errorInfo = RoboSDP::Errors::ToErrorInfo(
        RoboSDP::Errors::ErrorCode::Ok,
        QStringLiteral("info"));
    result.message = Zh(L"\u5DF2\u4ECE\u6587\u4EF6\u52A0\u8F7D\u914D\u7F6E");
    result.config = ParseConfigObject(document.object());
    return result;
}

AppConfig ConfigLoader::ParseConfigObject(const QJsonObject& jsonObject) const
{
    AppConfig config;

    if (jsonObject.contains(QStringLiteral("language")) && jsonObject.value(QStringLiteral("language")).isString())
    {
        config.language = jsonObject.value(QStringLiteral("language")).toString();
    }

    if (jsonObject.contains(QStringLiteral("theme")) && jsonObject.value(QStringLiteral("theme")).isString())
    {
        config.theme = jsonObject.value(QStringLiteral("theme")).toString();
    }

    if (jsonObject.contains(QStringLiteral("defaultLogLevel")) &&
        jsonObject.value(QStringLiteral("defaultLogLevel")).isString())
    {
        config.defaultLogLevel = jsonObject.value(QStringLiteral("defaultLogLevel")).toString();
    }

    return config;
}

} // namespace RoboSDP::Config
