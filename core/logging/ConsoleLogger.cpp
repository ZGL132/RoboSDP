#include "core/logging/ConsoleLogger.h"

#include "core/errors/ErrorInfo.h"

#include <QDateTime>
#include <QDebug>
#include <QStringList>

namespace RoboSDP::Logging
{

namespace
{

QString SeverityFromLevel(LogLevel level)
{
    switch (level)
    {
    case LogLevel::Error:
        return QStringLiteral("error");
    case LogLevel::Warning:
        return QStringLiteral("warning");
    case LogLevel::Debug:
    case LogLevel::Info:
        return QStringLiteral("info");
    }

    return QStringLiteral("info");
}

} // namespace

QString ToString(LogLevel logLevel)
{
    switch (logLevel)
    {
    case LogLevel::Debug:
        return QStringLiteral("DEBUG");
    case LogLevel::Info:
        return QStringLiteral("INFO");
    case LogLevel::Warning:
        return QStringLiteral("WARNING");
    case LogLevel::Error:
        return QStringLiteral("ERROR");
    }

    return QStringLiteral("INFO");
}

namespace
{

QString BuildLogLine(
    LogLevel level,
    const QString& message,
    RoboSDP::Errors::ErrorCode errorCode,
    const LogContext& context)
{
    const RoboSDP::Errors::ErrorInfo errorInfo = RoboSDP::Errors::ToErrorInfo(
        errorCode,
        SeverityFromLevel(level));

    QStringList parts;
    parts << QStringLiteral("[%1]").arg(QDateTime::currentDateTime().toString(Qt::ISODate))
          << QStringLiteral("[%1]").arg(ToString(level))
          << QStringLiteral("[%1]").arg(errorInfo.code_string);

    if (!context.moduleName.isEmpty())
    {
        parts << QStringLiteral("[module=%1]").arg(context.moduleName);
    }

    if (!context.actionName.isEmpty())
    {
        parts << QStringLiteral("[action=%1]").arg(context.actionName);
    }

    if (!context.locationHint.isEmpty())
    {
        parts << QStringLiteral("[location=%1]").arg(context.locationHint);
    }

    if (errorCode != RoboSDP::Errors::ErrorCode::Ok)
    {
        // 统一补上中文错误说明，便于控制台日志和后续文件日志保持同一口径。
        parts << QStringLiteral("[message_zh=%1]").arg(errorInfo.message_zh);
    }

    parts << message;
    return parts.join(QStringLiteral(" "));
}

} // namespace

void ConsoleLogger::Log(
    LogLevel level,
    const QString& message,
    RoboSDP::Errors::ErrorCode errorCode,
    const LogContext& context)
{
    const QString logLine = BuildLogLine(level, message, errorCode, context);

    switch (level)
    {
    case LogLevel::Debug:
    case LogLevel::Info:
        qInfo().noquote() << logLine;
        break;
    case LogLevel::Warning:
        qWarning().noquote() << logLine;
        break;
    case LogLevel::Error:
        qCritical().noquote() << logLine;
        break;
    }
}

} // namespace RoboSDP::Logging
