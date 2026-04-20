#include "core/config/AppConfigLoader.h"
#include "core/errors/ErrorInfo.h"
#include "core/logging/Logger.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QTextStream>

namespace
{

/**
 * @brief 测试用内存日志后端。
 * 该后端只记录最后一条日志，便于验证统一日志门面是否把信息正确转发出去。
 */
class MemoryLogger final : public RoboSDP::Logging::ILogger
{
public:
    void Log(
        RoboSDP::Logging::LogLevel level,
        const QString& message,
        RoboSDP::Errors::ErrorCode errorCode,
        const RoboSDP::Logging::LogContext& context) override
    {
        lastLevel = level;
        lastMessage = message;
        lastErrorCode = errorCode;
        lastContext = context;
    }

    RoboSDP::Logging::LogLevel lastLevel = RoboSDP::Logging::LogLevel::Info;
    QString lastMessage;
    RoboSDP::Errors::ErrorCode lastErrorCode = RoboSDP::Errors::ErrorCode::Ok;
    RoboSDP::Logging::LogContext lastContext;
};

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    const auto errorInfo = RoboSDP::Errors::ToErrorInfo(
        RoboSDP::Errors::ErrorCode::ConfigFileNotFound,
        QStringLiteral("error"));
    if (errorInfo.code_string != QStringLiteral("CONFIG_FILE_NOT_FOUND") ||
        errorInfo.message_zh.isEmpty() ||
        errorInfo.severity != QStringLiteral("error"))
    {
        return 1;
    }

    MemoryLogger memoryLogger;
    RoboSDP::Logging::Logger::Instance().SetBackend(&memoryLogger);
    RoboSDP::Logging::Logger::Instance().Warning(
        QStringLiteral("core smoke warning"),
        RoboSDP::Errors::ErrorCode::InvalidArgument,
        {QStringLiteral("core"), QStringLiteral("smoke"), QStringLiteral("test")});

    if (memoryLogger.lastMessage != QStringLiteral("core smoke warning") ||
        memoryLogger.lastErrorCode != RoboSDP::Errors::ErrorCode::InvalidArgument ||
        memoryLogger.lastContext.moduleName != QStringLiteral("core"))
    {
        RoboSDP::Logging::Logger::Instance().ResetToDefaultBackend();
        return 2;
    }
    RoboSDP::Logging::Logger::Instance().ResetToDefaultBackend();

    const QString appRoot = QDir::current().absoluteFilePath(QStringLiteral("core-infra-smoke-project"));
    QDir().mkpath(QDir(appRoot).filePath(QStringLiteral("config")));
    QFile configFile(QDir(appRoot).filePath(QStringLiteral("config/app-config.json")));
    if (!configFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        return 3;
    }

    QTextStream out(&configFile);
    out << "{\n"
        << "  \"language\": \"en_US\",\n"
        << "  \"theme\": \"dark\",\n"
        << "  \"defaultLogLevel\": \"WARNING\"\n"
        << "}\n";
    configFile.close();

    RoboSDP::Config::AppConfigLoader configLoader;
    if (configLoader.DefaultAppConfigRelativePath() != QStringLiteral("config/app-config.json"))
    {
        return 4;
    }

    const auto loadResult = configLoader.LoadFromApplicationRoot(appRoot);
    if (!loadResult.IsSuccess() ||
        loadResult.config.language != QStringLiteral("en_US") ||
        loadResult.config.theme != QStringLiteral("dark") ||
        loadResult.config.defaultLogLevel != QStringLiteral("WARNING"))
    {
        return 5;
    }

    const auto fallbackResult = configLoader.LoadFromApplicationRoot(QString());
    if (fallbackResult.message.isEmpty() || fallbackResult.config.language.isEmpty())
    {
        return 6;
    }

    return 0;
}
