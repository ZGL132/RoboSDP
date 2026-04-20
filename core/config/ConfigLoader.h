#pragma once

#include "core/errors/ErrorCode.h"
#include "core/errors/ErrorInfo.h"

#include <QString>

class QJsonObject;

namespace RoboSDP::Config
{

/**
 * @brief 应用配置对象。
 * 本轮只保留第一阶段稳定使用的最小公共配置字段，避免扩展成复杂配置中心。
 */
struct AppConfig
{
    QString language = QStringLiteral("zh_CN");
    QString theme = QStringLiteral("light");
    QString defaultLogLevel = QStringLiteral("INFO");
};

/**
 * @brief 配置加载结果。
 * 统一返回错误码、统一错误信息和配置对象，避免调用方重复拼装错误消息。
 */
struct ConfigLoadResult
{
    RoboSDP::Errors::ErrorCode errorCode = RoboSDP::Errors::ErrorCode::Ok;
    RoboSDP::Errors::ErrorInfo errorInfo = RoboSDP::Errors::ToErrorInfo(
        RoboSDP::Errors::ErrorCode::Ok,
        QStringLiteral("info"));
    QString message = QString::fromWCharArray(L"\u6210\u529F");
    AppConfig config;

    bool IsSuccess() const
    {
        return errorCode == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/**
 * @brief 集中配置加载器。
 * 当前只负责默认配置和 JSON 文件加载，不承担复杂偏好设置策略。
 */
class ConfigLoader
{
public:
    /// 加载内置默认配置，作为应用启动时的最小兜底路径。
    ConfigLoadResult LoadDefaults() const;

    /// 从 JSON 文件加载配置；若失败则返回明确错误码和中文说明。
    ConfigLoadResult LoadFromFile(const QString& filePath) const;

private:
    /// 将 JSON 对象映射为统一配置对象，避免调用方重复解析字段。
    AppConfig ParseConfigObject(const QJsonObject& jsonObject) const;
};

} // namespace RoboSDP::Config
