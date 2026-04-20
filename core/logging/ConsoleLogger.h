#pragma once

#include "core/logging/ILogger.h"

namespace RoboSDP::Logging
{

/**
 * @brief 控制台日志实现。
 * 当前先提供最小可用实现，后续可扩展为文件输出和桌面端日志面板桥接。
 */
class ConsoleLogger final : public ILogger
{
public:
    void Log(
        LogLevel level,
        const QString& message,
        RoboSDP::Errors::ErrorCode errorCode = RoboSDP::Errors::ErrorCode::Ok,
        const LogContext& context = {}) override;
};

} // namespace RoboSDP::Logging
