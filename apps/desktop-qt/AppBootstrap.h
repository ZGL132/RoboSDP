#pragma once

#include "core/config/ConfigLoader.h"
#include "core/errors/ErrorCode.h"
#include "core/logging/ConsoleLogger.h"

#include <QString>
#include <memory>

class QWidget;

namespace RoboSDP::Desktop
{

class MainWindow;

/**
 * @brief 桌面端启动结果。
 * 统一封装启动阶段的错误码与中文消息，避免入口函数重复拼装返回逻辑。
 */
struct AppBootstrapResult
{
    RoboSDP::Errors::ErrorCode errorCode = RoboSDP::Errors::ErrorCode::Ok;
    QString message = QStringLiteral("桌面端启动成功");

    bool IsSuccess() const
    {
        return errorCode == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/**
 * @brief 桌面端最小启动装配器。
 * 当前只负责图形格式、应用元信息、默认配置加载与主窗口创建，
 * 为后续继续接入 repository、service 和后台适配层预留统一入口。
 */
class AppBootstrap
{
public:
    /// 配置 Qt/VTK 共用的默认图形格式，必须在 QApplication 创建前调用。
    void ConfigureGraphicsSurfaceFormat() const;

    /// 执行桌面端最小启动装配，不承担业务模块初始化职责。
    AppBootstrapResult Initialize();

    /// 基于已完成的启动上下文创建主窗口，避免 main.cpp 直接拼装 UI 对象。
    std::unique_ptr<MainWindow> CreateMainWindow(QWidget* parent = nullptr);

private:
    /// 统一写入应用元信息，避免入口函数散落基础设置。
    void ApplyApplicationMetadata() const;

private:
    RoboSDP::Logging::ConsoleLogger m_logger;
    RoboSDP::Config::ConfigLoader m_configLoader;
    RoboSDP::Config::AppConfig m_appConfig;
    bool m_isInitialized = false;
};

} // namespace RoboSDP::Desktop
