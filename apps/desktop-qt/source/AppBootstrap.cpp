#include "apps/desktop-qt/AppBootstrap.h"

#include "apps/desktop-qt/MainWindow.h"

#include <QCoreApplication>
#include <QSurfaceFormat>

#if defined(ROBOSDP_HAVE_VTK)
#include <QVTKOpenGLNativeWidget.h>
#endif

namespace RoboSDP::Desktop
{

void AppBootstrap::ConfigureGraphicsSurfaceFormat() const
{
#if defined(ROBOSDP_HAVE_VTK)
    // 若当前构建已接入 VTK，则先统一 Qt/VTK 共用的默认 OpenGL 格式。
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
#endif
}

AppBootstrapResult AppBootstrap::Initialize()
{
    ApplyApplicationMetadata();

    const RoboSDP::Config::ConfigLoadResult defaultConfigResult = m_configLoader.LoadDefaults();
    if (!defaultConfigResult.IsSuccess())
    {
        m_logger.Log(
            RoboSDP::Logging::LogLevel::Error,
            QStringLiteral("默认配置加载失败，桌面端启动终止。"),
            defaultConfigResult.errorCode,
            {QStringLiteral("desktop-bootstrap"), QStringLiteral("load-default-config"), QStringLiteral("Initialize")});
        return {defaultConfigResult.errorCode, defaultConfigResult.message};
    }

    m_appConfig = defaultConfigResult.config;
    m_isInitialized = true;

    m_logger.Log(
        RoboSDP::Logging::LogLevel::Info,
        QStringLiteral("桌面端启动装配完成，当前语言=%1，主题=%2。")
            .arg(m_appConfig.language, m_appConfig.theme),
        RoboSDP::Errors::ErrorCode::Ok,
        {QStringLiteral("desktop-bootstrap"), QStringLiteral("initialize"), QStringLiteral("Initialize")});

    return {};
}

std::unique_ptr<MainWindow> AppBootstrap::CreateMainWindow(QWidget* parent)
{
    if (!m_isInitialized)
    {
        m_logger.Log(
            RoboSDP::Logging::LogLevel::Error,
            QStringLiteral("启动装配尚未完成，不能创建主窗口。"),
            RoboSDP::Errors::ErrorCode::UiStartupFailed,
            {QStringLiteral("desktop-bootstrap"), QStringLiteral("create-main-window"), QStringLiteral("CreateMainWindow")});
        return nullptr;
    }

    return std::make_unique<MainWindow>(parent);
}

void AppBootstrap::ApplyApplicationMetadata() const
{
    QCoreApplication::setOrganizationName(QStringLiteral("RoboSDP"));
    QCoreApplication::setApplicationName(QStringLiteral("RoboSDP Desktop"));
    QCoreApplication::setApplicationVersion(QStringLiteral("0.1.0"));
}

} // namespace RoboSDP::Desktop
