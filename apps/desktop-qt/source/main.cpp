#include "apps/desktop-qt/AppBootstrap.h"
#include "apps/desktop-qt/MainWindow.h"
#include "core/errors/ErrorCode.h"

#include <QApplication>
#include <QCoreApplication>
#include <QIcon>
#include <QTimer>

int main(int argc, char* argv[])
{
    RoboSDP::Desktop::AppBootstrap bootstrap;
    bootstrap.ConfigureGraphicsSurfaceFormat();

    QApplication app(argc, argv);
    // 中文说明：设置运行时窗口图标，确保标题栏和任务栏显示 RoboSDP 标识。
    app.setWindowIcon(QIcon(QStringLiteral(":/app/robosdp_256.png")));

    const RoboSDP::Desktop::AppBootstrapResult bootstrapResult = bootstrap.Initialize();
    if (!bootstrapResult.IsSuccess())
    {
        return static_cast<int>(bootstrapResult.errorCode);
    }

    auto desktopMainWindow = bootstrap.CreateMainWindow();
    if (!desktopMainWindow)
    {
        return static_cast<int>(RoboSDP::Errors::ErrorCode::UiStartupFailed);
    }

    desktopMainWindow->show();

    // 该模式仅用于本地自动化验收，确保主窗口可被创建并进入事件循环。
    if (app.arguments().contains(QStringLiteral("--smoke-test")))
    {
        QTimer::singleShot(300, &app, &QCoreApplication::quit);
    }

    return app.exec();
}
