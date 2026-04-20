#include "apps/vtk-smoke/VtkSmokeWindow.h"

#include <QApplication>
#include <QSurfaceFormat>

#if defined(ROBOSDP_VTK_SMOKE_HAVE_VTK)
#include <QVTKOpenGLNativeWidget.h>
#endif

int main(int argc, char* argv[])
{
#if defined(ROBOSDP_VTK_SMOKE_HAVE_VTK)
    // 在 QApplication 创建前设置 Qt 默认 OpenGL 格式，
    // 这是 QVTKOpenGLNativeWidget 正常初始化所需的最小前置条件。
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
#endif

    QApplication app(argc, argv);

    RoboSDP::VtkSmoke::VtkSmokeWindow window;
    window.show();

    return app.exec();
}
