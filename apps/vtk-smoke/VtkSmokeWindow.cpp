#include "apps/vtk-smoke/VtkSmokeWindow.h"

#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>

#if defined(ROBOSDP_VTK_SMOKE_HAVE_VTK)
#include <QVTKOpenGLNativeWidget.h>
#include <vtkActor.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#endif

namespace RoboSDP::VtkSmoke
{

VtkSmokeWindow::VtkSmokeWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle(QStringLiteral("RoboSDP VTK Smoke"));
    resize(960, 640);

    BuildLayout();
    BuildScene();
}

void VtkSmokeWindow::BuildLayout()
{
    // 该验证窗口只保留一个最小中央区域，不接入项目树、属性栏和日志区。
    m_centralWidget = new QWidget(this);
    m_layout = new QVBoxLayout(m_centralWidget);
    m_layout->setContentsMargins(12, 12, 12, 12);
    m_layout->setSpacing(8);

    m_statusLabel = new QLabel(m_centralWidget);
    m_statusLabel->setWordWrap(true);
    m_layout->addWidget(m_statusLabel);

    setCentralWidget(m_centralWidget);
}

void VtkSmokeWindow::BuildScene()
{
#if defined(ROBOSDP_VTK_SMOKE_HAVE_VTK)
    // 使用 QVTKOpenGLNativeWidget 作为最小 Qt 承载控件，验证原生 OpenGL 视图接线。
    auto* vtkWidget = new QVTKOpenGLNativeWidget(m_centralWidget);
    vtkWidget->setMinimumSize(640, 480);

    // 使用 vtkGenericOpenGLRenderWindow 作为渲染窗口，符合当前 Qt + VTK 推荐接法。
    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkWidget->setRenderWindow(renderWindow);

    vtkNew<vtkRenderer> renderer;
    renderWindow->AddRenderer(renderer);

    vtkNew<vtkNamedColors> colors;
    renderer->SetBackground(colors->GetColor3d("MidnightBlue").GetData());

    // 本轮只显示一个最小球体对象，用于验证三维窗口已真正创建并完成渲染。
    vtkNew<vtkSphereSource> sphereSource;
    sphereSource->SetRadius(0.5);
    sphereSource->SetThetaResolution(48);
    sphereSource->SetPhiResolution(48);

    vtkNew<vtkPolyDataMapper> sphereMapper;
    sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

    vtkNew<vtkActor> sphereActor;
    sphereActor->SetMapper(sphereMapper);
    sphereActor->GetProperty()->SetColor(colors->GetColor3d("Orange").GetData());

    renderer->AddActor(sphereActor);
    renderer->ResetCamera();

    m_statusLabel->setText(
        QStringLiteral(
            "VTK 最小验证已接通：当前窗口使用 QVTKOpenGLNativeWidget + "
            "vtkGenericOpenGLRenderWindow，并显示一个球体对象。"));
    m_layout->addWidget(vtkWidget, 1);
#else
    BuildFallbackView();
#endif
}

void VtkSmokeWindow::BuildFallbackView()
{
    // 若未检测到 VTK，则明确提示依赖要求，方便先完成工具链验证再接入主项目。
    auto* placeholder = new QLabel(
        QStringLiteral(
            "当前未检测到可用的 VTK Qt 依赖，因此未创建真实三维视图。\n"
            "若当前工具链为 Qt MinGW，则 VTK 也必须由同一套 MinGW 工具链编译，"
            "并且需要提供 QVTKOpenGLNativeWidget 所需模块。"),
        m_centralWidget);
    placeholder->setWordWrap(true);

    m_statusLabel->setText(QStringLiteral("VTK 最小验证窗口已启动，但当前为依赖缺失 fallback 模式。"));
    m_layout->addWidget(placeholder);
    m_layout->addStretch();
}

} // namespace RoboSDP::VtkSmoke
