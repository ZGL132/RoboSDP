#include "apps/desktop-qt/VtkSceneWidget.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#if defined(ROBOSDP_HAVE_VTK)
#include <QVTKOpenGLNativeWidget.h>
#include <vtkActor.h>
#include <vtkCubeSource.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSphereSource.h>
#endif

namespace RoboSDP::Desktop
{

VtkSceneWidget::VtkSceneWidget(QWidget* parent)
    : QWidget(parent)
{
    m_layout = new QVBoxLayout(this);
    m_layout->setContentsMargins(12, 12, 12, 12);
    m_layout->setSpacing(8);

    BuildToolbar();

    auto* contentHost = new QWidget(this);
    m_contentLayout = new QVBoxLayout(contentHost);
    m_contentLayout->setContentsMargins(0, 0, 0, 0);
    m_contentLayout->setSpacing(8);
    m_layout->addWidget(contentHost, 1);

    RefreshScene();
}

void VtkSceneWidget::BuildToolbar()
{
    // 工具栏只提供一个最小刷新入口，不扩展复杂三维交互。
    auto* toolbarLayout = new QHBoxLayout();
    toolbarLayout->setContentsMargins(0, 0, 0, 0);
    toolbarLayout->setSpacing(8);

    m_refreshButton = new QPushButton(QStringLiteral("刷新场景"), this);
    toolbarLayout->addWidget(m_refreshButton);
    toolbarLayout->addStretch();
    m_layout->addLayout(toolbarLayout);

    connect(m_refreshButton, &QPushButton::clicked, this, [this]() {
        // 刷新动作只重建当前最小示例场景，不触发业务模块计算。
        RefreshScene();
    });
}

void VtkSceneWidget::RefreshScene()
{
    ClearSceneContent();

#if defined(ROBOSDP_HAVE_VTK)
    BuildScene();
#else
    BuildFallbackView();
#endif
}

void VtkSceneWidget::ClearSceneContent()
{
    if (m_contentLayout == nullptr)
    {
        return;
    }

    // 刷新前清空旧控件，避免旧的三维窗口或说明标签残留。
    while (QLayoutItem* item = m_contentLayout->takeAt(0))
    {
        if (QWidget* widget = item->widget())
        {
            widget->deleteLater();
        }
        delete item;
    }

    m_statusLabel = nullptr;
}

void VtkSceneWidget::BuildScene()
{
#if defined(ROBOSDP_HAVE_VTK)
    // 本轮只显示球体 + 立方体两个简单对象，用于验证最小场景对象管理已接通。
    auto* vtkWidget = new QVTKOpenGLNativeWidget(this);
    vtkWidget->setMinimumSize(480, 360);

    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkWidget->setRenderWindow(renderWindow);

    vtkNew<vtkRenderer> renderer;
    renderWindow->AddRenderer(renderer);

    vtkNew<vtkNamedColors> colors;
    renderer->SetBackground(colors->GetColor3d("SlateGray").GetData());

    vtkNew<vtkSphereSource> sphereSource;
    sphereSource->SetRadius(0.5);
    sphereSource->SetThetaResolution(32);
    sphereSource->SetPhiResolution(32);

    vtkNew<vtkPolyDataMapper> sphereMapper;
    sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

    vtkNew<vtkActor> sphereActor;
    sphereActor->SetMapper(sphereMapper);
    sphereActor->SetPosition(-0.7, 0.0, 0.0);
    sphereActor->GetProperty()->SetColor(colors->GetColor3d("Tomato").GetData());

    vtkNew<vtkCubeSource> cubeSource;
    cubeSource->SetXLength(0.6);
    cubeSource->SetYLength(0.6);
    cubeSource->SetZLength(0.6);

    vtkNew<vtkPolyDataMapper> cubeMapper;
    cubeMapper->SetInputConnection(cubeSource->GetOutputPort());

    vtkNew<vtkActor> cubeActor;
    cubeActor->SetMapper(cubeMapper);
    cubeActor->SetPosition(0.8, 0.0, 0.0);
    cubeActor->GetProperty()->SetColor(colors->GetColor3d("Banana").GetData());

    renderer->AddActor(sphereActor);
    renderer->AddActor(cubeActor);
    renderer->ResetCamera();

    m_statusLabel = new QLabel(
        QStringLiteral("中央三维主视图区：VTK 最小场景已加载，当前包含球体与立方体两个示例对象。"),
        this);
    m_statusLabel->setWordWrap(true);
    m_contentLayout->addWidget(m_statusLabel);
    m_contentLayout->addWidget(vtkWidget, 1);
#else
    BuildFallbackView();
#endif
}

void VtkSceneWidget::BuildFallbackView()
{
    // 若当前构建环境未检测到 VTK，则保持中央区域可用，并明确提示阻塞点。
    m_statusLabel = new QLabel(
        QStringLiteral(
            "中央三维主视图区：VTK 最小接线骨架已接入，但当前构建环境未检测到 VTK 依赖。\n"
            "刷新场景会重建当前占位说明；后续补齐 VTK Qt 模块和链接配置后，可直接显示真实三维对象。"),
        this);
    m_statusLabel->setWordWrap(true);
    m_contentLayout->addWidget(m_statusLabel);
    m_contentLayout->addStretch();
}

} // namespace RoboSDP::Desktop
