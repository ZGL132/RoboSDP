#include "apps/desktop-qt/widgets/vtk/RobotVtkView.h"
#include "apps/desktop-qt/widgets/vtk/VtkSceneBuilder.h"

#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

#if defined(ROBOSDP_HAVE_VTK)
#include <QVTKOpenGLNativeWidget.h>
#include <vtkActor.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNew.h>
#include <vtkRenderer.h>
#endif

namespace RoboSDP::Desktop::Vtk
{

RobotVtkView::RobotVtkView(QWidget* parent)
    : QWidget(parent)
{
    BuildLayout();
}

void RobotVtkView::ShowUrdfPreviewScene(const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& scene)
{
    ClearCache();
    m_currentScene = scene;
    RefreshScene();
}

void RobotVtkView::UpdatePreviewPoses(
    const std::map<QString, RoboSDP::Kinematics::Dto::CartesianPoseDto>& linkWorldPoses)
{
    if (linkWorldPoses.empty())
    {
        return;
    }

    // 中文说明：同步更新内存中的轻量场景 DTO，避免后续标签开关触发完整重绘时回到旧姿态。
    for (auto& node : m_currentScene.nodes)
    {
        const auto poseIt = linkWorldPoses.find(node.link_name);
        if (poseIt == linkWorldPoses.end())
        {
            continue;
        }

        node.world_pose = poseIt->second;
        node.position_m = poseIt->second.position_m;
    }

#if defined(ROBOSDP_HAVE_VTK)
    if (VtkSceneBuilder::UpdateCachedMeshActorTransforms(
            linkWorldPoses,
            m_link_actors,
            m_link_mesh_geometries) &&
        m_renderWindow != nullptr)
    {
        // 中文说明：高频路径只触发渲染窗口刷新，不 RemoveAllViewProps，也不重新读取 STL。
        m_renderWindow->Render();
    }
#else
    Q_UNUSED(linkWorldPoses);
#endif
}

void RobotVtkView::ClearCache()
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer != nullptr)
    {
        for (const auto& [linkName, actor] : m_link_actors)
        {
            Q_UNUSED(linkName);
            if (actor != nullptr)
            {
                m_renderer->RemoveActor(actor);
            }
        }
    }

    m_link_actors.clear();
    m_link_mesh_geometries.clear();
#endif
}

void RobotVtkView::BuildLayout()
{
    m_layout = new QVBoxLayout(this);
    m_layout->setContentsMargins(12, 12, 12, 12);
    m_layout->setSpacing(8);

    BuildControlBar();

#if defined(ROBOSDP_HAVE_VTK)
    BuildVtkView();
#else
    BuildFallbackView();
#endif
}

void RobotVtkView::BuildControlBar()
{
    m_statusLabel = new QLabel(this);
    m_statusLabel->setWordWrap(true);
    m_layout->addWidget(m_statusLabel);

    m_toggleLayout = new QHBoxLayout();
    m_showAxesCheckBox = new QCheckBox(QStringLiteral("显示坐标系"), this);
    m_showLinkLabelsCheckBox = new QCheckBox(QStringLiteral("显示 Link 标签"), this);
    m_showJointLabelsCheckBox = new QCheckBox(QStringLiteral("显示 Joint 标签"), this);
    m_showAxesCheckBox->setChecked(true);
    m_showLinkLabelsCheckBox->setChecked(true);
    m_showJointLabelsCheckBox->setChecked(true);
    m_toggleLayout->addWidget(m_showAxesCheckBox);
    m_toggleLayout->addWidget(m_showLinkLabelsCheckBox);
    m_toggleLayout->addWidget(m_showJointLabelsCheckBox);
    m_toggleLayout->addStretch();
    m_layout->addLayout(m_toggleLayout);

    connect(m_showAxesCheckBox, &QCheckBox::toggled, this, [this](bool) { RefreshScene(); });
    connect(m_showLinkLabelsCheckBox, &QCheckBox::toggled, this, [this](bool) { RefreshScene(); });
    connect(m_showJointLabelsCheckBox, &QCheckBox::toggled, this, [this](bool) { RefreshScene(); });
}

void RobotVtkView::BuildVtkView()
{
#if defined(ROBOSDP_HAVE_VTK)
    // 中文说明：中央三维区使用原生 QVTK 控件承载骨架预览，而不是直接在 QWidget 里写渲染逻辑。
    m_vtkWidget = new QVTKOpenGLNativeWidget(this);
    m_vtkWidget->setMinimumSize(640, 480);

    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    m_vtkWidget->setRenderWindow(renderWindow);
    m_renderWindow = renderWindow;

    vtkNew<vtkRenderer> renderer;
    renderWindow->AddRenderer(renderer);
    m_renderer = renderer;

    m_layout->addWidget(m_vtkWidget, 1);
    RefreshScene();
#endif
}

void RobotVtkView::BuildFallbackView()
{
    m_layout->addStretch();
    RefreshScene();
}

void RobotVtkView::RefreshScene()
{
    if (m_statusLabel != nullptr)
    {
        m_statusLabel->setText(BuildStatusText());
    }

#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer != nullptr)
    {
        const bool showAxes =
            m_showAxesCheckBox != nullptr ? m_showAxesCheckBox->isChecked() : true;
        if (m_currentScene.IsEmpty())
        {
            VtkSceneBuilder::BuildMinimalTestScene(m_renderer, showAxes);
        }
        else
        {
            const bool showLinkLabels =
                m_showLinkLabelsCheckBox != nullptr ? m_showLinkLabelsCheckBox->isChecked() : true;
            const bool showJointLabels =
                m_showJointLabelsCheckBox != nullptr ? m_showJointLabelsCheckBox->isChecked() : true;
            VtkSceneBuilder::BuildUrdfPreviewScene(
                m_renderer,
                m_currentScene,
                showLinkLabels,
                showJointLabels,
                showAxes,
                m_link_actors,
                m_link_mesh_geometries);
        }
    }

    if (m_renderWindow != nullptr)
    {
        m_renderWindow->Render();
    }
#endif
}

QString RobotVtkView::BuildStatusText() const
{
#if !defined(ROBOSDP_HAVE_VTK)
    if (!m_currentScene.IsEmpty())
    {
        return QStringLiteral(
            "中央三维主视图区：已加载 URDF 骨架预览数据（模型 %1，节点 %2，连杆段 %3），"
            "但当前构建未启用 VTK，因此只能显示文字摘要。请检查 ROBOSDP_VTK_DIR 与 VTK 组件配置。")
            .arg(m_currentScene.model_name.isEmpty() ? QStringLiteral("未命名模型") : m_currentScene.model_name)
            .arg(m_currentScene.nodes.size())
            .arg(m_currentScene.segments.size());
    }
#endif

    if (!m_currentScene.IsEmpty())
    {
        const QString axesState =
            (m_showAxesCheckBox != nullptr && m_showAxesCheckBox->isChecked())
            ? QStringLiteral("开")
            : QStringLiteral("关");
        const QString linkLabelState =
            (m_showLinkLabelsCheckBox != nullptr && m_showLinkLabelsCheckBox->isChecked())
            ? QStringLiteral("开")
            : QStringLiteral("关");
        const QString jointLabelState =
            (m_showJointLabelsCheckBox != nullptr && m_showJointLabelsCheckBox->isChecked())
            ? QStringLiteral("开")
            : QStringLiteral("关");
        return QStringLiteral("中央三维主视图区：当前显示 URDF 骨架预览，坐标系=%1，Link 标签=%2，Joint 标签=%3，模型 %4，节点 %5，连杆段 %6。")
            .arg(axesState)
            .arg(linkLabelState)
            .arg(jointLabelState)
            .arg(m_currentScene.model_name.isEmpty() ? QStringLiteral("未命名模型") : m_currentScene.model_name)
            .arg(m_currentScene.nodes.size())
            .arg(m_currentScene.segments.size());
    }

#if defined(ROBOSDP_HAVE_VTK)
    const QString axesState =
        (m_showAxesCheckBox != nullptr && m_showAxesCheckBox->isChecked())
        ? QStringLiteral("开")
        : QStringLiteral("关");
    return QStringLiteral("中央三维主视图区：当前显示最小 VTK 测试场景，坐标系=%1，等待 URDF 骨架导入。")
        .arg(axesState);
#else
    return QStringLiteral(
        "中央三维主视图区：当前构建环境未检测到与 Qt 工具链匹配的 VTK。\n"
        "本轮仍可执行 URDF 骨架导入，但中央区域只显示文字摘要。");
#endif
}

} // namespace RoboSDP::Desktop::Vtk
