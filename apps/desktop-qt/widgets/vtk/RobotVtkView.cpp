#include "apps/desktop-qt/widgets/vtk/RobotVtkView.h"
#include "apps/desktop-qt/widgets/vtk/VtkSceneBuilder.h"

#include <QLabel>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

#if defined(ROBOSDP_HAVE_VTK)
#include <QVTKOpenGLNativeWidget.h>
#include <vtkActor.h>
#include <vtkCamera.h>
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

    auto readPosition = [&linkWorldPoses](
                            const QString& linkName,
                            const std::array<double, 3>& fallbackPosition) {
        const auto poseIt = linkWorldPoses.find(linkName);
        return poseIt != linkWorldPoses.end() ? poseIt->second.position_m : fallbackPosition;
    };

    for (auto& segment : m_currentScene.segments)
    {
        // 中文说明：骨架线段必须和 link world pose 同步更新，否则 FK 预览时 mesh 会动而骨架停在零位。
        segment.start_position_m = readPosition(segment.parent_link_name, segment.start_position_m);
        segment.end_position_m = readPosition(segment.child_link_name, segment.end_position_m);
    }

#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer != nullptr)
    {
        // 中文说明：动态路径重建轻量骨架/标签层，但复用 Mesh Actor 缓存，并且不重置相机。
        RefreshScene(false);
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

void RobotVtkView::ResetCameraToCurrentScene()
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer != nullptr)
    {
        // 中文说明：相机复位只作用于中央视图，不改变 URDF 场景 DTO 或模块计算状态。
        m_renderer->ResetCamera();
    }

    if (m_renderWindow != nullptr)
    {
        m_renderWindow->Render();
    }
#endif
}

void RobotVtkView::SetSkeletonVisible(bool visible)
{
    if (m_showSkeleton == visible)
    {
        return;
    }

    m_showSkeleton = visible;
    // 中文说明：顶部视图开关只影响显示层，不改变 URDF 场景 DTO 或 FK 计算结果。
    RefreshScene(false);
}

void RobotVtkView::SetVisualMeshVisible(bool visible)
{
    if (m_showVisualMesh == visible)
    {
        return;
    }

    m_showVisualMesh = visible;
    RefreshScene(false);
}

void RobotVtkView::SetCollisionMeshVisible(bool visible)
{
    if (m_showCollisionMesh == visible)
    {
        return;
    }

    m_showCollisionMesh = visible;
    RefreshScene(false);
}

void RobotVtkView::SetJointAxesVisible(bool visible)
{
    if (m_showJointAxes == visible)
    {
        return;
    }

    m_showJointAxes = visible;
    RefreshScene(false);
}

void RobotVtkView::SetAxesVisible(bool visible)
{
    if (m_showAxes == visible)
    {
        return;
    }

    m_showAxes = visible;
    RefreshScene(false);
}

void RobotVtkView::SetLinkLabelsVisible(bool visible)
{
    if (m_showLinkLabels == visible)
    {
        return;
    }

    m_showLinkLabels = visible;
    RefreshScene(false);
}

void RobotVtkView::SetJointLabelsVisible(bool visible)
{
    if (m_showJointLabels == visible)
    {
        return;
    }

    m_showJointLabels = visible;
    RefreshScene(false);
}

void RobotVtkView::SetFrontCameraView()
{
    ApplyCameraPreset(0.0, -1.0, 0.0, 0.0, 0.0, 1.0);
}

void RobotVtkView::SetSideCameraView()
{
    ApplyCameraPreset(1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
}

void RobotVtkView::SetTopCameraView()
{
    ApplyCameraPreset(0.0, 0.0, 1.0, 0.0, 1.0, 0.0);
}

void RobotVtkView::SetIsometricCameraView()
{
    ApplyCameraPreset(1.0, -1.0, 1.0, 0.0, 0.0, 1.0);
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
    m_statusLabel->setToolTip(QStringLiteral("视图显示开关已集中到顶部功能区的“视图”页签。"));
    m_layout->addWidget(m_statusLabel);
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

void RobotVtkView::RefreshScene(bool resetCamera)
{
    if (m_statusLabel != nullptr)
    {
        m_statusLabel->setText(BuildStatusText());
    }

#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer != nullptr)
    {
        UrdfPreviewDisplayOptions displayOptions;
        displayOptions.show_skeleton = m_showSkeleton;
        displayOptions.show_visual_meshes = m_showVisualMesh;
        displayOptions.show_collision_meshes = m_showCollisionMesh;
        displayOptions.show_joint_axes = m_showJointAxes;
        displayOptions.show_axes = m_showAxes;
        displayOptions.show_link_labels = m_showLinkLabels;
        displayOptions.show_joint_labels = m_showJointLabels;
        displayOptions.reset_camera = resetCamera;
        if (m_currentScene.IsEmpty())
        {
            VtkSceneBuilder::BuildMinimalTestScene(m_renderer, displayOptions.show_axes);
        }
        else
        {
            VtkSceneBuilder::BuildUrdfPreviewScene(
                m_renderer,
                m_currentScene,
                displayOptions,
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

void RobotVtkView::ApplyCameraPreset(
    double directionX,
    double directionY,
    double directionZ,
    double upX,
    double upY,
    double upZ)
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer == nullptr)
    {
        return;
    }

    double directionLength = std::sqrt(directionX * directionX + directionY * directionY + directionZ * directionZ);
    if (directionLength < 1.0e-9)
    {
        directionX = 1.0;
        directionY = -1.0;
        directionZ = 1.0;
        directionLength = std::sqrt(3.0);
    }

    directionX /= directionLength;
    directionY /= directionLength;
    directionZ /= directionLength;

    double bounds[6] = {-1.0, 1.0, -1.0, 1.0, -1.0, 1.0};
    m_renderer->ComputeVisiblePropBounds(bounds);
    const bool validBounds =
        std::isfinite(bounds[0]) &&
        std::isfinite(bounds[1]) &&
        std::isfinite(bounds[2]) &&
        std::isfinite(bounds[3]) &&
        std::isfinite(bounds[4]) &&
        std::isfinite(bounds[5]) &&
        bounds[0] <= bounds[1] &&
        bounds[2] <= bounds[3] &&
        bounds[4] <= bounds[5];

    const double centerX = validBounds ? (bounds[0] + bounds[1]) * 0.5 : 0.0;
    const double centerY = validBounds ? (bounds[2] + bounds[3]) * 0.5 : 0.0;
    const double centerZ = validBounds ? (bounds[4] + bounds[5]) * 0.5 : 0.0;
    const double spanX = validBounds ? (bounds[1] - bounds[0]) : 2.0;
    const double spanY = validBounds ? (bounds[3] - bounds[2]) : 2.0;
    const double spanZ = validBounds ? (bounds[5] - bounds[4]) : 2.0;
    const double maxSpan = std::max({spanX, spanY, spanZ, 1.0});
    const double distance = maxSpan * 2.8;

    vtkCamera* camera = m_renderer->GetActiveCamera();
    if (camera == nullptr)
    {
        return;
    }

    // 中文说明：预设视角只调整相机姿态，不修改模型 actor transform 或任何业务 DTO。
    camera->SetFocalPoint(centerX, centerY, centerZ);
    camera->SetPosition(
        centerX + directionX * distance,
        centerY + directionY * distance,
        centerZ + directionZ * distance);
    camera->SetViewUp(upX, upY, upZ);
    camera->OrthogonalizeViewUp();
    m_renderer->ResetCameraClippingRange();

    if (m_renderWindow != nullptr)
    {
        m_renderWindow->Render();
    }
#else
    Q_UNUSED(directionX);
    Q_UNUSED(directionY);
    Q_UNUSED(directionZ);
    Q_UNUSED(upX);
    Q_UNUSED(upY);
    Q_UNUSED(upZ);
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
        const QString skeletonState = m_showSkeleton ? QStringLiteral("开") : QStringLiteral("关");
        const QString visualState = m_showVisualMesh ? QStringLiteral("开") : QStringLiteral("关");
        const QString collisionState = m_showCollisionMesh ? QStringLiteral("开") : QStringLiteral("关");
        const QString jointAxisState = m_showJointAxes ? QStringLiteral("开") : QStringLiteral("关");
        const QString axesState = m_showAxes ? QStringLiteral("开") : QStringLiteral("关");
        const QString linkLabelState = m_showLinkLabels ? QStringLiteral("开") : QStringLiteral("关");
        const QString jointLabelState = m_showJointLabels ? QStringLiteral("开") : QStringLiteral("关");
        return QStringLiteral("中央三维主视图区：URDF 预览，骨架=%1，Visual=%2，Collision=%3，关节轴=%4，坐标系=%5，Link 标签=%6，Joint 标签=%7，模型 %8，节点 %9，连杆段 %10，visual mesh %11，collision mesh %12。")
            .arg(skeletonState)
            .arg(visualState)
            .arg(collisionState)
            .arg(jointAxisState)
            .arg(axesState)
            .arg(linkLabelState)
            .arg(jointLabelState)
            .arg(m_currentScene.model_name.isEmpty() ? QStringLiteral("未命名模型") : m_currentScene.model_name)
            .arg(m_currentScene.nodes.size())
            .arg(m_currentScene.segments.size())
            .arg(m_currentScene.visual_geometries.size())
            .arg(m_currentScene.collision_geometries.size());
    }

#if defined(ROBOSDP_HAVE_VTK)
    const QString axesState = m_showAxes ? QStringLiteral("开") : QStringLiteral("关");
    return QStringLiteral("中央三维主视图区：当前显示最小 VTK 测试场景，坐标系=%1，等待 URDF 骨架导入。")
        .arg(axesState);
#else
    return QStringLiteral(
        "中央三维主视图区：当前构建环境未检测到与 Qt 工具链匹配的 VTK。\n"
        "本轮仍可执行 URDF 骨架导入，但中央区域只显示文字摘要。");
#endif
}

} // namespace RoboSDP::Desktop::Vtk
