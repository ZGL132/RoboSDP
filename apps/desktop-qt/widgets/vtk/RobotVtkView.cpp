#include "apps/desktop-qt/widgets/vtk/RobotVtkView.h"
#include "apps/desktop-qt/widgets/vtk/VtkSceneBuilder.h"

#include <QLabel>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

#if defined(ROBOSDP_HAVE_VTK)
#include <QVTKOpenGLNativeWidget.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkCaptionActor2D.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNew.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkTextProperty.h>
#include <vtkTextActor.h>   // <--- 【新增】
#endif

namespace RoboSDP::Desktop::Vtk
{

RobotVtkView::RobotVtkView(QWidget* parent)
    : QWidget(parent)
{
    BuildLayout();
}

RobotVtkView::~RobotVtkView()
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_corner_axes_widget != nullptr)
    {
        // 中文说明：角落坐标轴绑定了 VTK interactor，销毁视图前先禁用，避免关闭窗口时访问失效对象。
        m_corner_axes_widget->SetEnabled(0);
    }
#endif
}

// RobotVtkView.cpp

void RobotVtkView::ShowPreviewScene(const PreviewSceneDto& scene, bool resetCamera)
{
    // 清理旧的 Actor 缓存（如 Mesh），但不一定会重置相机
    ClearCache();
    m_currentScene = scene;
    
    // 将 resetCamera 参数透传给实际的刷新函数
    RefreshScene(resetCamera);
}

void RobotVtkView::UpdatePreviewPoses(
    const std::map<QString, RoboSDP::Kinematics::Dto::CartesianPoseDto>& linkWorldPoses)
{
    // 1. 数据非空校验
    // 如果传入的姿态字典为空，说明没有新的运动学解算结果，直接返回，避免无意义的计算。
    if (linkWorldPoses.empty())
    {
        return;
    }

    // =========================================================================
    // 第一阶段：更新内存中的轻量级数据 (DTO 同步)
    // =========================================================================

    // 2. 更新节点 (Nodes) 数据
    // 中文说明：同步更新内存中的轻量场景 DTO，避免后续标签开关触发完整重绘时回到旧姿态。
    // 解释：m_currentScene 缓存了当前 3D 视图显示的所有数据。
    // 当拖动滑块时，机械臂姿态改变，我们必须把新的姿态覆盖回 m_currentScene，
    // 这样如果用户稍后点击了“显示/隐藏网格”按钮触发了重绘，画面不会错误地“跳回”到拖动前的样子。
    for (auto& node : m_currentScene.nodes)
    {
        // 去字典里找这个节点（比如 "J2" 或 "Link_1"）的新姿态
        const auto poseIt = linkWorldPoses.find(node.link_name);
        if (poseIt == linkWorldPoses.end())
        {
            continue; // 如果没找到，保持原样
        }

        // 覆盖为最新姿态（包含位置和欧拉角）
        node.world_pose = poseIt->second;
        // 单独把平移位置再存一份，方便后续画图使用
        node.position_m = poseIt->second.position_m;
    }

    // 3. 定义一个辅助读取函数 (Lambda)
    // 解释：这个函数用于安全地从 linkWorldPoses 字典里提取指定节点的三维坐标 (XYZ)。
    // 如果节点不在字典里（比如静态基座），就原样返回旧坐标 (fallbackPosition)，保证程序不崩溃。
    auto readPosition = [&linkWorldPoses](
                            const QString& linkName,
                            const std::array<double, 3>& fallbackPosition) {
        const auto poseIt = linkWorldPoses.find(linkName);
        return poseIt != linkWorldPoses.end() ? poseIt->second.position_m : fallbackPosition;
    };

    // 4. 更新连杆线段 (Segments) 数据
    for (auto& segment : m_currentScene.segments)
    {
        // 中文说明：骨架线段必须和 link world pose 同步更新，否则 FK 预览时 mesh 会动而骨架停在零位。
        // 解释：连杆在三维视图里是一根线（或圆柱）。线的两端分别是父节点和子节点。
        // 既然节点的位置已经在上面第 2 步更新了，这根线的两端坐标也必须跟着更新。
        // 否则会出现极其诡异的画面：三维模型（Mesh）在跟着滑块转动，但里面的骨架线条却停在原地。
        segment.start_position_m = readPosition(segment.parent_link_name, segment.start_position_m);
        segment.end_position_m = readPosition(segment.child_link_name, segment.end_position_m);
    }

    // =========================================================================
    // 第二阶段：触发 VTK 引擎重新渲染 (画面刷新)
    // =========================================================================

#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer != nullptr)
    {
        // 中文说明：动态路径重建轻量骨架/标签层，但复用 Mesh Actor 缓存，并且不重置相机。
        // 解释：调用核心刷新函数。注意传入了 `false` 参数，意思是：**“不要重置相机位置！”**
        // 如果这里传 `true`，用户一边拖滑块，三维视图不仅在转，相机会不断瞬间跳回初始视角，极其影响体验。
        RefreshScene(false);
    }
#else
    // 如果编译时没有开启 VTK 模块，就啥也不干，忽略这批姿态数据。
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
    // 中文说明：重置视角先包围当前可见场景，再统一回到产品要求的等轴测观察方向。
    ApplyCameraPreset(1.0, -1.0, 1.0, 0.0, 0.0, 1.0);
#endif
}

void RobotVtkView::SetSkeletonVisible(bool visible)
{
    if (m_showSkeleton == visible)
    {
        return;
    }

    m_showSkeleton = visible;
    // 中文说明：顶部视图开关只影响显示层，不改变预览场景 DTO 或 FK 计算结果。
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

void RobotVtkView::SetGroundGridVisible(bool visible)
{
    if (m_showGroundGrid == visible)
    {
        return;
    }

    m_showGroundGrid = visible;
    RefreshScene(false);
}

void RobotVtkView::SetCornerAxesVisible(bool visible)
{
    if (m_showCornerAxes == visible)
    {
        return;
    }

    m_showCornerAxes = visible;
    if (m_statusLabel != nullptr)
    {
        m_statusLabel->setText(BuildStatusText());
    }
    RefreshCornerAxesVisibility();
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

    BuildCornerAxesWidget();

    // 🔽🔽🔽 【新增水印初始化代码】 🔽🔽🔽
    m_watermark_actor = vtkSmartPointer<vtkTextActor>::New();
    m_watermark_actor->SetInput("No Preview Model"); // 默认文本
    // 设置文字属性
    vtkTextProperty* textProp = m_watermark_actor->GetTextProperty();
    textProp->SetFontSize(16);          // 字体大小
    textProp->SetColor(0.8, 0.8, 0.8);  // 浅灰色
    textProp->SetOpacity(0.6);          // 半透明效果，不喧宾夺主
    textProp->SetBold(true);
    textProp->SetShadow(true);          // 开启阴影让文字在任何背景下都清晰
    textProp->SetShadowOffset(1, -1);
    
    // 设置水印位置：屏幕右上角 (以屏幕像素系为基准)
    // VTK 的 SetPosition2 是用 Normalized Viewport (0~1) 坐标的，这里我们直接给具体像素位置或者比例
    m_watermark_actor->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
    m_watermark_actor->SetPosition(0.02, 0.95); // 0.02(左) 0.95(上)，即左上角

    m_renderer->AddActor2D(m_watermark_actor); // 加入渲染器 (2D 覆盖层)
    // 🔼🔼🔼 【新增结束】 🔼🔼🔼


    m_layout->addWidget(m_vtkWidget, 1);
    RefreshScene();
#endif
}

void RobotVtkView::BuildFallbackView()
{
    m_layout->addStretch();
    RefreshScene();
}

void RobotVtkView::BuildCornerAxesWidget()
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_vtkWidget == nullptr || m_corner_axes_widget != nullptr)
    {
        return;
    }

    vtkNew<vtkAxesActor> axesActor;
    axesActor->SetTotalLength(0.85, 0.85, 0.85);
    axesActor->SetShaftTypeToCylinder();
    axesActor->SetCylinderRadius(0.018);
    axesActor->SetConeRadius(0.075);
    axesActor->SetSphereRadius(0.045);
    axesActor->GetXAxisShaftProperty()->SetColor(0.62, 0.25, 0.25);
    axesActor->GetXAxisTipProperty()->SetColor(0.72, 0.34, 0.34);
    axesActor->GetYAxisShaftProperty()->SetColor(0.26, 0.56, 0.32);
    axesActor->GetYAxisTipProperty()->SetColor(0.32, 0.66, 0.38);
    axesActor->GetZAxisShaftProperty()->SetColor(0.28, 0.40, 0.72);
    axesActor->GetZAxisTipProperty()->SetColor(0.34, 0.48, 0.82);

    auto tuneCaption = [](vtkCaptionActor2D* captionActor) {
        if (captionActor == nullptr || captionActor->GetCaptionTextProperty() == nullptr)
        {
            return;
        }
        // 🔽🔽🔽 【核心修复：彻底禁用自动缩放】 🔽🔽🔽
        if (captionActor->GetTextActor())
        {
            captionActor->GetTextActor()->SetTextScaleModeToNone();
        }
        captionActor->BorderOff();
        captionActor->LeaderOff();
        // 🔼🔼🔼 【修复结束】 🔼🔼🔼
        captionActor->GetCaptionTextProperty()->SetFontSize(12);
        captionActor->GetCaptionTextProperty()->SetBold(false);
        captionActor->GetCaptionTextProperty()->SetColor(0.80, 0.84, 0.88);
    };
    tuneCaption(axesActor->GetXAxisCaptionActor2D());
    tuneCaption(axesActor->GetYAxisCaptionActor2D());
    tuneCaption(axesActor->GetZAxisCaptionActor2D());

    m_corner_axes_actor = axesActor;
    m_corner_axes_widget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    m_corner_axes_widget->SetOrientationMarker(m_corner_axes_actor);
    m_corner_axes_widget->SetInteractor(m_vtkWidget->interactor());
    m_corner_axes_widget->SetViewport(0.015, 0.015, 0.145, 0.145);
    RefreshCornerAxesVisibility();
#endif
}

void RobotVtkView::RefreshCornerAxesVisibility()
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_corner_axes_widget == nullptr)
    {
        return;
    }

    // 中文说明：角落方向坐标轴固定在屏幕空间，不参与主场景重建和相机包围盒计算。
    m_corner_axes_widget->SetEnabled(m_showCornerAxes ? 1 : 0);
    if (m_showCornerAxes)
    {
        m_corner_axes_widget->InteractiveOff();
    }
    if (m_renderWindow != nullptr)
    {
        m_renderWindow->Render();
    }
#endif
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
        displayOptions.show_ground_grid = m_showGroundGrid;
        displayOptions.show_link_labels = m_showLinkLabels;
        displayOptions.show_joint_labels = m_showJointLabels;
        displayOptions.reset_camera = resetCamera;
        if (m_currentScene.IsEmpty())
        {
            VtkSceneBuilder::BuildMinimalTestScene(
                m_renderer,
                displayOptions.show_axes,
                displayOptions.show_ground_grid);
                m_watermark_actor->SetInput("View: Minimal Test Scene");
                m_watermark_actor->GetTextProperty()->SetColor(0.6, 0.6, 0.6); 
        }
        else
        {
            VtkSceneBuilder::BuildUrdfPreviewScene(
                m_renderer,
                m_currentScene,
                displayOptions,
                m_link_actors,
                m_link_mesh_geometries);

           // 🔽🔽🔽 【修改这里的判断条件】 🔽🔽🔽
            // 不再判断有没有外壳，而是判断场景数据里有没有携带真实的 URDF 文件路径
            if (!m_currentScene.urdf_file_path.trimmed().isEmpty()) {
                // 如果有路径，说明它是从一个真实的 URDF 文件里读出来的
                m_watermark_actor->SetInput("[ Driven by: URDF Physical Model ]");
                m_watermark_actor->GetTextProperty()->SetColor(0.4, 0.8, 1.0); // 科技蓝
            } else {
                // 如果没有路径，说明它是我们在内存里用 DH 参数硬算出来的骨架
                m_watermark_actor->SetInput("[ Driven by: DH/MDH Kinematic Skeleton ]");
                m_watermark_actor->GetTextProperty()->SetColor(1.0, 0.6, 0.2); // 警示橙色
            }
            // 🔼🔼🔼 【修改结束】 🔼🔼🔼


        }
        // 🔽🔽🔽 【关键修复】：重新把水印贴回画布 🔽🔽🔽
        // 因为 VtkSceneBuilder 内部调用了 RemoveAllViewProps() 清空了图层，
        // 所以每次刷新后，必须重新把水印层注册进渲染器。
        m_renderer->AddActor2D(m_watermark_actor);
        // 🔼🔼🔼 【修复结束】 🔼🔼🔼

        if (resetCamera)
        {
            // 中文说明：无论当前是空参考场景还是机器人预览场景，首次展示都统一使用等轴测视角。
            ApplyCameraPreset(1.0, -1.0, 1.0, 0.0, 0.0, 1.0);
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

    vtkCamera* camera = m_renderer->GetActiveCamera();
    if (camera == nullptr)
    {
        return;
    }

    m_renderer->ResetCamera();

    double focalPoint[3] = {0.0, 0.0, 0.0};
    double position[3] = {0.0, 0.0, 1.0};
    camera->GetFocalPoint(focalPoint);
    camera->GetPosition(position);

    double distance = std::sqrt(
        (position[0] - focalPoint[0]) * (position[0] - focalPoint[0]) +
        (position[1] - focalPoint[1]) * (position[1] - focalPoint[1]) +
        (position[2] - focalPoint[2]) * (position[2] - focalPoint[2]));
    if (!std::isfinite(distance) || distance < 1.0e-6)
    {
        distance = 3.0;
    }

    // 中文说明：先让 VTK 用当前场景安全求出包围相机，再只覆写观察方向，避免标签/辅助层把手算包围盒拉坏。
    camera->SetFocalPoint(focalPoint);
    camera->SetPosition(
        focalPoint[0] + directionX * distance,
        focalPoint[1] + directionY * distance,
        focalPoint[2] + directionZ * distance);
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
            "中央三维主视图区：已加载骨架预览数据（模型 %1，节点 %2，连杆段 %3），"
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
        const QString groundGridState = m_showGroundGrid ? QStringLiteral("开") : QStringLiteral("关");
        const QString cornerAxesState = m_showCornerAxes ? QStringLiteral("开") : QStringLiteral("关");
        const QString linkLabelState = m_showLinkLabels ? QStringLiteral("开") : QStringLiteral("关");
        const QString jointLabelState = m_showJointLabels ? QStringLiteral("开") : QStringLiteral("关");
        return QStringLiteral("中央三维主视图区：预览模型，骨架=%1，Visual=%2，Collision=%3，关节轴=%4，坐标系=%5，地面网格=%6，角落坐标轴=%7，Link 标签=%8，Joint 标签=%9，模型 %10，节点 %11，连杆段 %12，visual mesh %13，collision mesh %14。")
            .arg(skeletonState)
            .arg(visualState)
            .arg(collisionState)
            .arg(jointAxisState)
            .arg(axesState)
            .arg(groundGridState)
            .arg(cornerAxesState)
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
    const QString groundGridState = m_showGroundGrid ? QStringLiteral("开") : QStringLiteral("关");
    const QString cornerAxesState = m_showCornerAxes ? QStringLiteral("开") : QStringLiteral("关");
    return QStringLiteral("中央三维主视图区：当前显示默认三维参考场景，坐标系=%1，地面网格=%2，角落坐标轴=%3，等待新的机器人骨架同步。")
        .arg(axesState)
        .arg(groundGridState)
        .arg(cornerAxesState);
#else
    return QStringLiteral(
        "中央三维主视图区：当前构建环境未检测到与 Qt 工具链匹配的 VTK。\n"
        "本轮仍可执行机器人骨架同步，但中央区域只显示文字摘要。");
#endif
}

} // namespace RoboSDP::Desktop::Vtk
