#include "apps/desktop-qt/widgets/vtk/RobotVtkView.h"
#include "apps/desktop-qt/widgets/vtk/VtkSceneBuilder.h"

#include <QLabel>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>

#if defined(ROBOSDP_HAVE_VTK)
#include <QVTKOpenGLNativeWidget.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkBoxWidget2.h>
#include <vtkBoxRepresentation.h>
#include <vtkCamera.h>
#include <vtkCaptionActor2D.h>
#include <vtkGlyph3D.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkSphereSource.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNew.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkProperty.h>
#include <vtkPropPicker.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTextProperty.h>
#include <vtkTextActor.h>
#include <vtkTransform.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>

/// @brief 自定义 VTK 鼠标交互器，支持左键点击拾取 3D Actor、滚轮关节驱动、虚线框选等。
class VtkClickInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
    /// @brief 工厂方法（手动实现而非 vtkStandardNewMacro，兼容 MinGW）。
    static VtkClickInteractorStyle* New()
    {
        auto* ret = new VtkClickInteractorStyle;
        ret->InitializeObjectBase();
        return ret;
    }

    vtkTypeMacro(VtkClickInteractorStyle, vtkInteractorStyleTrackballCamera);

    /// @brief 指向所属的 RobotVtkView，用于回调 HandleActorClicked。
    RoboSDP::Desktop::Vtk::RobotVtkView* parentView = nullptr;
    /// @brief 当前渲染器，用于 vtkPropPicker 拾取。
    vtkRenderer* renderer = nullptr;

    void OnLeftButtonDown() override
    {
        // 中文说明：左键按下时先执行拾取，再传递给父类保持视角旋转功能。
        if (renderer != nullptr && parentView != nullptr && this->GetInteractor() != nullptr)
        {
            const int* clickPos = this->GetInteractor()->GetEventPosition();
            vtkNew<vtkPropPicker> picker;
            picker->Pick(clickPos[0], clickPos[1], 0, renderer);

            vtkActor* clickedActor = picker->GetActor();
            // 中文说明：传递拾取位置（世界坐标），供 HandleActorClicked 在骨架节点查找时使用。
            const double* pickPos = picker->GetPickPosition();
            parentView->HandleActorClicked(clickedActor, pickPos);
        }
        // 保留原有的视角旋转/平移功能
        vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }

    // ── 【逆向驱动】鼠标滚轮事件重写 ──────────────────────────────
    /// @brief 滚轮向前滚动：如果已选中连杆，则发射关节正转信号，否则保留默认视角缩放。
    ///        步长由 parentView->GetScrollStep() 控制，默认 1.0°。
    void OnMouseWheelForward() override
    {
        if (parentView != nullptr && !parentView->GetPickedLinkName().isEmpty())
        {
            parentView->EmitJointScroll(parentView->GetScrollStep()); // 正转 +step 度
            return; // 不传递给基类，避免视角缩放干扰
        }
        vtkInteractorStyleTrackballCamera::OnMouseWheelForward();
    }

    /// @brief 滚轮向后滚动：如果已选中连杆，则发射关节反转信号。
    ///        步长由 parentView->GetScrollStep() 控制，默认 -1.0°。
    void OnMouseWheelBackward() override
    {
        if (parentView != nullptr && !parentView->GetPickedLinkName().isEmpty())
        {
            parentView->EmitJointScroll(-parentView->GetScrollStep()); // 反转 -step 度
            return;
        }
        vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
    }
};

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

    // 【修复】场景数据更新后，重建 link_name → joint_index 映射表，
    // 供 EmitJointScroll 在滚轮驱动时快速查找关节索引。
    RebuildLinkToJointMap();

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
        // 动态路径重建轻量骨架/标签层，但复用 Mesh Actor 缓存，并且不重置相机。
        RefreshScene(false);

        // 更新 TCP 坐标系指示器位姿（如果 tcp_frame 在 poseMap 中）
        const auto tcpPoseIt = linkWorldPoses.find(QStringLiteral("tcp_frame"));
        if (tcpPoseIt != linkWorldPoses.end() && m_tcp_axes_actor != nullptr)
        {
            const auto& tcpPose = tcpPoseIt->second;
            vtkNew<vtkTransform> tcpTransform;
            tcpTransform->Translate(tcpPose.position_m[0],
                                    tcpPose.position_m[1],
                                    tcpPose.position_m[2]);
            // RPY → VTK 旋转：先绕 Z 轴转 RZ，再绕 Y 轴转 RY，再绕 X 轴转 RX
            tcpTransform->RotateZ(tcpPose.rpy_deg[2]);
            tcpTransform->RotateY(tcpPose.rpy_deg[1]);
            tcpTransform->RotateX(tcpPose.rpy_deg[0]);
            m_tcp_axes_actor->SetUserTransform(tcpTransform);
            m_tcp_axes_actor->SetVisibility(true);
        }
        // TCP 坐标轴更新在 RefreshScene(false) 的 Render() 之后，
        // 必须额外触发一次渲染，否则新位置不会立即显示。
        if (m_renderWindow != nullptr)
        {
            m_renderWindow->Render();
        }
    }
#else
    Q_UNUSED(linkWorldPoses);
#endif
}

void RobotVtkView::ShowWorkspacePointCloud(
    const std::vector<std::array<double, 3>>& tcpPositions)
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer == nullptr) return;

    // 移除旧的点云 Actor（如果存在）
    if (m_workspace_point_cloud_actor != nullptr)
    {
        m_renderer->RemoveActor(m_workspace_point_cloud_actor);
    }

    if (tcpPositions.empty())
    {
        m_workspace_point_cloud_actor = nullptr;
        if (m_renderWindow != nullptr) m_renderWindow->Render();
        return;
    }

    // 构建点云数据：将 TCP 位置坐标填入 vtkPoints
    vtkNew<vtkPoints> points;
    points->SetNumberOfPoints(static_cast<vtkIdType>(tcpPositions.size()));
    for (vtkIdType i = 0; i < static_cast<vtkIdType>(tcpPositions.size()); ++i)
    {
        points->SetPoint(i, tcpPositions[i][0], tcpPositions[i][1], tcpPositions[i][2]);
    }

    // 构建 PolyData
    vtkNew<vtkPolyData> polyData;
    polyData->SetPoints(points);

    // 使用 VertexGlyphFilter 将每个点渲染为一个顶点
    vtkNew<vtkVertexGlyphFilter> glyphFilter;
    glyphFilter->SetInputData(polyData);
    glyphFilter->Update();

    // 映射器
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(glyphFilter->GetOutputPort());

    // Actor：绿色小圆点
    m_workspace_point_cloud_actor = vtkSmartPointer<vtkActor>::New();
    m_workspace_point_cloud_actor->SetMapper(mapper);
    m_workspace_point_cloud_actor->GetProperty()->SetColor(0.2, 0.8, 0.2); // 绿色
    m_workspace_point_cloud_actor->GetProperty()->SetPointSize(3);
    m_workspace_point_cloud_actor->GetProperty()->SetOpacity(0.7); // 半透明

    m_renderer->AddActor(m_workspace_point_cloud_actor);

    // 点云显示后，将点云 Actor 添加到点云可见性列表中，方便后续开关
    if (m_renderWindow != nullptr)
    {
        m_renderWindow->Render();
    }
#else
    Q_UNUSED(tcpPositions);
#endif
}

void RobotVtkView::ShowColoredWorkspacePointCloud(
    const std::vector<std::array<double, 3>>& tcpPositions,
    const std::vector<bool>& isSingular)
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer == nullptr) return;

    // 移除旧的点云 Actor
    if (m_workspace_point_cloud_actor != nullptr)
    {
        m_renderer->RemoveActor(m_workspace_point_cloud_actor);
    }

    if (tcpPositions.empty() || tcpPositions.size() != isSingular.size())
    {
        m_workspace_point_cloud_actor = nullptr;
        if (m_renderWindow != nullptr) m_renderWindow->Render();
        return;
    }

    // 构建点云数据
    vtkNew<vtkPoints> points;
    points->SetNumberOfPoints(static_cast<vtkIdType>(tcpPositions.size()));
    vtkNew<vtkUnsignedCharArray> colors;
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");

    for (vtkIdType i = 0; i < static_cast<vtkIdType>(tcpPositions.size()); ++i)
    {
        points->SetPoint(i, tcpPositions[i][0], tcpPositions[i][1], tcpPositions[i][2]);
        if (isSingular[static_cast<std::size_t>(i)])
            colors->InsertNextTuple3(220, 38, 38);   // 红色 = 奇异
        else
            colors->InsertNextTuple3(51, 204, 51);    // 绿色 = 正常
    }

    vtkNew<vtkPolyData> polyData;
    polyData->SetPoints(points);
    polyData->GetPointData()->SetScalars(colors);

    vtkNew<vtkVertexGlyphFilter> glyphFilter;
    glyphFilter->SetInputData(polyData);
    glyphFilter->Update();

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(glyphFilter->GetOutputPort());
    mapper->ScalarVisibilityOn();
    mapper->SetScalarModeToUsePointData();

    m_workspace_point_cloud_actor = vtkSmartPointer<vtkActor>::New();
    m_workspace_point_cloud_actor->SetMapper(mapper);
    m_workspace_point_cloud_actor->GetProperty()->SetPointSize(3);
    m_workspace_point_cloud_actor->GetProperty()->SetOpacity(0.7);

    m_renderer->AddActor(m_workspace_point_cloud_actor);
    if (m_renderWindow != nullptr)
        m_renderWindow->Render();
#else
    Q_UNUSED(tcpPositions);
    Q_UNUSED(isSingular);
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

    // 【修复】清除 m_link_actors 后，m_last_picked_actor 指向的 Actor 可能已被销毁，
    // 必须置空以避免后续点击时野指针崩溃。
    m_last_picked_actor = nullptr;
    m_link_actors.clear();
    m_link_mesh_geometries.clear();
    m_link_to_joint_index.clear();
#endif

    // 【修复】模型场景切换时，清除当前选中的连杆名称，避免旧引用残留。
    m_current_picked_link.clear();
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
    // 中文说明：开启 8x MSAA 多重采样抗锯齿，平滑 3D 骨架边缘棱角，提升渲染画质。
    renderWindow->SetMultiSamples(8);
    m_vtkWidget->setRenderWindow(renderWindow);
    m_renderWindow = renderWindow;

    vtkNew<vtkRenderer> renderer;
    renderWindow->AddRenderer(renderer);
    m_renderer = renderer;

    BuildCornerAxesWidget();

    // TCP 坐标系指示器：红绿蓝三轴，默认隐藏，FK 成功后显示在末端
    m_tcp_axes_actor = vtkSmartPointer<vtkAxesActor>::New();
    m_tcp_axes_actor->SetTotalLength(0.15, 0.15, 0.15);
    m_tcp_axes_actor->SetNormalizedShaftLength(0.8, 0.8, 0.8);
    m_tcp_axes_actor->SetNormalizedTipLength(0.2, 0.2, 0.2);
    m_tcp_axes_actor->SetVisibility(false);
    // XYZ 标签样式：与角落极坐标一致，禁用自动缩放，小字号 + 亮色
    {
        auto tuneCaption = [](vtkCaptionActor2D* captionActor) {
            if (captionActor == nullptr || captionActor->GetCaptionTextProperty() == nullptr)
                return;
            if (captionActor->GetTextActor())
                captionActor->GetTextActor()->SetTextScaleModeToNone();
            captionActor->BorderOff();
            captionActor->LeaderOff();
            captionActor->GetCaptionTextProperty()->SetFontSize(12);
            captionActor->GetCaptionTextProperty()->SetBold(false);
            captionActor->GetCaptionTextProperty()->SetColor(0.90, 0.92, 0.95);
        };
        tuneCaption(m_tcp_axes_actor->GetXAxisCaptionActor2D());
        tuneCaption(m_tcp_axes_actor->GetYAxisCaptionActor2D());
        tuneCaption(m_tcp_axes_actor->GetZAxisCaptionActor2D());
    }
    m_renderer->AddActor(m_tcp_axes_actor);

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

    // 中文说明：绑定自定义拾取交互器，替换默认相机操作风格，支持左键点击高亮。
    vtkNew<VtkClickInteractorStyle> clickStyle;
    clickStyle->parentView = this;
    clickStyle->renderer = m_renderer;
    clickStyle->SetDefaultRenderer(m_renderer);
    m_vtkWidget->interactor()->SetInteractorStyle(clickStyle);

    RefreshScene();
#endif
}

void RobotVtkView::HandleActorClicked(vtkActor* clickedActor, const double pickPosition[3])
{
#if defined(ROBOSDP_HAVE_VTK)
    // 中文说明：恢复上一个被选中对象的原始颜色和环境光系数。
    if (m_last_picked_actor != nullptr && m_last_picked_actor->GetProperty() != nullptr)
    {
        m_last_picked_actor->GetProperty()->SetColor(m_last_picked_color);
        m_last_picked_actor->GetProperty()->SetAmbient(m_last_picked_ambient);
    }

    if (clickedActor == nullptr)
    {
        // 【修复】点击空白区域，同时清除选中状态和当前选中连杆名称
        m_last_picked_actor = nullptr;
        m_current_picked_link.clear();    // <--- 【新增】清理选中连杆
        if (m_statusLabel != nullptr)
        {
            m_statusLabel->setText(BuildStatusText());
        }
        return;
    }

    // 记录新选中对象的原始属性
    m_last_picked_actor = clickedActor;
    clickedActor->GetProperty()->GetColor(m_last_picked_color);
    m_last_picked_ambient = clickedActor->GetProperty()->GetAmbient();

    // 设置高亮效果：青色 + 高环境光，模拟"发光发亮"的选中状态
    clickedActor->GetProperty()->SetColor(0.0, 1.0, 1.0); // Cyan
    clickedActor->GetProperty()->SetAmbient(0.8);

    // ── 在 m_link_actors 字典中反向查找被点击 Actor 的实体名称 ──────────
    // 【修复】从复合键 "visual:link_1:0" 中提取纯 link_name（中间段），
    // 格式为 "layerName:linkName:geometryIndex"，使用 section(':', 1, 1) 取 linkName。
    QString pickedLinkName;
    for (auto it = m_link_actors.begin(); it != m_link_actors.end(); ++it)
    {
        if (it->second.Get() == clickedActor)
        {
            // 提取复合键的第二段：layer:link_name:index → link_name
            pickedLinkName = it->first.section(QLatin1Char(':'), 1, 1);
            break;
        }
    }

    // ── 如果在 Mesh Actor 字典中未找到，尝试按拾取位置查找最近骨架节点 ──
    if (pickedLinkName.isEmpty() && pickPosition != nullptr)
    {
        // 中文说明：遍历所有骨架节点，找距离拾取位置最近的节点，
        // 如果距离小于合理阈值（2cm），认为点击了该骨架球。
        double minDistSq = std::numeric_limits<double>::max();
        int nearestNodeIndex = -1;
        for (int i = 0; i < static_cast<int>(m_currentScene.nodes.size()); ++i)
        {
            const auto& node = m_currentScene.nodes[i];
            const double dx = node.position_m[0] - pickPosition[0];
            const double dy = node.position_m[1] - pickPosition[1];
            const double dz = node.position_m[2] - pickPosition[2];
            const double distSq = dx*dx + dy*dy + dz*dz;
            if (distSq < minDistSq)
            {
                minDistSq = distSq;
                nearestNodeIndex = i;
            }
        }
        if (nearestNodeIndex >= 0)
        {
            const double threshold = 0.02; // 2cm 阈值，覆盖骨架球半径
            if (std::sqrt(minDistSq) < threshold)
            {
                pickedLinkName = m_currentScene.nodes[nearestNodeIndex].link_name;
            }
        }
    }

    // 【逆向驱动】保存当前选中连杆名称（纯 link_name），供滚轮关节驱动使用。
    m_current_picked_link = pickedLinkName;

    // 中文说明：状态栏显示，便于用户确认当前选中了哪个连杆/骨架。
    const QString displayName = pickedLinkName.isEmpty()
        ? QStringLiteral("骨架段/关节")
        : pickedLinkName;
    if (m_statusLabel != nullptr)
    {
        m_statusLabel->setText(
            QStringLiteral("中央三维主视图区：已选中 [ %1 ]").arg(displayName));
    }

    if (m_renderWindow != nullptr)
    {
        m_renderWindow->Render();
    }
#else
    Q_UNUSED(clickedActor);
    Q_UNUSED(pickPosition);
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
        // 【修复】场景重建会调用 RemoveAllViewProps()，之前缓存的 Actor 指针全部失效，
        // 必须清空 m_last_picked_actor 避免后续点击时野指针崩溃。
        // 注意：不清除 m_current_picked_link（QString），因为它不是指针类型，
        // 在场景重建后仍保持有效。模型切换时的清理由 ClearCache() 负责。
        m_last_picked_actor = nullptr;

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

        // 🔽🔽🔽 重新添加 TCP 坐标系指示器（同样被 RemoveAllViewProps 清掉）🔽🔽🔽
        if (m_tcp_axes_actor != nullptr)
        {
            m_renderer->AddActor(m_tcp_axes_actor);
        }
        // 🔼🔼🔼 🔼🔼🔼

        // ── 【逆向驱动】TCP 3D Gizmo 初始化（vtkBoxWidget2）────────────
        if (m_currentScene.IsEmpty())
        {
            // 空场景下禁用 Gizmo
            if (m_tcp_gizmo_widget != nullptr)
            {
                m_tcp_gizmo_widget->SetEnabled(0);
            }
        }
        else
        {
            // 有机器人场景 → 确保 Gizmo 创建并启用
            if (m_tcp_gizmo_widget == nullptr)
            {
                // 创建 vtkBoxRepresentation（变换框的表现层）
                vtkNew<vtkBoxRepresentation> boxRep;

                // 创建 vtkBoxWidget2（变换框的交互控件）
                m_tcp_gizmo_widget = vtkSmartPointer<vtkBoxWidget2>::New();
                m_tcp_gizmo_widget->SetInteractor(m_vtkWidget->interactor());
                m_tcp_gizmo_widget->SetRepresentation(boxRep);
                m_tcp_gizmo_widget->SetTranslationEnabled(1);
                m_tcp_gizmo_widget->SetRotationEnabled(1);
                m_tcp_gizmo_widget->SetScalingEnabled(0);   // 不缩放
                m_tcp_gizmo_widget->SetMoveFacesEnabled(1);

                // 注册 InteractionEvent 回调：拖动时发射位姿信号
                vtkNew<vtkCallbackCommand> tcpDragCallback;
                tcpDragCallback->SetCallback([](vtkObject* caller,
                                                 long unsigned int,
                                                 void* clientData,
                                                 void*) {
                    auto* widget = vtkBoxWidget2::SafeDownCast(caller);
                    if (widget == nullptr) return;
                    auto* view = static_cast<RoboSDP::Desktop::Vtk::RobotVtkView*>(clientData);
                    if (view == nullptr) return;
                    // 从关联的 Representation 中提取变换矩阵
                    auto* rep = vtkBoxRepresentation::SafeDownCast(
                        widget->GetRepresentation());
                    if (rep == nullptr) return;
                    vtkNew<vtkTransform> tcpTransform;
                    rep->GetTransform(tcpTransform);
                    view->EmitTcpDrag(tcpTransform);
                });
                tcpDragCallback->SetClientData(this);
                m_tcp_gizmo_widget->AddObserver(vtkCommand::InteractionEvent, tcpDragCallback);
            }

            // 将 Gizmo 定位到末端（取场景中最后一个节点的位置）
            if (!m_currentScene.nodes.empty())
            {
                const auto& lastNode = m_currentScene.nodes.back();
                const auto& pos = lastNode.world_pose.position_m;
                // 用小包围盒放置，然后通过 SetTransform 移动到 TCP 位置
                const double boxHalfSize = 0.05; // 5cm 的半边长
                double bounds[6] = {
                    -boxHalfSize, boxHalfSize,
                    -boxHalfSize, boxHalfSize,
                    -boxHalfSize, boxHalfSize
                };
                auto* boxRep = vtkBoxRepresentation::SafeDownCast(
                    m_tcp_gizmo_widget->GetRepresentation());
                if (boxRep != nullptr)
                {
                    boxRep->PlaceWidget(bounds);
                    // 用 SetTransform 将 Gizmo 平移到末端位置
                    vtkNew<vtkTransform> tcpTransform;
                    tcpTransform->Translate(pos[0], pos[1], pos[2]);
                    boxRep->SetTransform(tcpTransform);
                }
            }

            // 启用 Gizmo 使其可交互
            m_tcp_gizmo_widget->SetEnabled(1);
            m_tcp_gizmo_widget->SetProcessEvents(1);
        }
        // ── 【逆向驱动】TCP Gizmo 初始化结束 ──────────────────────

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

// =========================================================================
// 【逆向驱动】滚轮关节角度发射（修复版）
// =========================================================================
void RobotVtkView::EmitJointScroll(double deltaDeg)
{
    const QString& linkName = m_current_picked_link;
    if (linkName.isEmpty())
    {
        return;
    }

    // 中文说明：优先使用 m_link_to_joint_index 映射表查关节索引。
    // 该映射由 RebuildLinkToJointMap() 从 m_currentScene.segments 构建，
    // 以子 link 名称（child_link_name）为键，segment 索引（=joint 索引）为值。
    // 支持任意 link 命名约定（URDF 导入、拓扑构型生成等）。
    int jointIndex = -1;
    auto it = m_link_to_joint_index.find(linkName);
    if (it != m_link_to_joint_index.end())
    {
        jointIndex = it->second;
    }
    else
    {
        // 降级：兼容旧式命名规则 "J1", "J2", ...（拓扑构型生成）
        // 以及 "Link_1", "Link_2", ...（旧版 URDF 导入约定）
        if (linkName.startsWith(QLatin1Char('J')))
        {
            bool ok = false;
            const int idx = linkName.mid(1).toInt(&ok);
            if (ok && idx >= 1) jointIndex = idx - 1;
        }
        else if (linkName.startsWith(QStringLiteral("Link_")))
        {
            bool ok = false;
            const int idx = linkName.mid(5).toInt(&ok);
            if (ok && idx >= 1) jointIndex = idx - 1;
        }
    }

    if (jointIndex >= 0)
    {
        emit signalJointAngleScrolled(jointIndex, deltaDeg);
    }
}

// =========================================================================
// 【逆向驱动】link_name → joint_index 映射表重建
// =========================================================================
void RobotVtkView::RebuildLinkToJointMap()
{
    // 中文说明：遍历所有骨架段，以 child_link_name 为键，segment 索引为值。
    // 对于串联机械臂，segment[i] 的子 link 对应的 joint 索引即为 i。
    m_link_to_joint_index.clear();
    for (int i = 0; i < static_cast<int>(m_currentScene.segments.size()); ++i)
    {
        const auto& segment = m_currentScene.segments[i];
        const QString& childName = segment.child_link_name;
        if (!childName.isEmpty())
        {
            // 如果多个 segment 共享同一个子 link 名（分支拓扑），只保留第一个。
            if (m_link_to_joint_index.find(childName) == m_link_to_joint_index.end())
            {
                m_link_to_joint_index[childName] = i;
            }
        }
    }
}

// =========================================================================
// 【逆向驱动】TCP Gizmo 拖动位姿发射
// =========================================================================
void RobotVtkView::EmitTcpDrag(vtkTransform* transform)
{
#if defined(ROBOSDP_HAVE_VTK)
    if (transform == nullptr) return;

    // 提取 vtkTransform 的位置和 Z-Y-X 欧拉角（VTK 原生格式）
    const double* pos = transform->GetPosition();
    const double* orientation = transform->GetOrientation(); // Z-Y-X 度数

    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = {pos[0], pos[1], pos[2]};
    // 注意：VTK GetOrientation() 返回 Z-Y-X Euler 角（度），
    // 当前 IK 求解器统一使用 RPY 约定，此处直接透传。
    // 若后续需要坐标系转换，在此处增加适配逻辑。
    pose.rpy_deg = {orientation[0], orientation[1], orientation[2]};

    emit signalTcpPoseDragged(pose);
#else
    Q_UNUSED(transform);
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
