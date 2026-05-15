#pragma once

#include "modules/kinematics/dto/KinematicModelDto.h"
#include "modules/kinematics/dto/UrdfPreviewSceneDto.h"

#include <QWidget>

#include <map>

class QLabel;
class QVBoxLayout;

// vtkActor / vtkTransform 前置声明放在宏外，因为 HandleActorClicked / EmitTcpDrag
// 的公有 API 使用指针参数，仅前置声明即可通过编译。
class vtkActor;
class vtkTransform;

#if defined(ROBOSDP_HAVE_VTK)
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>

class QVTKOpenGLNativeWidget;
class vtkAxesActor;
class vtkBoxWidget2;
class vtkGenericOpenGLRenderWindow;
class vtkOrientationMarkerWidget;
class vtkRenderer;
class vtkTextActor;
#endif

namespace RoboSDP::Desktop::Vtk
{

/// @brief 当前阶段中央三维主视图区使用的统一预览场景别名；底层暂复用既有 DTO，避免扩大第二阶段范围。
using PreviewSceneDto = RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto;

/**
 * @brief RoboSDP 主窗口的中央 VTK 视图承载控件。
 *
 * 本轮职责：
 * 1. 默认显示最小 VTK 测试场景；
 * 2. 在运动学模块同步预览场景后切换到骨架预览场景；
 * 3. 若 VTK 不可用，则稳定退化为文字摘要视图。
 *
 * 【逆向驱动】新增职责：
 * 4. 关节直接拖动：点击连杆后鼠标滚轮修改关节角度；
 * 5. TCP 3D Gizmo：末端可拖拽坐标轴触发 IK。
 */
class RobotVtkView : public QWidget
{
    Q_OBJECT  // <--- 【新增】启用 Qt 信号/槽机制

public:
    explicit RobotVtkView(QWidget* parent = nullptr);
    ~RobotVtkView() override;

    /**
     * @brief 刷新中央视图中的机器人骨架预览。
     * @param scene 场景数据 DTO。
     * @param resetCamera 是否重置相机视角。如果为 false，则维持当前缩放倍数和观察角度。
     */
    void ShowPreviewScene(const PreviewSceneDto& scene, bool resetCamera = true);
    
    /// @brief 高频刷新 Mesh Actor 姿态，仅更新 vtkTransform，不重新加载 STL。
    void UpdatePreviewPoses(
        const std::map<QString, RoboSDP::Kinematics::Dto::CartesianPoseDto>& linkWorldPoses);

    /// @brief 清空当前缓存的 Mesh Actor，通常在重新导入 URDF 或模型结构变化时调用。
    void ClearCache();

    /// @brief 将相机重新对准当前预览/测试场景，便于同步后快速找回模型。
    void ResetCameraToCurrentScene();

    /// @brief 设置骨架层显示状态，供顶部视图页签统一控制。
    void SetSkeletonVisible(bool visible);

    /// @brief 设置 visual mesh 层显示状态，供顶部视图页签统一控制。
    void SetVisualMeshVisible(bool visible);

    /// @brief 设置 collision mesh 层显示状态，供顶部视图页签统一控制。
    void SetCollisionMeshVisible(bool visible);

    /// @brief 供自定义 VTK 交互器调用，处理用户在 3D 视图中点击 Actor 后的高亮和状态栏输出。
    /// @param clickedActor 被点击的 Actor（nullptr 表示点击空白区域）。
    /// @param pickPosition 拾取位置（世界坐标），用于骨架节点查找，可为 null。
    void HandleActorClicked(vtkActor* clickedActor, const double pickPosition[3] = nullptr);

    // ============================================================
    // 【逆向驱动】接口：供 VtkClickInteractorStyle 回调
    // ============================================================

    /// @brief 获取当前选中的连杆名称（供滚轮关节驱动解析关节索引）。
    QString GetPickedLinkName() const { return m_current_picked_link; }

    /// @brief 获取当前滚轮关节驱动角度步长（度/滚轮格）。
    double GetScrollStep() const { return m_scroll_step_deg; }

    /// @brief 设置滚轮关节驱动的角度步长（度/滚轮格）。
    /// @param stepDeg 步长值（度），建议范围 0.1 ~ 10.0
    void SetScrollStep(double stepDeg) { m_scroll_step_deg = stepDeg; }

    /// @brief 鼠标滚轮滚动时调用，解析连杆名称 → 发射 signalJointAngleScrolled。
    void EmitJointScroll(double deltaDeg);

    /// @brief TCP Gizmo 拖动时调用，提取 vtkTransform 位姿 → 发射 signalTcpPoseDragged。
    void EmitTcpDrag(vtkTransform* transform);

    /// @brief 在 3D 视图中显示工作空间采样点云。
    /// @param tcpPositions 所有可达采样点的 TCP 位置坐标列表，每个元素为 [x, y, z]。
    void ShowWorkspacePointCloud(const std::vector<std::array<double, 3>>& tcpPositions);

    /// @brief 在 3D 视图中显示带奇异分类的点云（绿=正常，红=奇异）。
    void ShowColoredWorkspacePointCloud(
        const std::vector<std::array<double, 3>>& tcpPositions,
        const std::vector<bool>& isSingular);

    /// @brief 设置关节轴诊断层显示状态，供顶部视图页签统一控制。
    void SetJointAxesVisible(bool visible);

    /// @brief 设置世界坐标系显示状态，供顶部视图页签统一控制。
    void SetAxesVisible(bool visible);

    /// @brief 设置地面网格显示状态，供顶部视图页签统一控制。
    void SetGroundGridVisible(bool visible);

    /// @brief 设置屏幕角落方向坐标轴显示状态，供顶部视图页签统一控制。
    void SetCornerAxesVisible(bool visible);

    /// @brief 设置 Link 标签显示状态，供顶部视图页签统一控制。
    void SetLinkLabelsVisible(bool visible);

    /// @brief 设置 Joint 标签显示状态，供顶部视图页签统一控制。
    void SetJointLabelsVisible(bool visible);

    /// @brief 设置 TCP 目标拖拽 Gizmo 显示状态。默认关闭，避免误触发 IK 求解。
    void SetTcpGizmoVisible(bool visible);

    /// @brief 应用设计预设：突出 DH/MDH 骨架、关节和基础参考，不显示 mesh。
    void ApplyDesignViewPreset();

    /// @brief 应用工程预设：显示 visual mesh 与整体姿态，隐藏诊断噪声。
    void ApplyEngineeringViewPreset();

    /// @brief 应用诊断预设：显示骨架、关节轴和标签，便于排查坐标系/关节顺序问题。
    void ApplyDiagnosticViewPreset();

    /// @brief 将相机切换到正视图，沿 -Y 方向观察，Z 轴向上。
    void SetFrontCameraView();

    /// @brief 将相机切换到侧视图，沿 +X 方向观察，Z 轴向上。
    void SetSideCameraView();

    /// @brief 将相机切换到俯视图，沿 +Z 方向观察，Y 轴向上。
    void SetTopCameraView();

    /// @brief 将相机切换到等轴测视图，从 (1, -1, 1) 方向观察。
    void SetIsometricCameraView();

signals:
    /// @brief 3D 视图中点击连杆后鼠标滚轮滚动 → 发出关节角度变更信号。
    void signalJointAngleScrolled(int jointIndex, double deltaDeg);

    /// @brief TCP Gizmo 被拖动 → 发出目标位姿信号，由 KinematicsWidget 的 IK 槽处理。
    void signalTcpPoseDragged(const RoboSDP::Kinematics::Dto::CartesianPoseDto& newPose);

    /// @brief 用户在 3D 视图中拾取到 link，用于属性表格同步选中对应行。
    void signalLinkPicked(const QString& linkName);

private:
    void BuildLayout();
    void BuildControlBar();
    void BuildVtkView();
    void BuildFallbackView();
    void BuildCornerAxesWidget();
    void RefreshCornerAxesVisibility();
    void RefreshScene(bool resetCamera = true);
    void ApplyCameraPreset(
        double directionX,
        double directionY,
        double directionZ,
        double upX,
        double upY,
        double upZ);
    QString BuildStatusText() const;

private:
    QVBoxLayout* m_layout = nullptr;
    QLabel* m_statusLabel = nullptr;
    bool m_showSkeleton = true;
    bool m_showVisualMesh = true;
    bool m_showCollisionMesh = false;
    bool m_showJointAxes = true;
    bool m_showAxes = true;
    bool m_showGroundGrid = true;
    bool m_showCornerAxes = true;
    bool m_showLinkLabels = true;
    bool m_showJointLabels = true;
    bool m_showTcpGizmo = false;
    PreviewSceneDto m_currentScene;

#if defined(ROBOSDP_HAVE_VTK)
    QVTKOpenGLNativeWidget* m_vtkWidget = nullptr;
    vtkGenericOpenGLRenderWindow* m_renderWindow = nullptr;
    vtkRenderer* m_renderer = nullptr;
    vtkSmartPointer<vtkAxesActor> m_corner_axes_actor;
    vtkSmartPointer<vtkOrientationMarkerWidget> m_corner_axes_widget;
    vtkSmartPointer<vtkTextActor> m_watermark_actor; // <--- 【新增】：保存水印对象
    /// @brief 所有 Mesh Actor 映射表：复合键 "layer:link_name:index" → Actor。
    std::map<QString, vtkSmartPointer<vtkActor>> m_link_actors;
    std::map<QString, RoboSDP::Kinematics::Dto::GeometryObjectDto> m_link_mesh_geometries;

    /// @brief 记录上一次被点击高亮的 Actor，用于恢复原始颜色。
    vtkActor* m_last_picked_actor = nullptr;
    /// @brief 上一次被点击 Actor 的原始颜色（R/G/B），用于恢复。
    double m_last_picked_color[3] = {0.0, 0.0, 0.0};
    /// @brief 上一次被点击 Actor 的原始环境光系数，用于恢复。
    double m_last_picked_ambient = 0.0;

    /// @brief TCP 3D Gizmo 控件（vtkBoxWidget2）：在末端生成可拖拽的 3D 变换框，拖动触发 IK。
    vtkSmartPointer<vtkBoxWidget2> m_tcp_gizmo_widget;

    /// @brief TCP 坐标系指示器（vtkAxesActor）：在末端显示红绿蓝三轴，随 FK 结果更新位姿。
    vtkSmartPointer<vtkAxesActor> m_tcp_axes_actor;

    /// @brief 工作空间采样点云 Actor：MC 采样得到的可达 TCP 位置散点图。
    vtkSmartPointer<vtkActor> m_workspace_point_cloud_actor;
#endif

    // ── 以下成员不使用 VTK 类型，放在宏外以便非 VTK 编译路径也能访问 ──

    /// @brief link_name → joint_index 映射表，由 m_currentScene.segments 构建。
    ///        用于 HandleActorClicked → EmitJointScroll 的关节索引查找。
    std::map<QString, int> m_link_to_joint_index;

    /// @brief 当前选中的连杆名称（纯 link_name，用于逆向驱动确定关节索引）。
    QString m_current_picked_link;

    /// @brief 滚轮关节驱动的角度步长（度/滚轮格），默认 1.0°。
    ///        由 KinematicsWidget 的步长控件通过 SetScrollStep() 设置。
    double m_scroll_step_deg = 1.0;

    /// @brief 从 m_currentScene.segments 重建 m_link_to_joint_index 映射表。
    void RebuildLinkToJointMap();
};

} // namespace RoboSDP::Desktop::Vtk
