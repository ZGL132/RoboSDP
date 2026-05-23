#pragma once

#include "modules/kinematics/dto/KinematicModelDto.h"
#include "modules/kinematics/dto/UrdfPreviewSceneDto.h"
#include "modules/requirement/dto/RequirementModelDto.h"

#include <QWidget>

#include <array>
#include <map>
#include <vector>

class QLabel;
class QCheckBox;
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
class vtkBillboardTextActor3D;
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

    /// @brief 在 3D 视图中显示 Requirement 需求工作空间图层。
    void ShowRequirementWorkspace(const std::vector<std::array<double, 3>>& workspacePoints);

    /// @brief 在 3D 视图中显示 Kinematics 可达工作空间图层。
    void ShowKinematicsWorkspace(const std::vector<std::array<double, 3>>& tcpPositions);

    /// @brief 在 3D 视图中显示带奇异分类的点云（绿=正常，红=奇异）。
    void ShowColoredWorkspacePointCloud(
        const std::vector<std::array<double, 3>>& tcpPositions,
        const std::vector<bool>& isSingular);

    /// @brief 在 3D 视图中显示奇异性分析图层。
    void ShowSingularityWorkspace(
        const std::vector<std::array<double, 3>>& tcpPositions,
        const std::vector<bool>& isSingular);

    /// @brief 在 3D 视图中实时显示 Requirement 关键工位。
    void ShowRequirementKeyPoses(
        const std::vector<RoboSDP::Requirement::Dto::RequirementKeyPoseDto>& keyPoses,
        int selectedIndex);

    /// @brief 显示 IK 目标位姿与实际 TCP 位姿对比层；hasActualPose=false 时仅显示目标。
    void ShowIkPoseComparison(
        const RoboSDP::Kinematics::Dto::CartesianPoseDto& targetPose,
        const RoboSDP::Kinematics::Dto::CartesianPoseDto& actualPose,
        double positionErrorMm,
        double orientationErrorDeg,
        bool withinTolerance,
        bool hasActualPose);

    /// @brief 清除 IK 目标/实际 TCP 对比层。
    void ClearIkPoseComparison();

    void SetRequirementWorkspaceLayerVisible(bool visible);
    void SetRequirementKeyPoseLayerVisible(bool visible);
    bool IsRequirementWorkspaceLayerVisible() const;
    bool IsRequirementKeyPoseLayerVisible() const;

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
    void signalRequirementWorkspaceLayerVisibilityChanged(bool visible);
    void signalRequirementKeyPoseLayerVisibilityChanged(bool visible);

private:
    void BuildLayout();
    void BuildControlBar();
    void BuildVtkView();
    void BuildFallbackView();
    void BuildCornerAxesWidget();
    void RefreshCornerAxesVisibility();
    void RefreshScene(bool resetCamera = true);
    void RenderIkPoseComparisonLayer();
    void ClearIkPoseComparisonActors();
    void BuildAnalysisLayerPanel(QWidget* viewportFrame);
    void RefreshAnalysisLayerPanel();
    void SetAnalysisLayerVisible(const QString& layerId, bool visible);
    bool IsAnalysisLayerVisible(const QString& layerId) const;
    void RenderAnalysisLayers(bool renderNow = true);
    void SetAnalysisLayerVisibleInternal(const QString& layerId, bool visible, bool userInitiated);
    void AutoEnableAnalysisLayerIfDefault(const QString& layerId);
    void RaiseViewOverlays();
    void PositionAnalysisLayerPanel();
    void RenderRequirementKeyPoseLayer();
    void ClearRequirementKeyPoseActors();
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
    std::map<QString, bool> m_analysisLayerVisibility;
    std::map<QString, bool> m_analysisLayerUserOverrides;
    std::map<QString, QCheckBox*> m_analysisLayerChecks;
    bool m_hasIkPoseComparison = false;
    bool m_hasIkActualPose = false;
    bool m_ikWithinTolerance = false;
    double m_ikPositionErrorMm = 0.0;
    double m_ikOrientationErrorDeg = 0.0;
    RoboSDP::Kinematics::Dto::CartesianPoseDto m_ikTargetPose;
    RoboSDP::Kinematics::Dto::CartesianPoseDto m_ikActualPose;
    std::vector<std::array<double, 3>> m_requirementWorkspacePositions;
    std::vector<std::array<double, 3>> m_kinematicsWorkspacePositions;
    std::vector<std::array<double, 3>> m_singularityWorkspacePositions;
    std::vector<bool> m_singularityFlags;
    std::vector<RoboSDP::Requirement::Dto::RequirementKeyPoseDto> m_requirementKeyPoses;
    int m_requirementSelectedKeyPoseIndex = -1;

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

    vtkSmartPointer<vtkActor> m_requirement_workspace_actor;
    vtkSmartPointer<vtkActor> m_kinematics_workspace_actor;
    vtkSmartPointer<vtkActor> m_singularity_workspace_actor;

    vtkSmartPointer<vtkAxesActor> m_ik_target_axes_actor;
    vtkSmartPointer<vtkAxesActor> m_ik_actual_axes_actor;
    vtkSmartPointer<vtkActor> m_ik_error_line_actor;
    vtkSmartPointer<vtkActor> m_ik_target_marker_actor;
    vtkSmartPointer<vtkBillboardTextActor3D> m_ik_target_label_actor;
    vtkSmartPointer<vtkBillboardTextActor3D> m_ik_actual_label_actor;
    vtkSmartPointer<vtkBillboardTextActor3D> m_ik_error_label_actor;
    std::vector<vtkSmartPointer<vtkActor>> m_requirement_key_pose_marker_actors;
    std::vector<vtkSmartPointer<vtkAxesActor>> m_requirement_key_pose_axes_actors;
    std::vector<vtkSmartPointer<vtkBillboardTextActor3D>> m_requirement_key_pose_label_actors;
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

    /// @brief 比例尺半透明覆盖层 QWidget（QPainter 绘制白色刻度线与标签）。
    QWidget* m_scaleBarOverlay = nullptr;
    /// @brief 分析图层开关覆盖层，统一管理工作空间、关键工位、奇异性等分析结果。
    QWidget* m_analysisLayerPanel = nullptr;
    /// @brief 当前比例尺代表的世界距离（米）。
    double m_scaleBarNiceDist = 0.5;
    /// @brief 当前比例尺单位字符串（m / cm / mm）。
    QString m_scaleBarUnit = QStringLiteral("m");

    /// @brief 从 m_currentScene.segments 重建 m_link_to_joint_index 映射表。
    void RebuildLinkToJointMap();

    /// @brief 更新比例尺：根据当前相机参数计算世界距离并刷新底部比例尺覆盖层。
    void UpdateScaleBar();

protected:
    /// @brief 监听 VTK 视口鼠标事件，在旋转/缩放/平移结束后刷新比例尺。
    bool eventFilter(QObject* watched, QEvent* event) override;
};

} // namespace RoboSDP::Desktop::Vtk
