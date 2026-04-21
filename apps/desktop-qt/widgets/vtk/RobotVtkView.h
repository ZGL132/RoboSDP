#pragma once

#include "modules/kinematics/dto/UrdfPreviewSceneDto.h"

#include <QWidget>

#include <map>

class QLabel;
class QVBoxLayout;

#if defined(ROBOSDP_HAVE_VTK)
#include <vtkSmartPointer.h>

class QVTKOpenGLNativeWidget;
class vtkActor;
class vtkGenericOpenGLRenderWindow;
class vtkRenderer;
#endif

namespace RoboSDP::Desktop::Vtk
{

/**
 * @brief RoboSDP 主窗口的中央 VTK 视图承载控件。
 *
 * 本轮职责：
 * 1. 默认显示最小 VTK 测试场景；
 * 2. 在导入 URDF 后切换到骨架预览场景；
 * 3. 若 VTK 不可用，则稳定退化为文字摘要视图。
 */
class RobotVtkView : public QWidget
{
public:
    explicit RobotVtkView(QWidget* parent = nullptr);
    ~RobotVtkView() override = default;

    /// @brief 刷新中央视图中的 URDF 骨架预览。
    void ShowUrdfPreviewScene(const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& scene);

    /// @brief 高频刷新 Mesh Actor 姿态，仅更新 vtkTransform，不重新加载 STL。
    void UpdatePreviewPoses(
        const std::map<QString, RoboSDP::Kinematics::Dto::CartesianPoseDto>& linkWorldPoses);

    /// @brief 清空当前缓存的 Mesh Actor，通常在重新导入 URDF 或模型结构变化时调用。
    void ClearCache();

    /// @brief 将相机重新对准当前 URDF/测试场景，便于导入后快速找回模型。
    void ResetCameraToCurrentScene();

    /// @brief 设置骨架层显示状态，供顶部视图页签统一控制。
    void SetSkeletonVisible(bool visible);

    /// @brief 设置 visual mesh 层显示状态，供顶部视图页签统一控制。
    void SetVisualMeshVisible(bool visible);

    /// @brief 设置 collision mesh 层显示状态，供顶部视图页签统一控制。
    void SetCollisionMeshVisible(bool visible);

    /// @brief 设置关节轴诊断层显示状态，供顶部视图页签统一控制。
    void SetJointAxesVisible(bool visible);

    /// @brief 设置世界坐标系显示状态，供顶部视图页签统一控制。
    void SetAxesVisible(bool visible);

    /// @brief 设置 Link 标签显示状态，供顶部视图页签统一控制。
    void SetLinkLabelsVisible(bool visible);

    /// @brief 设置 Joint 标签显示状态，供顶部视图页签统一控制。
    void SetJointLabelsVisible(bool visible);

    /// @brief 将相机切换到正视图，沿 -Y 方向观察，Z 轴向上。
    void SetFrontCameraView();

    /// @brief 将相机切换到侧视图，沿 +X 方向观察，Z 轴向上。
    void SetSideCameraView();

    /// @brief 将相机切换到俯视图，沿 +Z 方向观察，Y 轴向上。
    void SetTopCameraView();

    /// @brief 将相机切换到等轴测视图，从 (1, -1, 1) 方向观察。
    void SetIsometricCameraView();

private:
    void BuildLayout();
    void BuildControlBar();
    void BuildVtkView();
    void BuildFallbackView();
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
    bool m_showLinkLabels = true;
    bool m_showJointLabels = true;
    RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto m_currentScene;

#if defined(ROBOSDP_HAVE_VTK)
    QVTKOpenGLNativeWidget* m_vtkWidget = nullptr;
    vtkGenericOpenGLRenderWindow* m_renderWindow = nullptr;
    vtkRenderer* m_renderer = nullptr;
    std::map<QString, vtkSmartPointer<vtkActor>> m_link_actors;
    std::map<QString, RoboSDP::Kinematics::Dto::GeometryObjectDto> m_link_mesh_geometries;
#endif
};

} // namespace RoboSDP::Desktop::Vtk
