#pragma once

#include "modules/kinematics/dto/UrdfPreviewSceneDto.h"

#include <QWidget>

#include <map>

class QCheckBox;
class QLabel;
class QHBoxLayout;
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

private:
    void BuildLayout();
    void BuildControlBar();
    void BuildVtkView();
    void BuildFallbackView();
    void RefreshScene();
    QString BuildStatusText() const;

private:
    QVBoxLayout* m_layout = nullptr;
    QLabel* m_statusLabel = nullptr;
    QHBoxLayout* m_toggleLayout = nullptr;
    QCheckBox* m_showAxesCheckBox = nullptr;
    QCheckBox* m_showLinkLabelsCheckBox = nullptr;
    QCheckBox* m_showJointLabelsCheckBox = nullptr;
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
