#pragma once

#include "modules/kinematics/dto/UrdfPreviewSceneDto.h"

#include <map>

#if defined(ROBOSDP_HAVE_VTK)
#include <vtkSmartPointer.h>

class vtkActor;
class vtkRenderer;
#endif

namespace RoboSDP::Desktop::Vtk
{

/**
 * @brief VTK 最小场景构建器。
 *
 * 本轮只负责构建：
 * 1. 最小测试场景；
 * 2. URDF 导入后的骨架预览场景。
 */
class VtkSceneBuilder
{
public:
#if defined(ROBOSDP_HAVE_VTK)
    /// @brief 构建中央视图的最小测试场景。
    static void BuildMinimalTestScene(vtkRenderer* renderer, bool showAxes = true);

    /// @brief 根据 URDF 骨架预览 DTO 构建中央三维场景。
    static void BuildUrdfPreviewScene(
        vtkRenderer* renderer,
        const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& previewScene,
        bool showLinkLabels,
        bool showJointLabels,
        bool showAxes = true);

    /// @brief 根据 URDF 骨架预览 DTO 构建场景，并复用/填充 Mesh Actor 缓存。
    static void BuildUrdfPreviewScene(
        vtkRenderer* renderer,
        const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& previewScene,
        bool showLinkLabels,
        bool showJointLabels,
        bool showAxes,
        std::map<QString, vtkSmartPointer<vtkActor>>& linkActors,
        std::map<QString, RoboSDP::Kinematics::Dto::GeometryObjectDto>& linkGeometries);

    /// @brief 高频路径：只根据新的 link 全局位姿更新缓存 Actor 的 UserTransform，不重新读 Mesh 文件。
    static bool UpdateCachedMeshActorTransforms(
        const std::map<QString, RoboSDP::Kinematics::Dto::CartesianPoseDto>& linkWorldPoses,
        std::map<QString, vtkSmartPointer<vtkActor>>& linkActors,
        const std::map<QString, RoboSDP::Kinematics::Dto::GeometryObjectDto>& linkGeometries);
#endif
};

} // namespace RoboSDP::Desktop::Vtk
