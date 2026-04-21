#pragma once

#include "modules/kinematics/dto/KinematicModelDto.h"

#include <QString>

#include <array>
#include <vector>

namespace RoboSDP::Kinematics::Dto
{

/// @brief URDF 预览场景中的节点摘要。
struct UrdfPreviewNodeDto
{
    /// @brief 当前节点对应的 link 名称。
    QString link_name;
    /// @brief 当前 link 在骨架预览中的世界坐标位置，单位 m。
    std::array<double, 3> position_m {0.0, 0.0, 0.0};
    /// @brief 当前 link 在共享物理内核零位上的全局位姿，用于后续 Mesh 与骨架的精确空间绑定。
    CartesianPoseDto world_pose;
};

/// @brief URDF 预览场景中的连杆段摘要。
struct UrdfPreviewSegmentDto
{
    /// @brief 连接父子 link 的 joint 名称。
    QString joint_name;
    /// @brief joint 类型，例如 revolute、fixed。
    QString joint_type;
    /// @brief 父 link 名称。
    QString parent_link_name;
    /// @brief 子 link 名称。
    QString child_link_name;
    /// @brief URDF joint 旋转轴，表达在 joint 局部坐标系中；fixed joint 保留默认 Z 轴。
    std::array<double, 3> joint_axis_xyz {0.0, 0.0, 1.0};
    /// @brief 连杆段起点坐标，通常为父 link 参考点，单位 m。
    std::array<double, 3> start_position_m {0.0, 0.0, 0.0};
    /// @brief 连杆段终点坐标，通常为子 link 参考点，单位 m。
    std::array<double, 3> end_position_m {0.0, 0.0, 0.0};
};

/// @brief URDF 几何体对象摘要 DTO，用于承接 visual/collision 的统一几何语义。
struct GeometryObjectDto
{
    /// @brief 当前几何体挂载到的 link 名称。
    QString link_name;
    /// @brief 几何体类型，例如 mesh、box、cylinder、sphere。
    QString geometry_type;
    /// @brief 当类型为 mesh 时，对应的本地绝对文件路径；若无法解析则为空。
    QString absolute_file_path;
    /// @brief 几何体相对于所属 link 坐标系的局部位姿偏移。
    CartesianPoseDto local_pose;
    /// @brief 三维缩放比例；基础几何体默认为 1,1,1。
    std::array<double, 3> scale {1.0, 1.0, 1.0};
    /// @brief 标记当前几何资源是否可用；mesh 文件缺失时置为 false。
    bool resource_available = true;
    /// @brief 当路径缺失、文件不存在或解析失败时，记录面向 UI/测试的中文说明。
    QString status_message;
};

/// @brief URDF 导入后的最小 3D 骨架预览数据。
struct UrdfPreviewSceneDto
{
    /// @brief 机器人模型名称，优先取 robot/name。
    QString model_name;
    /// @brief 当前预览对应的 URDF 绝对路径。
    QString urdf_file_path;
    /// @brief 骨架中的 link 节点集合。
    std::vector<UrdfPreviewNodeDto> nodes;
    /// @brief 父子 link 之间的骨架线段集合。
    std::vector<UrdfPreviewSegmentDto> segments;
    /// @brief `<visual>` 标签提取出的几何体元数据集合。
    std::vector<GeometryObjectDto> visual_geometries;
    /// @brief `<collision>` 标签提取出的几何体元数据集合。
    std::vector<GeometryObjectDto> collision_geometries;

    bool IsEmpty() const
    {
        return nodes.empty() && segments.empty() &&
               visual_geometries.empty() && collision_geometries.empty();
    }
};

} // namespace RoboSDP::Kinematics::Dto
