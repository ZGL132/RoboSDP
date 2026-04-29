#pragma once

#include "modules/kinematics/dto/UnifiedRobotModelSnapshotDto.h"

#include <QString>

#include <array>
#include <vector>

namespace RoboSDP::Dynamics::Dto
{

/// Dynamics 模块内的安装位姿 DTO。
struct DynamicPoseDto
{
    std::array<double, 3> position_m {0.0, 0.0, 0.0};
    std::array<double, 3> rpy_deg {0.0, 0.0, 0.0};
};

/// Dynamics 模型元信息 DTO。
struct DynamicMetaDto
{
    QString dynamic_id = QStringLiteral("dynamic_001");
    QString name = QStringLiteral("6R 串联动力学模型");
    int version = 1;
    QString source = QStringLiteral("kinematics");
    QString status = QStringLiteral("draft");
    QString kinematic_ref;
    QString topology_ref;
    QString requirement_ref;
    /// @brief 记录当前 Dynamics 草稿依赖的统一机器人模型逻辑引用，便于确认是否与 Kinematics 主链对齐。
    QString unified_robot_model_ref;
    /// @brief 标记上游 Kinematics 是否已经进入共享 Pinocchio 内核，供本模块提示“统一主链是否就绪”。
    bool kinematic_kernel_ready = false;
    /// @brief 记录从 Kinematics 透传下来的主链诊断摘要，避免跨模块排查时丢失语义。
    QString kinematic_conversion_diagnostics;
    /// @brief 保存一份来自 Kinematics 的统一工程主链快照，作为本模块读取上游主链状态的单一入口。
    RoboSDP::Kinematics::Dto::UnifiedRobotModelSnapshotDto unified_robot_snapshot;
};

/// 连杆质量属性 DTO。
struct DynamicLinkDto
{
    QString link_id;
    double mass = 0.0;
    std::array<double, 3> cog {0.0, 0.0, 0.0};
    std::array<double, 6> inertia_tensor {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    QString source_detail = QStringLiteral("estimated_stage1");
};

/// 关节摩擦参数 DTO。
struct JointFrictionDto
{
    double viscous = 0.0;
    double coulomb = 0.0;
    double statik = 0.0;
};

/// 关节传动参数 DTO。
struct DynamicJointDriveDto
{
    QString joint_id;
    double transmission_ratio = 100.0;
    double efficiency = 0.92;
    JointFrictionDto friction;
};

/// 末端执行器质量属性 DTO。
struct EndEffectorDto
{
    double mass = 0.0;
    std::array<double, 3> cog {0.0, 0.0, 0.0};
    std::array<double, 6> inertia_tensor {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

/// 基准轨迹对象 DTO。
struct BenchmarkTrajectoryDto
{
    QString trajectory_id;
    QString name;
    QString profile_type;
    QString active_joint_id;
    std::vector<double> joint_start_deg;
    std::vector<double> joint_target_deg;
    double duration_s = 1.0;
    int sample_count = 64;
};

/**
 * @brief DynamicModel 主 DTO。
 *
 * 本轮只覆盖逆动力学主链所需字段：
 * 1. 重力与安装位姿；
 * 2. 连杆质量属性与关节传动参数；
 * 3. 末端执行器质量；
 * 4. 基准轨迹集合；
 * 5. 与 KinematicModel 的清晰引用关系。
 */
struct DynamicModelDto
{
    DynamicMetaDto meta;
    std::array<double, 3> gravity {0.0, 0.0, -9.81};
    DynamicPoseDto installation_pose;
    std::vector<DynamicLinkDto> links;
    std::vector<DynamicJointDriveDto> joints;
    EndEffectorDto end_effector;
    std::vector<BenchmarkTrajectoryDto> trajectories;

    /// 创建一份可立即进入最小逆动力学主链的默认模型。
    static DynamicModelDto CreateDefault()
    {
        DynamicModelDto dto;
        dto.links = {
            {QStringLiteral("link_1"), 8.0, {0.0, 0.0, 0.18}, {0.08, 0.08, 0.02, 0.0, 0.0, 0.0}, QStringLiteral("estimated_stage1")},
            {QStringLiteral("link_2"), 6.5, {0.15, 0.0, 0.0}, {0.06, 0.05, 0.03, 0.0, 0.0, 0.0}, QStringLiteral("estimated_stage1")},
            {QStringLiteral("link_3"), 5.0, {0.12, 0.0, 0.0}, {0.04, 0.04, 0.02, 0.0, 0.0, 0.0}, QStringLiteral("estimated_stage1")},
            {QStringLiteral("link_4"), 3.5, {0.0, 0.0, 0.06}, {0.02, 0.02, 0.01, 0.0, 0.0, 0.0}, QStringLiteral("estimated_stage1")},
            {QStringLiteral("link_5"), 2.4, {0.0, 0.0, 0.05}, {0.015, 0.015, 0.008, 0.0, 0.0, 0.0}, QStringLiteral("estimated_stage1")},
            {QStringLiteral("link_6"), 1.5, {0.0, 0.0, 0.04}, {0.01, 0.01, 0.005, 0.0, 0.0, 0.0}, QStringLiteral("estimated_stage1")}
        };

        dto.joints = {
            {QStringLiteral("joint_1"), 120.0, 0.92, {0.08, 0.45, 0.50}},
            {QStringLiteral("joint_2"), 120.0, 0.92, {0.08, 0.45, 0.50}},
            {QStringLiteral("joint_3"), 100.0, 0.93, {0.06, 0.35, 0.40}},
            {QStringLiteral("joint_4"), 80.0, 0.94, {0.04, 0.22, 0.25}},
            {QStringLiteral("joint_5"), 80.0, 0.94, {0.04, 0.22, 0.25}},
            {QStringLiteral("joint_6"), 60.0, 0.95, {0.02, 0.12, 0.15}}
        };

        dto.end_effector.mass = 1.2;
        dto.end_effector.cog = {0.0, 0.0, 0.05};
        dto.end_effector.inertia_tensor = {0.005, 0.005, 0.003, 0.0, 0.0, 0.0};
        return dto;
    }
};

} // namespace RoboSDP::Dynamics::Dto
