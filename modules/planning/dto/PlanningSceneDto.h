#pragma once

#include "modules/kinematics/dto/UnifiedRobotModelSnapshotDto.h"

#include <QString>

#include <array>
#include <vector>

namespace RoboSDP::Planning::Dto
{

/// 规划场景元信息 DTO。
struct PlanningSceneMetaDto
{
    QString planning_scene_id = QStringLiteral("planning_scene_001");
    QString name = QStringLiteral("6R 点到点规划场景");
    int version = 1;
    QString source = QStringLiteral("planning");
    QString status = QStringLiteral("draft");
    QString kinematic_ref;
    QString dynamic_ref;
    QString selection_ref;
    QString requirement_ref;
    /// @brief 记录当前 Planning 场景依赖的统一机器人模型逻辑引用，便于确认是否与 Kinematics 主链对齐。
    QString unified_robot_model_ref;
    /// @brief 标记上游 Kinematics 是否已经进入共享 Pinocchio 内核，供本模块判断任务验证是否站在统一主链上。
    bool kinematic_kernel_ready = false;
    /// @brief 记录从 Kinematics 透传下来的主链诊断摘要，便于规划失败时快速定位是否是上游主链未就绪。
    QString kinematic_conversion_diagnostics;
    /// @brief 保存一份来自 Kinematics 的统一工程主链快照，作为规划与验证依赖的统一机器人模型摘要入口。
    RoboSDP::Kinematics::Dto::UnifiedRobotModelSnapshotDto unified_robot_snapshot;
};

/// 规划场景中的环境障碍物 DTO。
struct PlanningEnvironmentObjectDto
{
    QString object_id;
    QString name;
    QString shape_type = QStringLiteral("box");
    std::array<double, 3> size_m {0.4, 0.4, 0.2};
    std::array<double, 3> position_m {0.6, 0.0, -0.1};
    std::array<double, 3> rpy_deg {0.0, 0.0, 0.0};
    bool enabled = true;
};

/// 规划阶段使用的关节限位 DTO。
struct PlanningJointLimitDto
{
    QString joint_id;
    double lower_deg = -180.0;
    double upper_deg = 180.0;
};

/// 规划场景中的关节状态 DTO。
struct PlanningJointStateDto
{
    std::vector<QString> joint_ids;
    std::vector<double> joint_values_deg;
};

/// 点到点规划最小配置 DTO。
struct PlanningConfigDto
{
    QString motion_group = QStringLiteral("manipulator");
    QString planner_id = QStringLiteral("RRTConnect");
    QString service_endpoint = QStringLiteral("127.0.0.1:50051");
    double allowed_planning_time_s = 1.0;
    int max_planning_attempts = 1;
    double target_cycle_time_s = 4.0;
    bool check_collision = true;
    bool check_self_collision = true;
};

/**
 * @brief PlanningScene DTO。
 *
 * 本轮只保留第 6 阶段最小闭环需要的字段：
 * 1. 上游模型引用；
 * 2. 关节限位与起终点状态；
 * 3. 最小环境对象列表；
 * 4. 点到点规划配置；
 * 5. 机器人碰撞模型引用。
 */
struct PlanningSceneDto
{
    PlanningSceneMetaDto meta;
    QString robot_collision_model_ref = QStringLiteral("robot_collision_model_6r");
    std::vector<PlanningJointLimitDto> joint_limits;
    PlanningJointStateDto start_state;
    PlanningJointStateDto goal_state;
    std::vector<PlanningEnvironmentObjectDto> environment_objects;
    PlanningConfigDto planning_config;

    static PlanningSceneDto CreateDefault()
    {
        PlanningSceneDto dto;
        dto.joint_limits = {
            {QStringLiteral("joint_1"), -175.0, 175.0},
            {QStringLiteral("joint_2"), -130.0, 80.0},
            {QStringLiteral("joint_3"), -145.0, 145.0},
            {QStringLiteral("joint_4"), -190.0, 190.0},
            {QStringLiteral("joint_5"), -120.0, 120.0},
            {QStringLiteral("joint_6"), -350.0, 350.0}};

        dto.start_state.joint_ids = {
            QStringLiteral("joint_1"),
            QStringLiteral("joint_2"),
            QStringLiteral("joint_3"),
            QStringLiteral("joint_4"),
            QStringLiteral("joint_5"),
            QStringLiteral("joint_6")};
        dto.start_state.joint_values_deg = {0.0, -20.0, 30.0, 0.0, 20.0, 0.0};
        dto.goal_state = dto.start_state;
        dto.goal_state.joint_values_deg = {15.0, -35.0, 45.0, 10.0, 15.0, 20.0};
        dto.environment_objects = {
            {QStringLiteral("table_box"),
             QStringLiteral("工作台占位盒"),
             QStringLiteral("box"),
             {0.8, 0.8, 0.2},
             {0.6, 0.0, -0.15},
             {0.0, 0.0, 0.0},
             true}};
        return dto;
    }
};

} // namespace RoboSDP::Planning::Dto
